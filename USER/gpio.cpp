/**
 * \file gpio.cpp
 * \brief Triển khai lớp GpioPin và tiện ích xử lý nút (debounce, press/hold/release).
 */
#include "gpio.h"

// Constructor đơn giản (chưa cấu hình)
GpioPin::GpioPin(GPIO_TypeDef* port, uint16_t pin)
	: port_(port), pin_(pin) {}

// Constructor đầy đủ: cấu hình ngay theo mode/speed (mặc định 50MHz)
GpioPin::GpioPin(GPIO_TypeDef* port, uint16_t pin, GPIO_Mode mode, GPIO_Speed speed)
	: port_(port), pin_(pin)
{
	Configure(mode, speed);
}

void GpioPin::Enable_Clock()
{
	uint32_t rcc = Rcc_From_Port(port_);
	if (rcc) RCC_APB2PeriphClockCmd(rcc, ENABLE);
}

uint32_t GpioPin::Rcc_From_Port(GPIO_TypeDef* port)
{
	if (port == GPIOA) return RCC_APB2Periph_GPIOA;
	if (port == GPIOB) return RCC_APB2Periph_GPIOB;
	if (port == GPIOC) return RCC_APB2Periph_GPIOC;
	if (port == GPIOD) return RCC_APB2Periph_GPIOD;
	if (port == GPIOE) return RCC_APB2Periph_GPIOE;
	if (port == GPIOF) return RCC_APB2Periph_GPIOF;
	if (port == GPIOG) return RCC_APB2Periph_GPIOG;
	// Mặc định an toàn, dù trên F103 thường chỉ A..G
	return 0;
}

void GpioPin::Configure(GPIO_Mode mode, GPIO_Speed speed)
{
	Enable_Clock();
	// AFIO thường cần khi dùng alternate function; bật sẵn khi cần
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef init = {};
	init.GPIO_Pin   = pin_;
	init.GPIO_Mode  = static_cast<GPIOMode_TypeDef>(mode);
	init.GPIO_Speed = static_cast<GPIOSpeed_TypeDef>(speed);
	GPIO_Init(port_, &init);
}

void GpioPin::As_Input_Pull_Up()
{
	Configure(GPIO_Mode::INPUT_PULL_UP);
}

void GpioPin::As_Input_Pull_Down()
{
	Configure(GPIO_Mode::INPUT_PULL_DOWN);
}

void GpioPin::As_Input_Floating()
{
	Configure(GPIO_Mode::INPUT_FLOATING);
}

void GpioPin::As_Output_Push_Pull(GPIO_Speed speed)
{
	Configure(GPIO_Mode::OUT_PUSH_PULL, speed);
}

void GpioPin::As_Output_Open_Drain(GPIO_Speed speed)
{
	Configure(GPIO_Mode::OUT_OPEN_DRAIN, speed);
}

void GpioPin::Toggle()
{
	if (Read_Output()) {
		Set_Low();
	} else {
		Set_High();
	}
}

// ================== BUTTON (POLLING) IMPLEMENTATION ==================
// Thiết kế: gọi Button_Init() một lần, sau đó gọi Button_Tick_1ms() đều đặn mỗi 1ms
// để cập nhật trạng thái và các cờ sự kiện. Các hàm *Was_* đọc cờ và tự xoá cờ.

void GpioPin::Button_Init(bool active_low, uint16_t hold_ms, uint8_t debounce_ms)
{
	button_inited_ = true;
	button_active_low_ = active_low;
	button_hold_ms_ = hold_ms;
	button_debounce_ms_ = debounce_ms == 0 ? 1 : debounce_ms; // tránh 0
	// Reset trạng thái
	Button_Reset();
	// Khuyến nghị cấu hình phần cứng bên ngoài: nếu active_low => kéo lên (pull-up) và nút kéo xuống GND.
}

void GpioPin::Button_Reset()
{
	press_event_ = release_event_ = hold_event_ = false;
	hold_count_ms_ = 0;
	stable_count_ = 0;
	last_button_state_ = button_state_ = false;
}

bool GpioPin::Button_Was_Pressed()
{
	if (!button_inited_) return false;
	bool f = press_event_;
	press_event_ = false;
	return f;
}

bool GpioPin::Button_Was_Released()
{
	if (!button_inited_) return false;
	bool f = release_event_;
	release_event_ = false;
	return f;
}

bool GpioPin::Button_Was_Held()
{
	if (!button_inited_) return false;
	bool f = hold_event_;
	hold_event_ = false;
	return f;
}

void GpioPin::Button_Tick_1ms()
{
	if (!button_inited_) return;

	// Đọc raw theo active level
	bool raw_down = Read_Input();
	if (button_active_low_) raw_down = !raw_down; // nếu active-low thì đảo

	// Debounce: nếu raw giống current stable candidate => tăng stable_count_, else reset
	if (raw_down == last_button_state_) {
		if (stable_count_ < 0xFF) stable_count_++; // tránh overflow
	} else {
		stable_count_ = 0;
		last_button_state_ = raw_down;
	}

	// Khi đủ ms ổn định -> cập nhật trạng thái button_state_
	if (stable_count_ >= button_debounce_ms_) {
		if (button_state_ != raw_down) {
			// Edge
			button_state_ = raw_down;
			if (button_state_) {
				// Press edge
				press_event_ = true;
				hold_count_ms_ = 0; // reset bộ đếm hold bắt đầu tính từ lúc nhấn
			} else {
				// Release edge
				release_event_ = true;
				hold_count_ms_ = 0; // dừng hold
			}
		}
	}

	// Xử lý hold nếu đang nhấn
	if (button_state_) {
		if (hold_count_ms_ < 0xFFFF) hold_count_ms_++;
		if (!hold_event_ && hold_count_ms_ >= button_hold_ms_) {
			hold_event_ = true; // set một lần
		}
	}
}

// ================== SIMPLE POLL (NO DEBOUNCE) ==================
// Button_Check(dt_ms): truyền khoảng thời gian kể từ lần gọi trước (dt_ms),
// đọc chân ngay lập tức và dựa vào thời gian tích luỹ để xác định HOLD.
// Cơ chế:
// - Nếu trước đó up và hiện tại down => trả về PRESS, reset bộ đếm hold.
// - Nếu vẫn đang down và thời gian tích luỹ >= hold_ms_ => trả về HOLD đúng một lần.
// - Các lần khác => NONE.
// Không dùng debounce, phù hợp khi phần cứng đã lọc hoặc không quá nhiễu.
GpioPin::ButtonEvent GpioPin::Button_Check(uint16_t dt_ms)
{
	if (!button_inited_) return ButtonEvent::NONE;

	bool down = Read_Input();
	if (button_active_low_) down = !down;

	ButtonEvent ev = ButtonEvent::NONE;

	if (down) {
		// Đang nhấn
		if (!button_state_) {
			// Cạnh xuống
			ev = ButtonEvent::PRESS;
			hold_count_ms_ = 0; // reset đếm
			hold_event_ = false; // reset cờ hold (dùng lại biến có sẵn)
			button_state_ = true;
		} else {
			// Đã nhấn từ trước, cộng dồn thời gian
			if (hold_count_ms_ < 0xFFFF) hold_count_ms_ += dt_ms;
			if (!hold_event_ && hold_count_ms_ >= button_hold_ms_) {
				ev = ButtonEvent::HOLD;
				hold_event_ = true; // chỉ một lần
			}
		}
	} else {
		// Không nhấn
		button_state_ = false;
		hold_count_ms_ = 0; // reset khi thả
		hold_event_ = false;
	}
	return ev;
}



