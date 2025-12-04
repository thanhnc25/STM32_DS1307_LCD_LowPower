/**
 * \file uart.h
 * \brief Lớp và hàm hỗ trợ giao tiếp UART cho STM32F103.
 * \details Cung cấp API ghi/đọc kiểu polling cơ bản. Có thể mở rộng thêm bằng ngắt / DMA.
 * Mã nguồn đã chuẩn hoá dấu ngoặc và có chú thích Doxygen bằng tiếng Việt có dấu.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "gpio.h"

// FreeRTOS queue (dùng khi bật nhận dòng qua ngắt). Nếu không dùng FreeRTOS vẫn biên dịch được.
#ifdef __cplusplus
extern "C"
{
#endif
#include "FreeRTOS.h"
#include "queue.h"
#ifdef __cplusplus
}
#endif

/** \brief Giao diện ghi byte trừu tượng để dễ kế thừa. */
struct IByteWriter
{
	virtual ~IByteWriter() {}
	/** \brief Ghi một mảng byte ra thiết bị.
	 *  \param data buffer dữ liệu (có thể nullptr => không ghi)
	 *  \param len số byte
	 *  \return số byte thực sự đã gửi
	 */
	virtual size_t Write(const uint8_t *data, size_t len) = 0;
	size_t Write(const char *s)
	{
		return Write(reinterpret_cast<const uint8_t *>(s), s ? strlen(s) : 0);
	}
};

/** \brief Giao diện đọc byte trừu tượng (polling). */
struct IByteReader
{
	virtual ~IByteReader() {}
	/** \brief Đọc 1 byte nếu có.
	 *  \return giá trị 0..255 hoặc -1 nếu không sẵn sàng.
	 */
	virtual int Read() = 0;
	/** \brief Kiểm tra có byte sẵn sàng.
	 *  \return 0 không có, 1 có.
	 */
	virtual int Available() = 0;
};

/** \brief Bản tin một dòng UART kết thúc bằng CR/LF thu thập từ ISR.
 *  Được queue sang task xử lý ở file main (phần bóc tách giữ nguyên tại main).
 */
struct UartLine
{
	char buf[64];
	uint8_t len; ///< Độ dài chuỗi (không tính '\0')
};

// Queue chia sẻ do main tạo để ISR đẩy dòng nhận được
extern QueueHandle_t g_uart_line_q;

/** \brief Lớp UART cơ bản (polling, blocking).
 *  - Tự cấu hình chân TX/RX mặc định theo instance.
 *  - Hỗ trợ ghi chuỗi/byte đơn giản.
 *  - Có thể mở rộng: kế thừa để thêm RX buffer, interrupt, DMA.
 */
class Uart : public IByteWriter, public IByteReader
{
public:
	/** \brief Khởi tạo đối tượng và tự động gọi Begin với baud mặc định.
	 *  \param instance USART1/USART2/USART3
	 *  \param baud tốc độ baud (mặc định 9600)
	 */
	explicit Uart(USART_TypeDef *instance, uint32_t baud = 9600);

	/** \brief Khởi tạo phần cứng UART.
	 *  \param baud tốc độ baud
	 *  \param parity cấu hình parity (USART_Parity_No ...)
	 *  \param stopBits số bit stop (USART_StopBits_1 ...)
	 */
	virtual void Begin(uint32_t baud,
					   uint16_t parity = USART_Parity_No,
					   uint16_t stopBits = USART_StopBits_1);

	/** \brief Tắt UART (không giải phóng clock). */
	virtual void End();

	// IByteWriter
	virtual size_t Write(const uint8_t *data, size_t len) override;
	size_t Write(const char *s)
	{
		return IByteWriter::Write(s);
	}
	/** \brief Ghi một ký tự đơn.
	 *  Chờ TXE và TC để đảm bảo gửi xong trước khi trả về.
	 */
	void Write_Char(char c);

	// IByteReader
	virtual int Read() override;	  ///< Đọc 1 byte hoặc -1
	virtual int Available() override; ///< 0 hoặc 1 (polling)

	inline USART_TypeDef *raw() const
	{
		return usart_;
	}

protected:
	virtual void Enable_Clocks();
	virtual void Setup_Default_Pins();
	virtual void Setup_Uart(uint32_t baud, uint16_t parity, uint16_t stopBits);
	uint32_t Usart_Clock_APB() const; ///< Giá trị RCC dùng bật clock
	bool Is_APB2() const;			  ///< true nếu nằm trên bus APB2

protected:
	USART_TypeDef *usart_;
	GpioPin tx_;
	GpioPin rx_;
};

/** \brief ISR phần cứng USART1 (định nghĩa tại uart.cpp).
 *  Thu thập ký tự, kết thúc dòng khi gặp CR/LF và gửi sang queue g_uart_line_q.
 *  Việc phân tích nội dung dòng (parse) được giữ ở main như yêu cầu.
 */
extern "C" void USART1_IRQHandler(void);
