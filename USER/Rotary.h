/**
 * \file Rotary.h
 * \brief Driver encoder quay dùng EXTI (ví dụ PA1/PA2) cho STM32F103.
 * \details
 *  - Giải mã theo bảng chuyển trạng thái 4-bit.
 *  - Tích luỹ quarter-step -> detent (mặc định 4 quarter = 1 bước người dùng).
 *  - Debounce bằng chu kỳ DWT (micro giây) để giảm rung tín hiệu.
 *  - ISR gọi ISR_Dispatch() -> Handle_ISR() -> gửi delta (+1/-1) qua queue FreeRTOS.
 */
#pragma once

#include "stm32f10x.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

class RotaryEncoder
{
public:
	/** \brief Khởi tạo encoder.
	 *  \param port Port chứa hai chân A & B.
	 *  \param pinA Chân pha A.
	 *  \param pinB Chân pha B.
	 *  \param debounceUs Thời gian debounce (micro giây).
	 *  \param detentQuarterSteps Số quarter-step trong 1 detent (4 mặc định).
	 */
	explicit RotaryEncoder(GPIO_TypeDef *port,
						   uint16_t pinA,
						   uint16_t pinB,
						   uint32_t debounceUs = 10,
						   uint8_t detentQuarterSteps = 4);

	/** \brief Lấy và xoá số detent tích luỹ kể từ lần gọi trước. */
	int8_t Read_Delta();

	/** \brief Đảo chiều quay (đổi dấu delta). */
	void Set_Reverse(bool reverse)
	{
		reverse_ = reverse;
	}

	/** \brief Gắn queue để ISR đẩy delta (+/-1). */
	static void Set_Event_Queue(QueueHandle_t q)
	{
		s_queue_ = q;
	}

	/** \brief Dispatch từ EXTI IRQ (được gọi trong handler C). */
	static void ISR_Dispatch();

private:
	inline void Handle_ISR(); ///< Xử lý logic trong ngắt

	GPIO_TypeDef *port_;
	uint16_t pin_a_;
	uint16_t pin_b_;

	volatile uint8_t state_ = 0;		///< last 2 states (prev|curr)
	volatile int8_t accum_q_ = 0;		///< quarter-step accumulator
	volatile int16_t steps_ = 0;		///< detent steps accumulator
	volatile uint32_t last_cycles_ = 0; ///< debounce bằng chu kỳ DWT

	uint32_t debounce_us_ = 10;
	uint8_t detent_qsteps_ = 4;
	uint8_t port_source_ = 0; ///< AFIO EXTI port source
	bool reverse_ = false;
	uint32_t cycles_per_us_ = 0;

	void Gpio_And_Exti_Init();
	static int8_t Transition_Step(uint8_t s);

	static RotaryEncoder *s_instance_; ///< Instance duy nhất
	static QueueHandle_t s_queue_;	   ///< Queue nhận delta
};
