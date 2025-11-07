/**
 * \file Rotary.cpp
 * \brief Triển khai driver encoder quay dựa trên EXTI và bảng chuyển trạng thái.
 */
// STM32F103C8 Rotary encoder using EXTI on two pins
#include "Rotary.h"

static inline uint8_t Port_To_AFIO_Source(GPIO_TypeDef *port)
{
    if (port == GPIOA)
        return GPIO_PortSourceGPIOA;
    if (port == GPIOB)
        return GPIO_PortSourceGPIOB;
    if (port == GPIOC)
        return GPIO_PortSourceGPIOC;
    if (port == GPIOD)
        return GPIO_PortSourceGPIOD;
    return GPIO_PortSourceGPIOA;
}

RotaryEncoder *RotaryEncoder::s_instance_ = 0;
QueueHandle_t RotaryEncoder::s_queue_ = 0;

static inline int8_t TransitionTable(uint8_t s)
{
    // Same mapping as Arduino code
    static const int8_t T[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0};
    return T[s & 0x0F];
}

RotaryEncoder::RotaryEncoder(GPIO_TypeDef *port, uint16_t pinA, uint16_t pinB,
                             uint32_t debounceUs, uint8_t detentQuarterSteps)
    : port_(port), pin_a_(pinA), pin_b_(pinB)
{
    debounce_us_ = debounceUs;
    detent_qsteps_ = (detentQuarterSteps == 0) ? 4 : detentQuarterSteps;
    cycles_per_us_ = SystemCoreClock / 1000000UL;
    // Bật DWT CYCCNT để debounce theo thời gian; nếu không bật, CYCCNT=0 sẽ chặn mọi sự kiện
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    port_source_ = Port_To_AFIO_Source(port_);
    Gpio_And_Exti_Init();

    // Initialize state from current pins
    uint8_t a = (uint8_t)((port_->IDR & pin_a_) ? 1 : 0);
    uint8_t b = (uint8_t)((port_->IDR & pin_b_) ? 1 : 0);
    uint8_t ab = (uint8_t)((a << 1) | b);
    state_ = (uint8_t)(((ab & 0x03) | (ab << 2)) & 0x0F);

    s_instance_ = this;
}

void RotaryEncoder::Gpio_And_Exti_Init()
{
    // Enable clocks
    if (port_ == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (port_ == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (port_ == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Configure pins as input pull-up
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = pin_a_;
    GPIO_Init(port_, &gpio);
    gpio.GPIO_Pin = pin_b_;
    GPIO_Init(port_, &gpio);

    // Map to EXTI lines: only support PA1/PA2 default use-case
    uint8_t line_a = (pin_a_ == GPIO_Pin_1) ? 1 : 0;
    uint8_t line_b = (pin_b_ == GPIO_Pin_2) ? 2 : 0;
    if (line_a == 1)
        GPIO_EXTILineConfig(port_source_, GPIO_PinSource1);
    if (line_b == 2)
        GPIO_EXTILineConfig(port_source_, GPIO_PinSource2);

    EXTI_InitTypeDef exti;
    EXTI_StructInit(&exti);
    if (line_a == 1)
    {
        exti.EXTI_Line = EXTI_Line1;
        exti.EXTI_Mode = EXTI_Mode_Interrupt;
        exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        exti.EXTI_LineCmd = ENABLE;
        EXTI_Init(&exti);
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
    if (line_b == 2)
    {
        exti.EXTI_Line = EXTI_Line2;
        exti.EXTI_Mode = EXTI_Mode_Interrupt;
        exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        exti.EXTI_LineCmd = ENABLE;
        EXTI_Init(&exti);
        EXTI_ClearITPendingBit(EXTI_Line2);
    }

    NVIC_InitTypeDef nvic;
    // Ưu tiên ngắt đủ thấp để gọi FreeRTOS FromISR (với PriorityGroup_4 và MAX_SYSCALL ~ 128)
    // Trên STM32F1: số càng nhỏ ưu tiên càng cao. Chọn 11 (>=8) là an toàn.
    nvic.NVIC_IRQChannelPreemptionPriority = 11;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    if (line_a == 1)
    {
        nvic.NVIC_IRQChannel = EXTI1_IRQn;
        NVIC_Init(&nvic);
    }
    if (line_b == 2)
    {
        nvic.NVIC_IRQChannel = EXTI2_IRQn;
        NVIC_Init(&nvic);
    }
}

void RotaryEncoder::ISR_Dispatch()
{
    if (s_instance_)
        s_instance_->Handle_ISR();
}

inline void RotaryEncoder::Handle_ISR()
{
    // DWT-based debounce if enabled
    uint32_t now = DWT->CYCCNT;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) && cycles_per_us_ != 0)
    {
        uint32_t min_delta = debounce_us_ * cycles_per_us_;
        uint32_t delta = now - last_cycles_;
        if (delta < min_delta)
            return;
        last_cycles_ = now;
    }
    else
    {
        // Không dùng debounce nếu CYCCNT không chạy
        last_cycles_ = now;
    }

    uint8_t a = (uint8_t)((port_->IDR & pin_a_) ? 1 : 0);
    uint8_t b = (uint8_t)((port_->IDR & pin_b_) ? 1 : 0);
    uint8_t ab = (uint8_t)((a << 1) | b);
    state_ = (uint8_t)(((state_ << 2) | (ab & 0x03)) & 0x0F);
    int8_t step = TransitionTable(state_);
    if (reverse_)
        step = (int8_t)-step;
    if (step != 0)
    {
        accum_q_ = (int8_t)(accum_q_ + step);
        if (accum_q_ >= (int8_t)detent_qsteps_)
        {
            steps_++;
            accum_q_ = 0;
            if (s_queue_) {
                int8_t delta = +1;
                BaseType_t hpw = pdFALSE;
                xQueueSendFromISR(s_queue_, &delta, &hpw);
                portYIELD_FROM_ISR(hpw);
            }
        }
        else if (accum_q_ <= -(int8_t)detent_qsteps_)
        {
            steps_--;
            accum_q_ = 0;
            if (s_queue_) {
                int8_t delta = -1;
                BaseType_t hpw = pdFALSE;
                xQueueSendFromISR(s_queue_, &delta, &hpw);
                portYIELD_FROM_ISR(hpw);
            }
        }
    }
}

int8_t RotaryEncoder::Read_Delta()
{
    __disable_irq();
    int16_t d = steps_;
    steps_ = 0;
    __enable_irq();
    if (d > 127)
        d = 127;
    if (d < -128)
        d = -128;
    return (int8_t)d;
}

extern "C" void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);
        RotaryEncoder::ISR_Dispatch();
    }
}

extern "C" void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line2);
        RotaryEncoder::ISR_Dispatch();
    }
}