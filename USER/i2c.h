/**
 * \file i2c.h
 * \brief Lớp tiện ích I2C (StdPeriph) cho STM32F103: init, read/write, read/write register.
 */
#pragma once

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/** \brief Lớp điều khiển I2C đơn giản, blocking.
 *  Không dùng ngắt/DMA, phù hợp thiết bị tốc độ thấp (DS1307, PCF8574,...)
 */
class I2c
{
public:
    enum I2C_BUS
    {
        I2C_BUS1 = 1,
        I2C_BUS2 = 2
    };

    I2c()
        : i2c_(nullptr), bus_(I2C_BUS2)
    {
    }

    void Init(I2C_BUS bus, uint32_t clock_hz);
    void Deinit();

    ErrorStatus Write(uint8_t addr7, const uint8_t *data, uint16_t len, uint32_t timeout);
    ErrorStatus Read(uint8_t addr7, uint8_t *data, uint16_t len, uint32_t timeout);

    ErrorStatus Write_Reg(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len, uint32_t timeout);
    ErrorStatus Read_Reg(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len, uint32_t timeout);

    I2C_TypeDef *Instance() const { return i2c_; }

private:
    I2C_TypeDef *i2c_;
    I2C_BUS bus_;

    void Gpio_Init_For_Bus(I2C_BUS bus);
    void Gpio_Deinit_For_Bus(I2C_BUS bus);
    void I2c_Periph_Init(uint32_t clock_hz);

    // low-level helpers
    ErrorStatus Start(uint32_t timeout);
    ErrorStatus Send_Address(uint8_t addr7, uint8_t direction, uint32_t timeout);
    ErrorStatus Send_Byte(uint8_t data, uint32_t timeout);
    ErrorStatus Stop();
};

#endif // __cplusplus
