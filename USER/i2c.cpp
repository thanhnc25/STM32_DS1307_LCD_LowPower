/**
 * \file i2c.cpp
 * \brief Triển khai lớp I2c (blocking) cho STM32F103.
 * \details Hỗ trợ Start/Stop, gửi địa chỉ, đọc/ghi buffer và thao tác thanh ghi.
 */
#include "stm32f10x.h"
#include "i2c.h"

#ifndef I2C_DEFAULT_TIMEOUT
#define I2C_DEFAULT_TIMEOUT ((uint32_t)100000)
#endif

void I2c::Gpio_Init_For_Bus(I2C_BUS bus)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    if (bus == I2C_BUS1)
    {
        gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // PB6=SCL, PB7=SDA
    }
    else
    {
        gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PB10=SCL, PB11=SDA
    }
    GPIO_Init(GPIOB, &gpio);
}

void I2c::Gpio_Deinit_For_Bus(I2C_BUS bus)
{
    // Return pins to Analog Input to reduce leakage/current
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    if (bus == I2C_BUS1)
    {
        gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // PB6=SCL, PB7=SDA
    }
    else
    {
        gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PB10=SCL, PB11=SDA
    }
    GPIO_Init(GPIOB, &gpio);
}

void I2c::I2c_Periph_Init(uint32_t clock_hz)
{
    I2C_DeInit(i2c_);
    I2C_InitTypeDef i2c;
    i2c.I2C_ClockSpeed = clock_hz;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(i2c_, &i2c);
    I2C_Cmd(i2c_, ENABLE);
}

void I2c::Init(I2C_BUS bus, uint32_t clock_hz)
{
    bus_ = bus;
    if (bus == I2C_BUS1)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        i2c_ = I2C1;
    }
    else
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        i2c_ = I2C2;
    }
    Gpio_Init_For_Bus(bus);
    I2c_Periph_Init(clock_hz);
}

void I2c::Deinit()
{
    if (!i2c_)
        return;

    // Disable I2C peripheral and reset
    I2C_Cmd(i2c_, DISABLE);
    I2C_DeInit(i2c_);

    // Disable bus clock
    if (i2c_ == I2C1)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
        Gpio_Deinit_For_Bus(I2C_BUS1);
    }
    else if (i2c_ == I2C2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, DISABLE);
        Gpio_Deinit_For_Bus(I2C_BUS2);
    }

    i2c_ = 0;
}

ErrorStatus I2c::Start(uint32_t timeout)
{
    I2C_GenerateSTART(i2c_, ENABLE);
    while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if (timeout-- == 0)
            return ERROR;
    }
    return SUCCESS;
}

ErrorStatus I2c::Send_Address(uint8_t addr7, uint8_t direction, uint32_t timeout)
{
    I2C_Send7bitAddress(i2c_, (uint8_t)(addr7 << 1), direction);
    if (direction == I2C_Direction_Transmitter)
    {
        while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        {
            if (timeout-- == 0)
                return ERROR;
        }
    }
    else
    {
        while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
        {
            if (timeout-- == 0)
                return ERROR;
        }
    }
    return SUCCESS;
}

ErrorStatus I2c::Send_Byte(uint8_t data, uint32_t timeout)
{
    I2C_SendData(i2c_, data);
    while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if (timeout-- == 0)
            return ERROR;
    }
    return SUCCESS;
}

ErrorStatus I2c::Stop()
{
    I2C_GenerateSTOP(i2c_, ENABLE);
    return SUCCESS;
}

ErrorStatus I2c::Write(uint8_t addr7, const uint8_t *data, uint16_t len, uint32_t timeout)
{
    if (Start(timeout) != SUCCESS)
        return ERROR;
    if (Send_Address(addr7, I2C_Direction_Transmitter, timeout) != SUCCESS)
    {
        Stop();
        return ERROR;
    }
    for (uint16_t i = 0; i < len; ++i)
    {
        if (Send_Byte(data[i], timeout) != SUCCESS)
        {
            Stop();
            return ERROR;
        }
    }
    Stop();
    return SUCCESS;
}

ErrorStatus I2c::Read(uint8_t addr7, uint8_t *data, uint16_t len, uint32_t timeout)
{
    if (len == 0)
        return SUCCESS;

    if (Start(timeout) != SUCCESS)
        return ERROR;
    if (Send_Address(addr7, I2C_Direction_Receiver, timeout) != SUCCESS)
    {
        Stop();
        return ERROR;
    }

    if (len == 1)
    {
        // Single byte: NACK then STOP
        I2C_AcknowledgeConfig(i2c_, DISABLE);
        I2C_GenerateSTOP(i2c_, ENABLE);
        while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            if (timeout-- == 0)
                return ERROR;
        }
        *data = I2C_ReceiveData(i2c_);
        I2C_AcknowledgeConfig(i2c_, ENABLE);
        return SUCCESS;
    }
    else if (len == 2)
    {
        // Two bytes: use BTF sequence
        I2C_AcknowledgeConfig(i2c_, DISABLE);
        while (I2C_GetFlagStatus(i2c_, I2C_FLAG_BTF) == RESET)
        {
            if (timeout-- == 0)
                return ERROR;
        }
        I2C_GenerateSTOP(i2c_, ENABLE);
        data[0] = I2C_ReceiveData(i2c_);
        data[1] = I2C_ReceiveData(i2c_);
        I2C_AcknowledgeConfig(i2c_, ENABLE);
        return SUCCESS;
    }
    else
    {
        // len >= 3
        I2C_AcknowledgeConfig(i2c_, ENABLE);
        uint16_t i = 0;
        // Receive bytes until 3 remain
        while ((len - i) > 3)
        {
            while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_BYTE_RECEIVED))
            {
                if (timeout-- == 0)
                    return ERROR;
            }
            data[i++] = I2C_ReceiveData(i2c_);
        }
        // Last 3 bytes sequence
        while (I2C_GetFlagStatus(i2c_, I2C_FLAG_BTF) == RESET)
        {
            if (timeout-- == 0)
                return ERROR;
        }
        I2C_AcknowledgeConfig(i2c_, DISABLE);
        data[i++] = I2C_ReceiveData(i2c_); // N-2
        while (I2C_GetFlagStatus(i2c_, I2C_FLAG_BTF) == RESET)
        {
            if (timeout-- == 0)
                return ERROR;
        }
        I2C_GenerateSTOP(i2c_, ENABLE);
        data[i++] = I2C_ReceiveData(i2c_); // N-1
        while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            if (timeout-- == 0)
                return ERROR;
        }
        data[i++] = I2C_ReceiveData(i2c_); // N
        I2C_AcknowledgeConfig(i2c_, ENABLE);
        return SUCCESS;
    }
}

ErrorStatus I2c::Write_Reg(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len, uint32_t timeout)
{
    if (Start(timeout) != SUCCESS)
        return ERROR;
    if (Send_Address(addr7, I2C_Direction_Transmitter, timeout) != SUCCESS)
    {
        Stop();
        return ERROR;
    }
    if (Send_Byte(reg, timeout) != SUCCESS)
    {
        Stop();
        return ERROR;
    }
    for (uint16_t i = 0; i < len; ++i)
    {
        if (Send_Byte(data[i], timeout) != SUCCESS)
        {
            Stop();
            return ERROR;
        }
    }
    Stop();
    return SUCCESS;
}

ErrorStatus I2c::Read_Reg(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len, uint32_t timeout)
{
    if (Start(timeout) != SUCCESS)
        return ERROR;
    if (Send_Address(addr7, I2C_Direction_Transmitter, timeout) != SUCCESS)
    {
        Stop();
        return ERROR;
    }
    if (Send_Byte(reg, timeout) != SUCCESS)
    {
        Stop();
        return ERROR;
    }

    // Re-START for read
    if (Start(timeout) != SUCCESS)
        return ERROR;
    if (Send_Address(addr7, I2C_Direction_Receiver, timeout) != SUCCESS)
    {
        Stop();
        return ERROR;
    }

    if (len == 0)
        return SUCCESS;
    if (len == 1)
    {
        I2C_AcknowledgeConfig(i2c_, DISABLE);
        I2C_GenerateSTOP(i2c_, ENABLE);
        while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            if (timeout-- == 0)
                return ERROR;
        }
        *data = I2C_ReceiveData(i2c_);
        I2C_AcknowledgeConfig(i2c_, ENABLE);
        return SUCCESS;
    }
    else if (len == 2)
    {
        I2C_AcknowledgeConfig(i2c_, DISABLE);
        while (I2C_GetFlagStatus(i2c_, I2C_FLAG_BTF) == RESET)
        {
            if (timeout-- == 0)
                return ERROR;
        }
        I2C_GenerateSTOP(i2c_, ENABLE);
        data[0] = I2C_ReceiveData(i2c_);
        data[1] = I2C_ReceiveData(i2c_);
        I2C_AcknowledgeConfig(i2c_, ENABLE);
        return SUCCESS;
    }
    else
    {
        I2C_AcknowledgeConfig(i2c_, ENABLE);
        uint16_t i = 0;
        while ((len - i) > 3)
        {
            while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_BYTE_RECEIVED))
            {
                if (timeout-- == 0)
                    return ERROR;
            }
            data[i++] = I2C_ReceiveData(i2c_);
        }
        while (I2C_GetFlagStatus(i2c_, I2C_FLAG_BTF) == RESET)
        {
            if (timeout-- == 0)
                return ERROR;
        }
        I2C_AcknowledgeConfig(i2c_, DISABLE);
        data[i++] = I2C_ReceiveData(i2c_); // N-2
        while (I2C_GetFlagStatus(i2c_, I2C_FLAG_BTF) == RESET)
        {
            if (timeout-- == 0)
                return ERROR;
        }
        I2C_GenerateSTOP(i2c_, ENABLE);
        data[i++] = I2C_ReceiveData(i2c_); // N-1
        while (!I2C_CheckEvent(i2c_, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            if (timeout-- == 0)
                return ERROR;
        }
        data[i++] = I2C_ReceiveData(i2c_); // N
        I2C_AcknowledgeConfig(i2c_, ENABLE);
        return SUCCESS;
    }
}
