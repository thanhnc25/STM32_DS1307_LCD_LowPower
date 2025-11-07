/**
 * \file lcd_i2c.cpp
 * \brief Điều khiển LCD HD44780 qua PCF8574 (I2C) – chế độ 4-bit.
 */
#include "stm32f10x.h"
#include "lcd_i2c.h"
#include <string.h>

// PCF8574 mapping: P7..P4 -> D7..D4, P3 -> BL, P2 -> EN, P1 -> RW, P0 -> RS
#define LCD_BL 0x08
#define LCD_EN 0x04
#define LCD_RW 0x02
#define LCD_RS 0x01

Lcd_I2c::Lcd_I2c(I2c *i2c, uint8_t addr7, uint8_t cols, uint8_t rows)
    : i2c_(i2c), addr_(addr7), cols_(cols), rows_(rows), backlight_(1) {}

void Lcd_I2c::Delay_Ms(uint32_t ms)
{
    volatile uint32_t count;
    while (ms--)
    {
        count = SystemCoreClock / 8000U; // ~1ms loop
        while (count--)
        {
            __NOP();
        }
    }
}

void Lcd_I2c::Write_Nibble(uint8_t nibble_with_ctrl)
{
    uint8_t data = (uint8_t)(nibble_with_ctrl | (backlight_ ? LCD_BL : 0));
    i2c_->Write(addr_, &data, 1, 100000);
    uint8_t data_en = (uint8_t)(data | LCD_EN);
    i2c_->Write(addr_, &data_en, 1, 100000);
    Delay_Ms(2);
    data_en = (uint8_t)(data_en & (uint8_t)(~LCD_EN));
    i2c_->Write(addr_, &data_en, 1, 100000);
    Delay_Ms(2);
}

void Lcd_I2c::Send_Byte(uint8_t value, uint8_t rs)
{
    uint8_t high = (uint8_t)(value & 0xF0);
    uint8_t low = (uint8_t)((value << 4) & 0xF0);
    if (rs)
    {
        high |= LCD_RS;
        low |= LCD_RS;
    }
    Write_Nibble(high);
    Write_Nibble(low);
}

void Lcd_I2c::Command(uint8_t cmd) { Send_Byte(cmd, 0); }
void Lcd_I2c::Data(uint8_t data) { Send_Byte(data, 1); }

void Lcd_I2c::Backlight(uint8_t on)
{
    backlight_ = on ? 1 : 0;
    uint8_t b = (uint8_t)(backlight_ ? LCD_BL : 0);
    i2c_->Write(addr_, &b, 1, 100000);
}

void Lcd_I2c::Clear()
{
    Command(0x01);
    Delay_Ms(2);
}
void Lcd_I2c::Home()
{
    Command(0x02);
    Delay_Ms(2);
}

void Lcd_I2c::Set_Cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= rows_)
        row = (uint8_t)(rows_ - 1);
    Command((uint8_t)(0x80 | (col + row_offsets[row])));
}

void Lcd_I2c::Print(const char *s)
{
    while (s && *s)
    {
        Data((uint8_t)*s++);
    }
}

void Lcd_I2c::Create_Char(uint8_t location, const uint8_t pattern[8])
{
    location &= 0x7;
    Command((uint8_t)(0x40 | (location << 3)));
    for (int i = 0; i < 8; ++i)
    {
        Data(pattern[i]);
    }
}

void Lcd_I2c::Init()
{
    Delay_Ms(50);
    Write_Nibble(0x30);
    Delay_Ms(5);
    Write_Nibble(0x30);
    Delay_Ms(5);
    Write_Nibble(0x30);
    Delay_Ms(5);
    Write_Nibble(0x20);
    Delay_Ms(5);

    Command(0x28); // 4-bit, 2 lines, 5x8
    Command(0x08); // display off
    Clear();
    Command(0x06); // entry mode
    Command(0x0C); // display on
}

int Lcd_I2c::Present()
{
    uint8_t b = (uint8_t)(backlight_ ? LCD_BL : 0);
    return (i2c_->Write(addr_, &b, 1, 100000) == SUCCESS) ? 1 : 0;
}

void Lcd_I2c::Print_Two(uint8_t v)
{
    Data((uint8_t)('0' + (v / 10)));
    Data((uint8_t)('0' + (v % 10)));
}

void Lcd_I2c::Print_Time(uint8_t col, uint8_t row,
                         uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    Set_Cursor(col, row);
    Print_Two(hours);
    Data(':');
    Print_Two(minutes);
    Data(':');
    Print_Two(seconds);
}

void Lcd_I2c::Print_Date(uint8_t col, uint8_t row,
                         uint8_t date, uint8_t month, uint8_t year)
{
    Set_Cursor(col, row);
    Print_Two(date);
    Data('/');
    Print_Two(month);
    Data('/');
    Print_Two(year);
}
