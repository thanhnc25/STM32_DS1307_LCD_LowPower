/**
 * \file ds1307.cpp
 * \brief Triển khai điều khiển DS1307 và hàm kiểm tra thời gian hợp lệ.
 */
#include "stm32f10x.h"
#include "ds1307.h"

static uint8_t Bcd_Encode(uint8_t v)
{
    return (uint8_t)(((v / 10) << 4) | (v % 10));
}
static uint8_t Bcd_Decode(uint8_t b)
{
    return (uint8_t)((((b) >> 4) * 10) + ((b) & 0x0F));
}

/** \brief Trả về số ngày trong tháng (1..12). year: 0..99 => 2000..2099. */
uint8_t Ds1307_Days_In_Month(uint8_t month, uint8_t year)
{
    static const uint8_t dim[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8_t d = (month >= 1 && month <= 12) ? dim[month - 1] : 31;
    if (month == 2 && (year % 4) == 0)
    {
        d = 29; // Năm nhuận đơn giản
    }
    return d;
}

/** \brief Kiểm tra cấu trúc thời gian có nằm trong phạm vi hợp lệ. */
uint8_t Ds1307_Time_Is_Valid(const Ds1307_Time *t)
{
    if (!t)
        return 0;
    if (t->hours > 23 || t->minutes > 59 || t->seconds > 59)
        return 0;
    if (t->month < 1 || t->month > 12)
        return 0;
    if (t->day < 1 || t->day > 7)
        return 0;
    if (t->year > 99)
        return 0;
    uint8_t dim = Ds1307_Days_In_Month(t->month, t->year);
    if (t->date < 1 || t->date > dim)
        return 0;
    return 1;
}

ErrorStatus Ds1307::Start()
{
    uint8_t sec;
    if (i2c_->Read_Reg(DS1307_ADDR, 0x00, &sec, 1, 100000) != SUCCESS)
        return ERROR;
    sec &= 0x7F; // CH=0
    if (i2c_->Write_Reg(DS1307_ADDR, 0x00, &sec, 1, 100000) != SUCCESS)
        return ERROR;
    return SUCCESS;
}

ErrorStatus Ds1307::Set_Time(const Ds1307_Time &t)
{
    if (!Ds1307_Time_Is_Valid(&t))
        return ERROR;
    uint8_t buf[7];
    buf[0] = (uint8_t)(Bcd_Encode(t.seconds) & 0x7F); // CH=0
    buf[1] = Bcd_Encode(t.minutes);
    buf[2] = (uint8_t)(Bcd_Encode(t.hours) & 0x3F); // 24h
    buf[3] = Bcd_Encode(t.day);
    buf[4] = Bcd_Encode(t.date);
    buf[5] = Bcd_Encode(t.month);
    buf[6] = Bcd_Encode(t.year);
    return i2c_->Write_Reg(DS1307_ADDR, 0x00, buf, 7, 100000);
}

ErrorStatus Ds1307::Get_Time(Ds1307_Time *t)
{
    // Thử đọc liên tiếp 7 byte trước
    uint8_t buf[7];
    if (i2c_->Read_Reg(DS1307_ADDR, 0x00, buf, 7, 100000) == SUCCESS)
    {
        t->seconds = (uint8_t)(Bcd_Decode(buf[0] & 0x7F));
        t->minutes = Bcd_Decode(buf[1]);
        t->hours = (uint8_t)(Bcd_Decode(buf[2] & 0x3F));
        t->day = Bcd_Decode(buf[3]);
        t->date = Bcd_Decode(buf[4]);
        t->month = Bcd_Decode(buf[5]);
        t->year = Bcd_Decode(buf[6]);
        return SUCCESS;
    }

    // Fallback an toàn: đọc từng byte
    uint8_t b;
    if (i2c_->Read_Reg(DS1307_ADDR, 0x00, &b, 1, 100000) != SUCCESS)
        return ERROR; // seconds
    t->seconds = (uint8_t)(Bcd_Decode(b & 0x7F));

    if (i2c_->Read_Reg(DS1307_ADDR, 0x01, &b, 1, 100000) != SUCCESS)
        return ERROR; // minutes
    t->minutes = Bcd_Decode(b);

    if (i2c_->Read_Reg(DS1307_ADDR, 0x02, &b, 1, 100000) != SUCCESS)
        return ERROR; // hours
    t->hours = (uint8_t)(Bcd_Decode(b & 0x3F));

    if (i2c_->Read_Reg(DS1307_ADDR, 0x03, &b, 1, 100000) != SUCCESS)
        return ERROR; // day
    t->day = Bcd_Decode(b);

    if (i2c_->Read_Reg(DS1307_ADDR, 0x04, &b, 1, 100000) != SUCCESS)
        return ERROR; // date
    t->date = Bcd_Decode(b);

    if (i2c_->Read_Reg(DS1307_ADDR, 0x05, &b, 1, 100000) != SUCCESS)
        return ERROR; // month
    t->month = Bcd_Decode(b);

    if (i2c_->Read_Reg(DS1307_ADDR, 0x06, &b, 1, 100000) != SUCCESS)
        return ERROR; // year
    t->year = Bcd_Decode(b);

    return SUCCESS;
}
