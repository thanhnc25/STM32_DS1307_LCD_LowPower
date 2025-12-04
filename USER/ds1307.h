/**
 * \file ds1307.h
 * \brief Thư viện điều khiển RTC DS1307 (I2C) cho STM32F103.
 * \details Bổ sung hàm kiểm tra hợp lệ thời gian và số ngày trong tháng.
 */
#pragma once

#include "stm32f10x.h"
#include "i2c.h"
#include <stdint.h>

#define DS1307_ADDR 0x68

/** \brief Cấu trúc thời gian chuẩn DS1307 (tất cả ở dạng số thập phân, không phải BCD). */
typedef struct
{
    uint8_t seconds; ///< 0-59
    uint8_t minutes; ///< 0-59
    uint8_t hours;   ///< 0-23 (24h mode)
    uint8_t day;     ///< 1-7 (1=CN)
    uint8_t date;    ///< 1-31
    uint8_t month;   ///< 1-12
    uint8_t year;    ///< 0-99 (tương ứng 2000-2099)
} Ds1307_Time;

/** \brief Tần số ngõ ra SQW (nếu sử dụng). */
enum DS1307_RATE
{
    DS1307_RATE_1HZ = 0,
    DS1307_RATE_4096HZ = 1,
    DS1307_RATE_8192HZ = 2,
    DS1307_RATE_32768HZ = 3
};

/** \brief Trả về số ngày của một tháng, có xét năm nhuận đơn giản (chia hết cho 4). */
uint8_t Ds1307_Days_In_Month(uint8_t month, uint8_t year);

/** \brief Kiểm tra cấu trúc thời gian có hợp lệ hay không.
 *  \return 1 nếu hợp lệ, 0 nếu sai phạm phạm vi.
 */
uint8_t Ds1307_Time_Is_Valid(const Ds1307_Time *t);

class Ds1307
{
public:
    explicit Ds1307(I2c *i2c)
        : i2c_(i2c)
    {
    }
    void Init() { /* no-op */ }
    ErrorStatus Start();                        ///< Bật bit CH=0 để bắt đầu dao động nếu đang dừng.
    ErrorStatus Set_Time(const Ds1307_Time &t); ///< Ghi thời gian (convert sang BCD).
    ErrorStatus Get_Time(Ds1307_Time *t);       ///< Đọc thời gian.

private:
    I2c *i2c_;
};
