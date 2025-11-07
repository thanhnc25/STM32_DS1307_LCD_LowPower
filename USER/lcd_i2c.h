/**
 * \file lcd_i2c.h
 * \brief Điều khiển LCD HD44780 qua PCF8574 (I2C) – đơn giản, dễ dùng.
 */

#include "stm32f10x.h"
#include "i2c.h"
#include <stdint.h>

/** \brief Lớp điều khiển LCD 16x2/20x4 qua expander PCF8574. */
class Lcd_I2c
{
public:
    /** \brief Tạo đối tượng.
     *  \param i2c con trỏ bus I2C đã khởi tạo
     *  \param addr7 địa chỉ 7-bit của PCF8574 (ví dụ 0x27)
     *  \param cols số cột LCD
     *  \param rows số hàng LCD
     */
    Lcd_I2c(I2c *i2c, uint8_t addr7, uint8_t cols, uint8_t rows);

    /** \brief Khởi tạo LCD ở chế độ 4-bit. */
    void Init();
    /** \brief Xoá màn hình. */
    void Clear();
    /** \brief Đưa con trỏ về (0,0). */
    void Home();
    /** \brief Đặt vị trí con trỏ. */
    void Set_Cursor(uint8_t col, uint8_t row);
    /** \brief In chuỗi ASCII. */
    void Print(const char *s);
    /** \brief Gửi lệnh thô. */
    void Command(uint8_t cmd);
    /** \brief Gửi dữ liệu thô. */
    void Data(uint8_t data);
    /** \brief Bật/tắt đèn nền. */
    void Backlight(uint8_t on);
    /** \brief Tạo ký tự custom (CGRAM). */
    void Create_Char(uint8_t location, const uint8_t pattern[8]);

    /** \brief Thử viết 1 byte để kiểm tra thiết bị có ACK không. */
    int Present();
    /** \brief In thời gian hh:mm:ss tại vị trí cho trước. */
    void Print_Time(uint8_t col, uint8_t row,
                    uint8_t hours, uint8_t minutes, uint8_t seconds);
    /** \brief In ngày tháng dd/mm/yy tại vị trí cho trước. */
    void Print_Date(uint8_t col, uint8_t row,
                    uint8_t date, uint8_t month, uint8_t year);

private:
    I2c *i2c_;
    uint8_t addr_;
    uint8_t cols_;
    uint8_t rows_;
    uint8_t backlight_; // 0/1

    void Send_Byte(uint8_t value, uint8_t rs);
    void Write_Nibble(uint8_t nibble_with_ctrl);
    void Delay_Ms(uint32_t ms);
    void Print_Two(uint8_t v);
};
