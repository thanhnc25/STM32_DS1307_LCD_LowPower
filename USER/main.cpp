#include "stm32f10x.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_i2c.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "i2c.h"
#include "lcd_i2c.h"
#include "ds1307.h"
#include "Rotary.h"
#include "uart.h"
#include "gpio.h"
#include "math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

static void Delay_Ms_Nop(uint32_t ms);
static void Task_LCD(void *arg);
enum class RtcCmdType : uint8_t;
struct RtcCmd;
static void Task_RTC(void *arg);
static void Bump_Field(Ds1307_Time &t, uint8_t field, int8_t dir);
static void Task_Input(void *arg);
static void Task_Power(void *arg);
static int parse_uint(const char *s, int maxlen);
static bool Parse_Set_Line(const char *line, Ds1307_Time *out);
static void Task_UART(void *arg);

#define QUEUE_LEN_INPUT 16
#define MUTEX_WAIT_MS 50
#define DS1307_POLL_MS 500
#define LCD_REFRESH_MS 10
#define EDIT_BLINK_MS 120
#define INACTIVITY_SLEEP_MS 10000
#define HOLD_ENTER_EDIT_MS 2000

#define EVT_USER_ACTIVE (1u << 0)
#define EVT_TIME_UPDATED (1u << 1)
#define EVT_EDIT_MODE (1u << 2)
#define EVT_SLEEPING (1u << 3)

static RotaryEncoder *g_encoder = nullptr;

static GpioPin g_pin_lcd(GPIOB, GPIO_Pin_1);
static GpioPin g_pin_rtc(GPIOB, GPIO_Pin_5);
static GpioPin g_pin_btn(GPIOA, GPIO_Pin_0);

static inline void Power_Pins_Init()
{
    g_pin_lcd.As_Output_Push_Pull();
    g_pin_lcd.Set_High();

    g_pin_rtc.As_Output_Push_Pull();
    g_pin_rtc.Set_High();

    g_pin_btn.As_Input_Floating();
    // Cấu hình ngưỡng giữ theo yêu cầu
    g_pin_btn.Button_Init(/*active_low=*/false, /*hold_ms=*/HOLD_ENTER_EDIT_MS);
}
static inline void LCD_Power(bool on)
{
    if (on)
        g_pin_lcd.Set_High();
    else
        g_pin_lcd.Set_Low();
}
static inline void RTC_Power(bool on)
{
    if (on)
        g_pin_rtc.Set_High();
    else
        g_pin_rtc.Set_Low();
}

static EventGroupHandle_t g_evt;
static QueueHandle_t g_input_q;             // queue nhận delta int8 từ ISR encoder
static SemaphoreHandle_t g_i2c1_mtx;        // RTC
static SemaphoreHandle_t g_i2c2_mtx;        // LCD
static SemaphoreHandle_t g_state_mtx;       // bảo vệ state/time chỉnh
static TaskHandle_t g_power_task = nullptr; // handle để suspend/resume Task_Power khi vào/ra chế độ chỉnh

static I2c g_i2c_lcd;
static I2c g_i2c_rtc;
static Lcd_I2c *g_lcd = nullptr;
static Ds1307 *g_rtc = nullptr;

static Ds1307_Time g_time;      // thời gian thực đọc được
static Ds1307_Time g_edit_time; // thời gian đang chỉnh
// Thứ tự chỉnh: 0:hh,1:mm,2:ss,3:day,4:date,5:month,6:year
static uint8_t g_edit_field = 0;
static uint8_t g_in_edit = 0;
static Uart Uart1(USART1, 115200);

static const char *DOW_Name(uint8_t day)
{
    switch (day)
    {
    case 1:
        return "CN"; // Chủ nhật
    case 2:
        return "T2";
    case 3:
        return "T3";
    case 4:
        return "T4";
    case 5:
        return "T5";
    case 6:
        return "T6";
    case 7:
        return "T7";
    default:
        return "--";
    }
}

static void Delay_Ms_Nop(uint32_t ms)
{
    while (ms--)
    {
        volatile uint32_t n = SystemCoreClock / 8000U; // ~1ms
        while (n--)
        {
            __NOP();
        }
    }
}

static void Task_LCD(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    TickType_t last_blink = last;
    uint8_t blink = 0;
    static char last0[17] = {0};
    static char last1[17] = {0};
    for (;;)
    {
        // Chờ sự kiện TIME_UPDATED hoặc refresh theo chu kỳ để nháy
        (void)xEventGroupWaitBits(g_evt, EVT_TIME_UPDATED, pdTRUE, pdFALSE, pdMS_TO_TICKS(LCD_REFRESH_MS));

        Ds1307_Time t;
        uint8_t in_edit;
        uint8_t field;
        Ds1307_Time et;
        xSemaphoreTake(g_state_mtx, portMAX_DELAY);
        t = g_time;
        in_edit = g_in_edit;
        field = g_edit_field;
        et = g_edit_time;
        xSemaphoreGive(g_state_mtx);
        // Cập nhật trạng thái nhấp nháy theo chu kỳ riêng khi đang chỉnh
        TickType_t nowTick = xTaskGetTickCount();
        if (in_edit)
        {
            if ((nowTick - last_blink) >= pdMS_TO_TICKS(EDIT_BLINK_MS))
            {
                blink ^= 1;
                last_blink = nowTick;
            }
        }
        else
        {
            blink = 0; // ngoài chế độ chỉnh: không nháy
        }

        // Tạo chuỗi căn giữa cho dòng 0 (DOW dd/mm/yyyy)
        if (xSemaphoreTake(g_i2c2_mtx, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
        {
            const Ds1307_Time &src = in_edit ? et : t;
            uint16_t full_year = 2000 + src.year;
            const char *dow = DOW_Name(src.day);
            char line0[17];
            char temp0[17];
            // Các phần cần có thể nháy nếu đang chỉnh
            bool hide_day = in_edit && field == 3 && blink;
            bool hide_date = in_edit && field == 4 && blink;
            bool hide_month = in_edit && field == 5 && blink;
            bool hide_year = in_edit && field == 6 && blink;

            char dow_buf[3];
            dow_buf[0] = hide_day ? ' ' : dow[0];
            dow_buf[1] = hide_day ? ' ' : dow[1];
            dow_buf[2] = '\0';

            // Date
            char date_buf[3];
            if (hide_date)
            {
                date_buf[0] = ' ';
                date_buf[1] = ' ';
                date_buf[2] = '\0';
            }
            else
            {
                date_buf[0] = (char)('0' + (src.date / 10));
                date_buf[1] = (char)('0' + (src.date % 10));
                date_buf[2] = '\0';
            }
            // Month
            char month_buf[3];
            if (hide_month)
            {
                month_buf[0] = ' ';
                month_buf[1] = ' ';
                month_buf[2] = '\0';
            }
            else
            {
                month_buf[0] = (char)('0' + (src.month / 10));
                month_buf[1] = (char)('0' + (src.month % 10));
                month_buf[2] = '\0';
            }
            // Year
            char year_buf[5];
            if (hide_year)
            {
                year_buf[0] = year_buf[1] = year_buf[2] = year_buf[3] = ' ';
                year_buf[4] = '\0';
            }
            else
            {
                year_buf[0] = (char)('0' + (full_year / 1000) % 10);
                year_buf[1] = (char)('0' + (full_year / 100) % 10);
                year_buf[2] = (char)('0' + (full_year / 10) % 10);
                year_buf[3] = (char)('0' + (full_year % 10));
                year_buf[4] = '\0';
            }

            // temp string (length variable)
            // Format: DOW space date '/' month '/' year
            snprintf(temp0, sizeof(temp0), "%s %s/%s/%s", dow_buf, date_buf, month_buf, year_buf);
            size_t len0 = strlen(temp0);
            size_t pad0 = (len0 < 16) ? (16 - len0) / 2 : 0;
            memset(line0, ' ', 16);
            line0[16] = '\0';
            for (size_t i = 0; i < len0 && (pad0 + i) < 16; i++)
                line0[pad0 + i] = temp0[i];
            if (strcmp(last0, line0) != 0)
            {
                g_lcd->Set_Cursor(0, 0);
                g_lcd->Print(line0);
                strncpy(last0, line0, 17);
            }

            // Dòng 1: hh:mm:ss căn giữa
            uint8_t hh = src.hours;
            uint8_t mm = src.minutes;
            uint8_t ss = src.seconds;
            bool hide_h = in_edit && field == 0 && blink;
            bool hide_m = in_edit && field == 1 && blink;
            bool hide_s = in_edit && field == 2 && blink;
            char h_buf[3];
            char m_buf[3];
            char s_buf[3];
            if (hide_h)
            {
                h_buf[0] = ' ';
                h_buf[1] = ' ';
                h_buf[2] = '\0';
            }
            else
            {
                h_buf[0] = (char)('0' + (hh / 10));
                h_buf[1] = (char)('0' + (hh % 10));
                h_buf[2] = '\0';
            }
            if (hide_m)
            {
                m_buf[0] = ' ';
                m_buf[1] = ' ';
                m_buf[2] = '\0';
            }
            else
            {
                m_buf[0] = (char)('0' + (mm / 10));
                m_buf[1] = (char)('0' + (mm % 10));
                m_buf[2] = '\0';
            }
            if (hide_s)
            {
                s_buf[0] = ' ';
                s_buf[1] = ' ';
                s_buf[2] = '\0';
            }
            else
            {
                s_buf[0] = (char)('0' + (ss / 10));
                s_buf[1] = (char)('0' + (ss % 10));
                s_buf[2] = '\0';
            }
            char temp1[17];
            snprintf(temp1, sizeof(temp1), "%s:%s:%s", h_buf, m_buf, s_buf);
            size_t len1 = strlen(temp1);
            size_t pad1 = (len1 < 16) ? (16 - len1) / 2 : 0;
            char line1[17];
            memset(line1, ' ', 16);
            line1[16] = '\0';
            for (size_t i = 0; i < len1 && (pad1 + i) < 16; i++)
                line1[pad1 + i] = temp1[i];
            if (strcmp(last1, line1) != 0)
            {
                g_lcd->Set_Cursor(0, 1);
                g_lcd->Print(line1);
                strncpy(last1, line1, 17);
            }

            xSemaphoreGive(g_i2c2_mtx);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(LCD_REFRESH_MS));
    }
}

// Lệnh cho Task RTC
enum class RtcCmdType : uint8_t
{
    NONE = 0,
    SET_TIME
};
struct RtcCmd
{
    RtcCmdType type;
    Ds1307_Time t;
};
static QueueHandle_t g_rtc_cmd_q;

static void Task_RTC(void *arg)
{
    (void)arg;
    if (xSemaphoreTake(g_i2c1_mtx, portMAX_DELAY) == pdTRUE)
    {
        g_rtc->Init();
        (void)g_rtc->Start();
        xSemaphoreGive(g_i2c1_mtx);
    }

    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        // Ưu tiên xử lý lệnh SET_TIME nếu có
        RtcCmd cmd;
        if (xQueueReceive(g_rtc_cmd_q, &cmd, 0) == pdPASS)
        {
            if (cmd.type == RtcCmdType::SET_TIME)
            {
                if (xSemaphoreTake(g_i2c1_mtx, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
                {
                    (void)g_rtc->Set_Time(cmd.t);
                    xSemaphoreGive(g_i2c1_mtx);
                }
            }
        }

        // Đọc thời gian định kỳ (bỏ qua nếu đang ở chế độ chỉnh để tránh chớp hiển thị)
        uint8_t in_edit_snapshot = 0;
        xSemaphoreTake(g_state_mtx, portMAX_DELAY);
        in_edit_snapshot = g_in_edit;
        xSemaphoreGive(g_state_mtx);
        if (!in_edit_snapshot)
        {
            if (xSemaphoreTake(g_i2c1_mtx, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
            {
                Ds1307_Time t;
                if (g_rtc->Get_Time(&t) == SUCCESS)
                {
                    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
                    g_time = t;
                    xSemaphoreGive(g_state_mtx);
                    xEventGroupSetBits(g_evt, EVT_TIME_UPDATED);
                }
                xSemaphoreGive(g_i2c1_mtx);
            }
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(DS1307_POLL_MS));
    }
}

static void Bump_Field(Ds1307_Time &t, uint8_t field, int8_t dir)
{
    switch (field)
    {
    case 0: // hh
        t.hours = (uint8_t)((t.hours + (dir > 0 ? 1 : 23)) % 24);
        break;
    case 1: // mm
        t.minutes = (uint8_t)((t.minutes + (dir > 0 ? 1 : 59)) % 60);
        break;
    case 2: // ss
        t.seconds = (uint8_t)((t.seconds + (dir > 0 ? 1 : 59)) % 60);
        break;
    case 3: // day 1..7
    {
        int v = t.day + (dir > 0 ? 1 : -1);
        if (v < 1)
            v = 7;
        if (v > 7)
            v = 1;
        t.day = (uint8_t)v;
        break;
    }
    case 4:
    {
        uint8_t dim = Ds1307_Days_In_Month(t.month, t.year);
        int v = t.date + (dir > 0 ? 1 : -1);
        if (v < 1)
            v = dim;
        if (v > dim)
            v = 1;
        t.date = (uint8_t)v;
        break;
    }
    case 5: // month 1..12
    {
        int v = t.month + (dir > 0 ? 1 : -1);
        if (v < 1)
            v = 12;
        if (v > 12)
            v = 1;
        t.month = (uint8_t)v;
        uint8_t dim = Ds1307_Days_In_Month(t.month, t.year);
        if (t.date > dim)
            t.date = dim;
        break;
    }
    case 6: // year 0..99
        t.year = (uint8_t)((t.year + (dir > 0 ? 1 : 99)) % 100);
        break;
    }
}

/** \brief Task đọc đầu vào: nhận delta encoder từ queue + đọc nút (PRESS/HOLD). */
static void Task_Input(void *arg)
{
    (void)arg;
    TickType_t last_check = xTaskGetTickCount();
    for (;;)
    {
        // Nhận delta từ ISR (mỗi phần tử queue là int8_t: +1 CW, -1 CCW)
        int8_t delta;
        if (xQueueReceive(g_input_q, &delta, 0) == pdPASS)
        {
            xEventGroupSetBits(g_evt, EVT_USER_ACTIVE);
            if (g_in_edit)
            {
                xSemaphoreTake(g_state_mtx, portMAX_DELAY);
                Bump_Field(g_edit_time, g_edit_field, (delta > 0) ? +1 : -1);
                xSemaphoreGive(g_state_mtx);
            }
        }

        TickType_t now = xTaskGetTickCount();
        uint32_t dt_ms = (uint32_t)pdTICKS_TO_MS(now - last_check);
        if (dt_ms == 0)
            dt_ms = 1; // an toàn
        last_check = now;

        auto bev = g_pin_btn.Button_Check((uint16_t)dt_ms);
        if (bev == GpioPin::ButtonEvent::PRESS)
        {
            xEventGroupSetBits(g_evt, EVT_USER_ACTIVE);
            if (g_in_edit)
            {
                // Nhấn ngắn: chuyển field theo thứ tự hh->mm->ss->day->date->month->year
                xSemaphoreTake(g_state_mtx, portMAX_DELAY);
                g_edit_field = (uint8_t)((g_edit_field + 1) % 7);
                xSemaphoreGive(g_state_mtx);
            }
        }
        else if (bev == GpioPin::ButtonEvent::HOLD)
        {
            xEventGroupSetBits(g_evt, EVT_USER_ACTIVE);
            if (!g_in_edit)
            {
                // Giữ để vào chỉnh giờ
                xSemaphoreTake(g_state_mtx, portMAX_DELAY);
                g_edit_time = g_time;
                g_edit_field = 0;
                g_in_edit = 1;
                xSemaphoreGive(g_state_mtx);
                xEventGroupSetBits(g_evt, EVT_EDIT_MODE);

                if (g_power_task)
                {
                    vTaskSuspend(g_power_task);
                }
            }
            else
            {
                RtcCmd cmd;
                cmd.type = RtcCmdType::SET_TIME;
                xSemaphoreTake(g_state_mtx, portMAX_DELAY);
                // Cập nhật ngay thời gian toàn cục để tránh LCD hiển thị lại thời gian cũ trong 1 chu kỳ refresh
                cmd.t = g_edit_time;
                g_time = g_edit_time; // cập nhật tức thì
                g_in_edit = 0;
                g_edit_field = 0;
                xSemaphoreGive(g_state_mtx);
                xQueueSend(g_rtc_cmd_q, &cmd, 0);
                // Phát sự kiện TIME_UPDATED để Task_LCD render ngay giá trị vừa chỉnh, không chớp về thời gian cũ
                xEventGroupSetBits(g_evt, EVT_TIME_UPDATED);
                // Thoát chỉnh: resume lại task POWER
                if (g_power_task)
                {
                    vTaskResume(g_power_task);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/** \brief Task quản lý năng lượng: theo dõi inactivity và chuyển MCU vào STANDBY. */
static void Task_Power(void *arg)
{
    (void)arg;
    // Bật clock PWR cho chế độ tiết kiệm
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    for (;;)
    {
        // Chờ hoạt động người dùng, timeout 10s
        EventBits_t bits = xEventGroupWaitBits(g_evt, EVT_USER_ACTIVE, pdTRUE, pdFALSE, pdMS_TO_TICKS(INACTIVITY_SLEEP_MS));
        if ((bits & EVT_USER_ACTIVE) == 0)
        {
            xEventGroupSetBits(g_evt, EVT_SLEEPING);

            if (g_lcd)
            {
                if (xSemaphoreTake(g_i2c2_mtx, pdMS_TO_TICKS(MUTEX_WAIT_MS)) == pdTRUE)
                {
                    g_lcd->Backlight(0);
                    xSemaphoreGive(g_i2c2_mtx);
                }
            }

            LCD_Power(false);
            RTC_Power(false);

            g_i2c_lcd.Deinit();
            g_i2c_rtc.Deinit();

            Uart1.Write("Enter STANDBY...\r\n");

            // Chuẩn bị Standby: bật WakeUp pin (PA0 WKUP, sườn lên), xóa cờ
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
            PWR_WakeUpPinCmd(ENABLE);
            PWR_ClearFlag(PWR_FLAG_WU);
            PWR_ClearFlag(PWR_FLAG_SB);

            // Vào Standby (sẽ reset khi thức dậy)
            PWR_EnterSTANDBYMode();

            // Không trở lại đây; nếu có, cứ resume để an toàn
            xTaskResumeAll();
        }
    }
}

// ----------------- UART: ISR và Task UART -----------------
// Định dạng bản tin đặt giờ: "SET,hh:mm:ss,dd/mm/yyyy,day\r\n" (day=1..7, CN=1)
static int parse_uint(const char *s, int maxlen)
{
    int v = 0;
    int i = 0;
    if (!s)
        return -1;
    while (i < maxlen && s[i] >= '0' && s[i] <= '9')
    {
        v = v * 10 + (s[i] - '0');
        ++i;
    }
    return (i == 0) ? -1 : v;
}

static bool Parse_Set_Line(const char *line, Ds1307_Time *out)
{
    if (!line || !out)
        return false;
    if (!(line[0] == 'S' && line[1] == 'E' && line[2] == 'T' && line[3] == ','))
        return false; // Prefix
    const char *p = line + 4;
    int hh = -1, mm = -1, ss = -1, dd = -1, mo = -1, yy = -1, day = -1;
    hh = parse_uint(p, 2);
    if (hh < 0)
        return false;
    while (*p && *p >= '0' && *p <= '9')
        ++p;
    if (*p++ != ':')
        return false;
    mm = parse_uint(p, 2);
    if (mm < 0)
        return false;
    while (*p && *p >= '0' && *p <= '9')
        ++p;
    if (*p++ != ':')
        return false;
    ss = parse_uint(p, 2);
    if (ss < 0)
        return false;
    while (*p && *p >= '0' && *p <= '9')
        ++p;
    if (*p++ != ',')
        return false;
    dd = parse_uint(p, 2);
    if (dd < 0)
        return false;
    while (*p && *p >= '0' && *p <= '9')
        ++p;
    if (*p++ != '/')
        return false;
    mo = parse_uint(p, 2);
    if (mo < 0)
        return false;
    while (*p && *p >= '0' && *p <= '9')
        ++p;
    if (*p++ != '/')
        return false;
    yy = parse_uint(p, 4);
    if (yy < 0)
        return false;
    while (*p && *p >= '0' && *p <= '9')
        ++p;
    if (*p++ != ',')
        return false;
    day = parse_uint(p, 1);
    if (day < 0)
        return false;

    uint8_t y2 = (yy >= 2000) ? (uint8_t)(yy - 2000) : (uint8_t)yy;
    Ds1307_Time tmp;
    tmp.hours = (uint8_t)hh;
    tmp.minutes = (uint8_t)mm;
    tmp.seconds = (uint8_t)ss;
    tmp.date = (uint8_t)dd;
    tmp.month = (uint8_t)mo;
    tmp.year = y2;
    tmp.day = (uint8_t)day;
    if (!Ds1307_Time_Is_Valid(&tmp))
        return false;
    *out = tmp;
    return true;
}

/** \brief Task UART: xử lý dòng nhận được và in thời gian định kỳ. */
static void Task_UART(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        // Xử lý các dòng nhận được từ ISR
        UartLine line;
        while (xQueueReceive(g_uart_line_q, &line, 0) == pdPASS)
        {
            Ds1307_Time nt;
            if (Parse_Set_Line(line.buf, &nt))
            {
                RtcCmd cmd;
                cmd.type = RtcCmdType::SET_TIME;
                cmd.t = nt;
                (void)xQueueSend(g_rtc_cmd_q, &cmd, 0);
                Uart1.Write("OK\r\n");
                xEventGroupSetBits(g_evt, EVT_USER_ACTIVE);
            }
            else
            {
                Uart1.Write("ERR\r\n");
            }
        }

        // Cứ 1s in thời gian hiện tại lên UART
        if (xTaskGetTickCount() - last >= pdMS_TO_TICKS(1000))
        {
            last += pdMS_TO_TICKS(1000);
            Ds1307_Time t;
            xSemaphoreTake(g_state_mtx, portMAX_DELAY);
            t = g_time;
            xSemaphoreGive(g_state_mtx);
            char buf[48];
            uint16_t y = 2000 + t.year;
            snprintf(buf, sizeof(buf), "TIME,%02u:%02u:%02u,%02u/%02u/%04u,%u\r\n",
                     t.hours, t.minutes, t.seconds, t.date, t.month, y, t.day);
            Uart1.Write(buf);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void)
{
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    if (PWR_GetFlagStatus(PWR_FLAG_SB) != RESET)
    {
        PWR_ClearFlag(PWR_FLAG_SB);
        PWR_ClearFlag(PWR_FLAG_WU);
    }

    Power_Pins_Init();
    Delay_Ms_Nop(100); // cho nguồn LCD/RTC ổn định trước khi cấu hình tiếp

    // RTOS objects
    g_evt = xEventGroupCreate();
    g_input_q = xQueueCreate(QUEUE_LEN_INPUT, sizeof(int8_t));
    g_rtc_cmd_q = xQueueCreate(4, sizeof(RtcCmd));
    g_i2c1_mtx = xSemaphoreCreateMutex();
    g_i2c2_mtx = xSemaphoreCreateMutex();
    g_state_mtx = xSemaphoreCreateMutex();

    // Khởi tạo I2C và thiết bị ngoại vi
    g_i2c_lcd.Init(I2c::I2C_BUS2, 100000);
    g_i2c_rtc.Init(I2c::I2C_BUS1, 100000);

    // LCD cố định địa chỉ 0x27 (không cần autodetect)
    static Lcd_I2c s_lcd(&g_i2c_lcd, 0x27, 16, 2);
    g_lcd = &s_lcd;
    g_lcd->Init();
    g_lcd->Backlight(1);
    g_lcd->Clear();

    // Tạo đối tượng RTC (tránh nullptr)
    static Ds1307 s_rtc(&g_i2c_rtc);
    g_rtc = &s_rtc;

    // Encoder trên PA1/PA2: khởi tạo SAU khi set PriorityGroup và tạo queue
    static RotaryEncoder s_encoder(GPIOA, GPIO_Pin_1, GPIO_Pin_2, 100, 4);
    g_encoder = &s_encoder;
    RotaryEncoder::Set_Event_Queue(g_input_q);

    // Queue UART line
    g_uart_line_q = xQueueCreate(8, sizeof(UartLine));
    // Bật interrupt RX cho USART1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 10; // thấp hơn các IRQ quan trọng khác
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // Tạo task với stack thấp hơn để phù hợp heap mặc định
    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(Task_LCD, "LCD", 192, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    ok &= xTaskCreate(Task_RTC, "RTC", 192, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    ok &= xTaskCreate(Task_Input, "INPUT", 256, nullptr, tskIDLE_PRIORITY + 3, nullptr);
    ok &= xTaskCreate(Task_Power, "POWER", 192, nullptr, tskIDLE_PRIORITY + 1, &g_power_task);
    ok &= xTaskCreate(Task_UART, "UART", 256, nullptr, tskIDLE_PRIORITY + 2, nullptr);

    vTaskStartScheduler();
    while (1)
    {
    }
}
