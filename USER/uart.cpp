#include "uart.h"
#include <string.h>

// ----------------- RX theo dòng: queue chia sẻ -----------------
// Lưu queue dùng bởi USART1 IRQ để đẩy UartLine sang Task UART ở main
QueueHandle_t g_uart_line_q = NULL;

// Helper: lấy port mặc định cho TX/RX theo instance
static GPIO_TypeDef *defaultTxPort(USART_TypeDef *u)
{
	if (u == USART1)
		return GPIOA; // PA9
	if (u == USART2)
		return GPIOA; // PA2
	if (u == USART3)
		return GPIOB; // PB10
	return GPIOA;
}
static uint16_t defaultTxPin(USART_TypeDef *u)
{
	if (u == USART1)
		return GPIO_Pin_9; // USART1_TX
	if (u == USART2)
		return GPIO_Pin_2; // USART2_TX
	if (u == USART3)
		return GPIO_Pin_10; // USART3_TX
	return GPIO_Pin_9;
}
static GPIO_TypeDef *defaultRxPort(USART_TypeDef *u)
{
	if (u == USART1)
		return GPIOA; // PA10
	if (u == USART2)
		return GPIOA; // PA3
	if (u == USART3)
		return GPIOB; // PB11
	return GPIOA;
}
static uint16_t defaultRxPin(USART_TypeDef *u)
{
	if (u == USART1)
		return GPIO_Pin_10; // USART1_RX
	if (u == USART2)
		return GPIO_Pin_3; // USART2_RX
	if (u == USART3)
		return GPIO_Pin_11; // USART3_RX
	return GPIO_Pin_10;
}

Uart::Uart(USART_TypeDef *instance, uint32_t baud)
	: usart_(instance)
	, tx_(defaultTxPort(instance), defaultTxPin(instance))
	, rx_(defaultRxPort(instance), defaultRxPin(instance))
{
	// Khởi tạo UART ngay trong constructor với baud mặc định (9600 nếu không truyền)
	Begin(baud);
}

void Uart::Enable_Clocks()
{
	// Clock cho GPIO TX/RX và AFIO đã bật trong GpioPin::Configure
	// Bật clock cho USART
	if (Is_APB2())
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	}
	else
	{
		if (usart_ == USART2)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		if (usart_ == USART3)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	}
}

void Uart::Setup_Default_Pins()
{
	// TX: AF_PP, RX: IN_FLOATING
	tx_.Configure(GPIO_Mode::AF_PUSH_PULL, GPIO_Speed::MHZ_50);
	rx_.Configure(GPIO_Mode::INPUT_FLOATING);
}

bool Uart::Is_APB2() const
{
	return usart_ == USART1;
}

uint32_t Uart::Usart_Clock_APB() const
{
	if (usart_ == USART1)
		return RCC_APB2Periph_USART1;
	if (usart_ == USART2)
		return RCC_APB1Periph_USART2;
	if (usart_ == USART3)
		return RCC_APB1Periph_USART3;
	return 0;
}

void Uart::Setup_Uart(uint32_t baud, uint16_t parity, uint16_t stopBits)
{
	USART_InitTypeDef cfg;
	USART_StructInit(&cfg);
	cfg.USART_BaudRate = baud;
	cfg.USART_WordLength = USART_WordLength_8b;
	cfg.USART_StopBits = stopBits;
	cfg.USART_Parity = parity;
	cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	cfg.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(usart_, &cfg);
	USART_Cmd(usart_, ENABLE);
}

void Uart::Begin(uint32_t baud, uint16_t parity, uint16_t stopBits)
{
	Enable_Clocks();
	Setup_Default_Pins();
	Setup_Uart(baud, parity, stopBits);
}

void Uart::End()
{
	USART_Cmd(usart_, DISABLE);
}

void Uart::Write_Char(char c)
{
	while (USART_GetFlagStatus(usart_, USART_FLAG_TXE) == RESET)
	{
	}
	USART_SendData(usart_, (uint16_t)c);
	while (USART_GetFlagStatus(usart_, USART_FLAG_TC) == RESET)
	{
	}
}

size_t Uart::Write(const uint8_t *data, size_t len)
{
	if (!data || len == 0)
		return 0;
	for (size_t i = 0; i < len; ++i)
	{
		Write_Char((char)data[i]);
	}
	return len;
}

int Uart::Available()
{
	return (USART_GetFlagStatus(usart_, USART_FLAG_RXNE) == SET) ? 1 : 0;
}

int Uart::Read()
{
	if (!Available())
		return -1;
	return (int)(USART_ReceiveData(usart_) & 0xFF);
}

// ----------------- USART1 IRQ chuyển dòng vào queue -----------------
extern "C" void USART1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		static char rxbuf[64];
		static uint8_t idx = 0;
		char c = (char)(USART_ReceiveData(USART1) & 0xFF);
		if (c == '\r' || c == '\n')
		{
			if (idx > 0)
			{
				UartLine line;
				if (idx > 63)
					idx = 63;
				line.len = idx;
				for (uint8_t i = 0; i < idx; ++i)
				{
					line.buf[i] = rxbuf[i];
				}
				line.buf[idx] = '\0';
				idx = 0;
				if (g_uart_line_q)
				{
					xQueueSendFromISR(g_uart_line_q, &line, &xHigherPriorityTaskWoken);
				}
			}
		}
		else
		{
			if (idx < 63)
			{
				rxbuf[idx++] = c;
			}
			else
			{
				idx = 0; // overflow reset
			}
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



