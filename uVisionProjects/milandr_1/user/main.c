#include "MDR32Fx.h"                    // Device header
#include "MDR32F9Qx_config.h"           // Keil::Device:Startup
#include "MDR32F9Qx_port.h"             // Keil::Drivers:PORT
#include "MDR32F9Qx_rst_clk.h"          // Keil::Drivers:RST_CLK
#include "MDR32F9Qx_uart.h"             // Keil::Drivers:UART

#define DELAY(T) for (int i = T; i > 0; i--)

void diod(int num)
{
	for (int m = num; m > 0; m--)
	{
		PORT_SetBits(MDR_PORTC, PORT_Pin_0);
		DELAY(1000000);
		PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
		DELAY(1000000);	
	}
}

void diod_min(int num)
{
	for (int m = num; m > 0; m--)
	{
		PORT_SetBits(MDR_PORTC, PORT_Pin_0);
		DELAY(100000);
		PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
		DELAY(100000);	
	}
}

void setup_ports_for_diods()
{
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);
	PORT_InitTypeDef Nastroyka;
	Nastroyka.PORT_Pin = PORT_Pin_0;
	Nastroyka.PORT_OE = PORT_OE_OUT;
	Nastroyka.PORT_FUNC = PORT_FUNC_PORT;
	Nastroyka.PORT_MODE = PORT_MODE_DIGITAL;
	Nastroyka.PORT_SPEED = PORT_SPEED_SLOW;
	PORT_Init (MDR_PORTC,&Nastroyka);
}

uint16_t data;
void UART1_IRQHandler(void)
{
	//diod_min(1);
	if (UART_GetITStatusMasked(MDR_UART1, UART_IT_RX) == SET)
	{
		UART_ClearITPendingBit(MDR_UART1, UART_IT_RX);
		data = UART_ReceiveData(MDR_UART1);
		diod_min(data);
	}
	
}

void UART_ini()
{
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);
	
	PORT_InitTypeDef Nastroyka;
	Nastroyka.PORT_Pin = PORT_Pin_7;
	Nastroyka.PORT_OE = PORT_OE_OUT;
	Nastroyka.PORT_PULL_UP = PORT_PULL_UP_OFF;
	Nastroyka.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	Nastroyka.PORT_PD_SHM = PORT_PD_SHM_OFF;
	Nastroyka.PORT_PD = PORT_PD_DRIVER;
	Nastroyka.PORT_GFEN = PORT_GFEN_OFF;
	Nastroyka.PORT_FUNC = PORT_FUNC_OVERRID;
	Nastroyka.PORT_MODE = PORT_MODE_DIGITAL;
	Nastroyka.PORT_SPEED = PORT_SPEED_FAST;
	
	PORT_Init (MDR_PORTA,&Nastroyka);
	
	Nastroyka.PORT_Pin = PORT_Pin_6;
	Nastroyka.PORT_OE = PORT_OE_IN;
	
	PORT_Init (MDR_PORTA,&Nastroyka);
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART1, ENABLE);
	
	NVIC_EnableIRQ(UART1_IRQn);
	
	UART_BRGInit(MDR_UART1, UART_HCLKdiv1);
	
	UART_InitTypeDef uart_user_ini;
	uart_user_ini.UART_BaudRate = 9600;
	uart_user_ini.UART_FIFOMode = UART_FIFO_OFF;
	uart_user_ini.UART_HardwareFlowControl = UART_HardwareFlowControl_TXE|UART_HardwareFlowControl_RXE;
	uart_user_ini.UART_Parity = UART_Parity_0;
	uart_user_ini.UART_StopBits = UART_StopBits1;
	uart_user_ini.UART_WordLength = UART_WordLength8b;
	
	UART_Init(MDR_UART1, &uart_user_ini);
	
	UART_ITConfig(MDR_UART1, UART_IT_RX, ENABLE);
	
	UART_Cmd(MDR_UART1, ENABLE);
}

int main()
{
	setup_ports_for_diods();
	UART_ini;
	uint8_t count = 5;
	while (1)
	{
		//diod(1);
		//DELAY(1000000);	
		UART_SendData(MDR_UART1, count);
	}
}