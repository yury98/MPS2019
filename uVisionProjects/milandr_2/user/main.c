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
void UART2_IRQHandler(void) //?????????? UART1
{
	if (UART_GetITStatusMasked(MDR_UART2, UART_IT_RX) == SET)
	{
		UART_ClearITPendingBit(MDR_UART2, UART_IT_RX);
		data = UART_ReceiveData(MDR_UART2);
		DELAY(10000000);
	}
	if (UART_GetITStatusMasked(MDR_UART2, UART_IT_TX) == SET)
	{
		UART_ClearITPendingBit(MDR_UART2, UART_IT_TX);
		PORT_SetBits(MDR_PORTC, PORT_Pin_0);
	}
}


void UART2_ini(void) //????????????? UART2
{
	UART_InitTypeDef UART_InitStructure;
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
	UART_BRGInit(MDR_UART2, UART_HCLKdiv1);
	NVIC_EnableIRQ(UART2_IRQn);
	UART_InitStructure.UART_BaudRate = 115200;
	UART_InitStructure.UART_WordLength = UART_WordLength8b;
	UART_InitStructure.UART_StopBits = UART_StopBits1;
	UART_InitStructure.UART_Parity = UART_Parity_No;
	UART_InitStructure.UART_FIFOMode = UART_FIFO_OFF;
	UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
	UART_Init (MDR_UART2, &UART_InitStructure);
	UART_ITConfig (MDR_UART2, UART_IT_RX, ENABLE);
	UART_ITConfig (MDR_UART2, UART_IT_TX, ENABLE);
	UART_Cmd(MDR_UART2, ENABLE);
}


void UART2_ports_ini(void)
{
	PORT_InitTypeDef UARTInit;
	PORT_StructInit(&UARTInit);
	UARTInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
	UARTInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	UARTInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
	UARTInit.PORT_PD = PORT_PD_DRIVER;
	UARTInit.PORT_GFEN = PORT_GFEN_OFF;
	UARTInit.PORT_FUNC = PORT_FUNC_ALTER;
	UARTInit.PORT_SPEED = PORT_SPEED_MAXFAST;
	UARTInit.PORT_MODE = PORT_MODE_DIGITAL;
	UARTInit.PORT_OE = PORT_OE_IN;
	UARTInit.PORT_Pin = PORT_Pin_0;
	PORT_Init(MDR_PORTD, &UARTInit);
	UARTInit.PORT_OE = PORT_OE_OUT;
	UARTInit.PORT_Pin = PORT_Pin_1;
	PORT_Init(MDR_PORTD, &UARTInit);
	
}

int main()
{
	setup_ports_for_diods();
	UART2_ports_ini();
	UART2_ini();
	uint16_t count = 5;
	while (1)
	{
		//diod(1);
		//DELAY(1000000);	
		UART_SendData(MDR_UART2, count);
		//PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
		//uint16_t data = UART_ReceiveData(MDR_UART2);
		diod_min(1);
	}
}