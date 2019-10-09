/**
  ******************************************************************************
  * @file    Examples/MDR32F9Q2_EVAL/UART/Interrupt/main.c
  * @author  Milandr Application Team
  * @version V1.2.0
  * @date    04/07/2011
  * @brief   Main program body.
  ******************************************************************************
  * <br><br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MILANDR SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 Milandr</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_board.h"
#include "MDR32F9Qx_config.h"
#include "MDR32Fx.h"
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_it.h"
#define DELAY(T) for (int i = T; i > 0; i--)

static PORT_InitTypeDef PortInit;
static UART_InitTypeDef UART_InitStructure;

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

void setup_ports_for_diods()
{
	PORT_InitTypeDef Nastroyka;
	Nastroyka.PORT_Pin = PORT_Pin_0;
	Nastroyka.PORT_OE = PORT_OE_OUT;
	Nastroyka.PORT_FUNC = PORT_FUNC_PORT;
	Nastroyka.PORT_MODE = PORT_MODE_DIGITAL;
	Nastroyka.PORT_SPEED = PORT_SPEED_SLOW;
	PORT_Init (MDR_PORTC,&Nastroyka);
}

void Uart2PinCfg(void)
{
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_OVERRID;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
    /* Configure PORTF pins 0 (UART2_RX) as input */
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = PORT_Pin_0;
    PORT_Init(MDR_PORTF, &PortInit);
    /* Configure PORTF pins 1 (UART2_TX) as output */
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_1;
    PORT_Init(MDR_PORTF, &PortInit);	
}

void Uart2Setup(void)
{
	/* Select HSI/2 as CPU_CLK source*/
    RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);
    /* Enables the CPU_CLK clock on UART2 */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
    /* Set the HCLK division factor = 1 for UART2*/
	UART_BRGInit(MDR_UART2, UART_HCLKdiv1);

    UART_InitStructure.UART_BaudRate                = 9600;
    UART_InitStructure.UART_WordLength              = UART_WordLength8b;
    UART_InitStructure.UART_StopBits                = UART_StopBits1;
    UART_InitStructure.UART_Parity                  = UART_Parity_No;
    UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
    UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;

	/* Configure UART2 parameters*/
	UART_Init (MDR_UART2,&UART_InitStructure);
    /* Enables UART2 peripheral */
    UART_Cmd(MDR_UART2,ENABLE);
}

int UartTest (void)
{
uint8_t DataByte = 'm';

	Uart2PinCfg();
	Uart2Setup();
 	while(1)
	{
		/* Check RXFF flag*/
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit;
			}
		} 
		while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET);
        /* Send Data */
        UART_SendData (MDR_UART2,DataByte);
		PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
		DELAY(1000000);
		/* Check TXFE flag*/
		goto exit;

	}
exit: return 0;
}

int main (void)
{
RST_CLK_LSEconfig(RST_CLK_LSE_ON);
    while (RST_CLK_LSEstatus() != SUCCESS);
    
	RST_CLK_HSEconfig(RST_CLK_HSE_ON);
    while (RST_CLK_HSEstatus() != SUCCESS);

	/* Enables the clock on PORTA */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);
	/* Enables the clock on PORTB */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE);	
    /* Enables the clock on PORTC */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);
	/* Enables the clock on PORTD */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);
	/* Enables the clock on PORTE */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE, ENABLE);
	/* Enables the HSI clock on PORTF */
    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF, ENABLE);
	/* Enables the HSI clock on ExtBus */
    RST_CLK_PCLKcmd(RST_CLK_PCLK_EBC, ENABLE);
	setup_ports_for_diods();
	while (1)
	{
		PORT_SetBits(MDR_PORTC, PORT_Pin_0);
		DELAY(1000000);
		UartTest();
	}
}

