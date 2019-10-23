/**
  ******************************************************************************
  * @file    Examples/UART/Example1/main.c
  * @author  Phyton Application Team
  * @version V1.0.0
  * @date    16.06.2010
  * @brief   Main program body.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 Phyton</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "1986be9x_config.h"
#include "1986BE9x.h"
#include "1986BE9x_uart.h"
#include "1986BE9x_port.h"
#include "1986BE9x_rst_clk.h"
#include "1986BE9x_it.h"
#include "mlt_lcd.h"
//#include "font.h"
#include "MilFlash.h"

/** @addtogroup __1986BE9x_StdPeriph_Examples 1986BE9x StdPeriph Examples
  * @{
  */

/** @addtogroup PORT_Example PORT Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define Ram
#define BLINK_NUM 50
#define BLINK_DELAY 20000

#define LED1            PORT_Pin_0
#define LED2            PORT_Pin_1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int count_title = 0;
int MenuMainItem = 0;
int CursorPosItem = 0;

char TestLedDone = 0;
char UartFlag = 0;
char Can1Flag = 0;
char Can2Flag = 0;
char LedFlag = 0;
static PORT_InitTypeDef PortInit;
static UART_InitTypeDef UART_InitStructure;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Delay(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}

void LEDOn(uint32_t LED_Num)
{
  PORT_SetBits(PORTC, LED_Num);
}

void LEDOff(uint32_t LED_Num)
{
  PORT_ResetBits(PORTC, LED_Num);
}

void BlinkLED1(uint32_t num, uint32_t del,uint32_t LED_Num)
{
  uint32_t cnt;
  for ( cnt = 0; cnt < num; cnt++)
  {
    LEDOn(LED_Num);
    Delay(del);
    LEDOff(LED_Num);
    Delay(del);
  }
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
    PORT_Init(PORTF, &PortInit);
    /* Configure PORTF pins 1 (UART2_TX) as output */
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_1;
    PORT_Init(PORTF, &PortInit);	
}
void Can2PinCfg(void)
{
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_ALTER;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
    /* Configure PORTA pins 7 (CAN2_RX) as input */
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = PORT_Pin_7;
    PORT_Init(PORTA, &PortInit);
    /* Configure PORTA pins 6 (CAN2_TX) as output */
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_6;
    PORT_Init(PORTA, &PortInit);	
}
void MltPinCfg (void)
{
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
	/* Configure PORTA pins 0..5 for mlt inout data  */
	PortInit.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3 | PORT_Pin_4 | PORT_Pin_5);
	PortInit.PORT_OE    = PORT_OE_IN;
	PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
 	PORT_Init(PORTA, &PortInit);
	/* Configure PORTF pins 2,3 for mlt inout data  */
	PortInit.PORT_Pin   = (PORT_Pin_2 | PORT_Pin_3);
	PORT_Init(PORTF, &PortInit);

	/* Configure PORTB pins 7..10 for mlt output  */
	PortInit.PORT_Pin   = (PORT_Pin_7 | PORT_Pin_8 | PORT_Pin_9 | PORT_Pin_10);
	PortInit.PORT_OE    = PORT_OE_OUT;

	PORT_Init(PORTB, &PortInit);

	/* Configure PORTC pins 0,1 for mlt output */
	PortInit.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1);

	PORT_Init(PORTC, &PortInit);
}

void LedPinGfg (void)
{
	/* Configure PORTC pins 0,1 for output to switch LEDs on/off */
	PortInit.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;

	PORT_Init(PORTC, &PortInit);
}

void ButtonPinGfg (void)
{
	/* Configure PORTB pins 5,6 for input Buttons  */
	PortInit.PORT_Pin   = (PORT_Pin_5 | PORT_Pin_6);
	PortInit.PORT_OE    = PORT_OE_IN;
	PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;

	PORT_Init(PORTB, &PortInit);
	/* Configure PORTC pins 2 for input Buttons  */
    PortInit.PORT_Pin   = (PORT_Pin_2);

	PORT_Init(PORTC, &PortInit);
	/* Configure PORTE pins 1,3 for input Buttons  */
    PortInit.PORT_Pin   = (PORT_Pin_1 | PORT_Pin_3);

	PORT_Init(PORTE, &PortInit);
}
	
void Uart2Setup(void)
{
	/* Select HSI/2 as CPU_CLK source*/
    RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);
    /* Enables the CPU_CLK clock on UART2 */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
    /* Set the HCLK division factor = 1 for UART2*/
	UART_BRGInit(UART2, UART_HCLKdiv1);

    UART_InitStructure.UART_BaudRate                = 9600;
    UART_InitStructure.UART_WordLength              = UART_WordLength8b;
    UART_InitStructure.UART_StopBits                = UART_StopBits1;
    UART_InitStructure.UART_Parity                  = UART_Parity_No;
    UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
    UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;

	/* Configure UART2 parameters*/
	UART_Init (UART2,&UART_InitStructure);
    /* Enables UART2 peripheral */
    UART_Cmd(UART2,ENABLE);
}
void Can2Setup(int rate)
{
   
	//* Select HSE/2 as CPU_CLK source
	RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSEdiv1,RST_CLK_CPU_PLLmul4);
    RST_CLK_CPUclkPrescaler(RST_CLK_CPUclkDIV1);
    RST_CLK_CPU_PLLcmd(ENABLE);
    while (RST_CLK_CPU_PLLstatus() == 0)//wait ready PLL
	RST_CLK_CPU_PLLuse(ENABLE);
    RST_CLK_CPUclkSelection(RST_CLK_CPUclkCPU_C3);   	

    /* Enables the CPU_CLK clock on CAN1 */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_CAN1, ENABLE);
	/* */
	RST_CLK->CAN_CLOCK = 0x01000000;
	/* Timing setup */
	if (rate == 1) CAN1->BITTMNG = 1 << 27 | (3 << 25) | ((5 - 1) << 22) | ((5 - 1) << 19) | ((2 - 1) << 16) | 4; //125
	if (rate == 2) CAN1->BITTMNG = 1 << 27 | ((2-1)<<25) | ((2-1)<<22) | ((3-1)<<19) | ((2-1)<<16) | (2-1); //500
	/* TX buffer 1*/
  	CAN1->BUF_01_CON    = 0x00000000;
	CAN1->BUF_01_MASK   = 0x00000000;
	CAN1->BUF_01_FILTER = 0x00000000;	
	/* RX buffer 2*/
  	CAN1->BUF_02_CON    = 0x00000003;
	CAN1->BUF_02_ID     = 0x00000000;
	CAN1->BUF_02_DLC    = 0x00000000;
	CAN1->BUF_02_DATAL  = 0x00000000;
	CAN1->BUF_02_DATAH  = 0x00000000;
	CAN1->BUF_02_MASK   = 0x00000000;
	CAN1->BUF_02_FILTER = 0x00000000;
	/* Enable CAN1 */
	CAN1->CONTROL = 0x00000001;
}
void Tim1Setup(void)
{
//	vu16 TIMx_ARR, TIMx_CNT, TIMx_PSG, TIMx_CNTRL; 
//	vu16 value;
//	vu16 TIMx_CCRy[4];

	/* Enables the clock on  TIMER1*/
	RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER1, ENABLE);
	RST_CLK->TIM_CLOCK = 0x07000000;

	TIMER1->CNTRL = 0x00000000;

	TIMER1->CNT = 0x00000000;	//start count value
	TIMER1->PSG = 0x0000003F;	//div main clk
	TIMER1->ARR = 0x0000FFFF;	//end count value

	TIMER1->CH1_CNTRL = 0x00000000;
	TIMER1->CH2_CNTRL = 0x00000000;
	TIMER1->CH3_CNTRL = 0x00000000;
	TIMER1->CH4_CNTRL = 0x00000000;

	TIMER1->CH1_CNTRL1 = 0x00000000;
	TIMER1->CH2_CNTRL1 = 0x00000000;
	TIMER1->CH3_CNTRL1 = 0x00000000;
	TIMER1->CH4_CNTRL1 = 0x00000000;

	TIMER1->STATUS = 0x00000000;
	TIMER1->IE  = 0x00000002;
	TIMER1->CNTRL  |= 0x00000001;
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
			if (PORT_ReadInputDataBit(PORTE,PORT_Pin_3) == 0)
			{
				goto exit;
			}
		} 
		while (UART_GetFlagStatus (UART2, UART_FLAG_TXFE)!= SET);
        /* Send Data */
        UART_SendData (UART2,DataByte);
		
		while (UART_GetFlagStatus (UART2, UART_FLAG_RXFF)!= SET);
        /* Recive data*/
		DataByte = UART_ReceiveData (UART2); 
		UART_SendData (UART2,DataByte);
		/* Check TXFE flag*/
		goto exit;

	}
exit: return 0;
}

void Can1Test (void)
{
uint32_t DataL,DataH;

DataL = 0x55555555;
DataH = 0x55555555;

	Can2PinCfg();
	Can2Setup(1);
   	 
	while(1)
	{
		/*do
		{
			if (PORT_ReadInputDataBit(PORTE,PORT_Pin_3) == 0)
			{
				goto exitC1;
			}
		} while ((CAN1->BUF_02_DLC & 0x00000008) != 0x00000008);*/
		DataL = CAN1->BUF_02_DATAL;
		DataH = CAN1->BUF_02_DATAH;
	    //clear RX buf
	  	CAN1->BUF_02_CON    = 0x00000000;
		CAN1->BUF_02_ID     = 0x00000000;
		CAN1->BUF_02_DLC    = 0x00000000;
		CAN1->BUF_02_DATAL  = 0x00000000;
		CAN1->BUF_02_DATAH  = 0x00000000;
		//send packet
		CAN1->BUF_01_ID     = 0x15AA0067;
		CAN1->BUF_01_DLC    = 0x8 | (1<<9) | (1<<11) | (1<<12);
		CAN1->BUF_01_DATAL  = DataL;
		CAN1->BUF_01_DATAH  = DataH;
		CAN1->BUF_01_CON    = 0x1 | (1<<5);
		while ((CAN1->BUF_01_CON & 0x00000020) != 0x00000000);
		CAN1->BUF_01_CON    = 0x00000000;//disable TX buffer
		CAN1->BUF_02_CON    = 0x00000003;//disable RX buffer
	}
exitC1: 	
	/* Prepare HSI clk */
	RST_CLK_HSIcmd(ENABLE);
	RST_CLK_HSIstatus();
	RST_CLK_CPUclkSelection(RST_CLK_CPUclkHSI);
	
	/* Reset CPU_CLOCK bits */
	RST_CLK->CPU_CLOCK   &= (uint32_t)0x00000000;
	
	/* Reset PLL_CONTROL bits */
	RST_CLK->PLL_CONTROL &= (uint32_t)0x00000000;
	
	/* Reset HSEON and HSEBYP bits */
	RST_CLK->HS_CONTROL  &= (uint32_t)0x00000000;

}

void Can2Test (void)
{
uint32_t DataL,DataH;

	Can2PinCfg();
	Can2Setup(2);

	while(1)
	{
		do
		{
			if (PORT_ReadInputDataBit(PORTE,PORT_Pin_3) == 0)
			{
				goto exitC2;
			}
		} while ((CAN1->BUF_02_DLC & 0x00000008) != 0x00000008);
		DataL = CAN1->BUF_02_DATAL;
		DataH = CAN1->BUF_02_DATAH;
	    //clear RX buf
	  	CAN1->BUF_02_CON    = 0x00000000;
		CAN1->BUF_02_ID     = 0x00000000;
		CAN1->BUF_02_DLC    = 0x00000000;
		CAN1->BUF_02_DATAL  = 0x00000000;
		CAN1->BUF_02_DATAH  = 0x00000000;
		//send packet
		CAN1->BUF_01_ID     = 0x15AA0067;
		CAN1->BUF_01_DLC    = 0x8 | (1<<9) | (1<<11) | (1<<12);
		CAN1->BUF_01_DATAL  = DataL;
		CAN1->BUF_01_DATAH  = DataH;
		CAN1->BUF_01_CON    = 0x1 | (1<<5);
		while ((CAN1->BUF_01_CON & 0x00000020) != 0x00000000);
		CAN1->BUF_01_CON    = 0x00000000;//disable TX buffer
		CAN1->BUF_02_CON    = 0x00000003;//disable RX buffer
	}
exitC2: 	
	/* Prepare HSI clk */
	RST_CLK_HSIcmd(ENABLE);
	RST_CLK_HSIstatus();
	RST_CLK_CPUclkSelection(RST_CLK_CPUclkHSI);
	
	/* Reset CPU_CLOCK bits */
	RST_CLK->CPU_CLOCK   &= (uint32_t)0x00000000;
	
	/* Reset PLL_CONTROL bits */
	RST_CLK->PLL_CONTROL &= (uint32_t)0x00000000;
	
	/* Reset HSEON and HSEBYP bits */
	RST_CLK->HS_CONTROL  &= (uint32_t)0x00000000;

}

int LedTest (void)
{
int i;

LedStart:
	TestLedDone = 0;
	
	for (i=0;i<1000000;i++){}
	NVIC->ICER[0] = 0x00004000;

	BlinkLED1(5,100000,LED1);
	BlinkLED1(5,100000,LED2);

    TestLedDone = 1;
	
	NVIC->ISER[0] = 0x00004000;
	for (;;)
	{
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_2) == 0) goto LedStart;
	    if (PORT_ReadInputDataBit(PORTE,PORT_Pin_3) == 0) goto LedExit;
	}
LedExit:
	TestLedDone = 0;
	return 0;
}

int main (void)
{
char s1;

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
    RST_CLK_PCLKcmd(RST_CLK_PCLK_EXT_BUS_CNTRL, ENABLE);

	MltPinCfg ();
	ButtonPinGfg ();

	LcdInit();
	
	do
	{
		s1 = ReadStatus(1);
	} while (s1 & 0x80 != 0x00);
	DispOn (1);

	do
	{
		s1 = ReadStatus(2);
	} while (s1 & 0x80 != 0x00);
	DispOn (2);

	LcdClearChip (1);
	LcdClearChip (2);

	SCB->AIRCR = 0x05FA0000 | 0x00000500;//configure
	SCB->VTOR = 0x08000000;				   //NVIC
	NVIC->ISER[0] = 0x00004000;			   
	
	Tim1Setup();

 	while (1)
	{
		//select
		if ( ! PORT_ReadInputDataBit(PORTC,PORT_Pin_2) )
		{
			while ( ! PORT_ReadInputDataBit(PORTC,PORT_Pin_2) ) {};
			if (MenuMainItem == 0)
			{
				LedFlag = 1;
				LedTest();
				LedFlag = 0;
			}
			if (MenuMainItem == 1)
			{
				UartFlag = 1;
				UartTest();
				UartFlag = 0;
			}
			if (MenuMainItem == 2)
			{
				Can1Flag = 1;
				Can1Test();
				Can1Flag = 0;
			}
			if (MenuMainItem == 3)
			{
				Can2Flag = 1;
				Can2Test();
				Can2Flag = 0;
			}
		}
		//up
		else if (!PORT_ReadInputDataBit(PORTB,PORT_Pin_5))
		{
			while ( ! PORT_ReadInputDataBit(PORTB,PORT_Pin_5) ) {};
			if (MenuMainItem != 0) MenuMainItem--;
			if (CursorPosItem != 0) CursorPosItem--;
			else 	BlinkLED1(5,100000,LED1);	
			
		}
		//down
		else if (!PORT_ReadInputDataBit(PORTE,PORT_Pin_1))
		{
			while ( ! PORT_ReadInputDataBit(PORTE,PORT_Pin_1) ) {};
			if (MenuMainItem != 3) MenuMainItem++;
			if (CursorPosItem != 2) CursorPosItem++;
			
		}
		//left
		else if (!PORT_ReadInputDataBit(PORTE,PORT_Pin_3))
		{
			while ( ! PORT_ReadInputDataBit(PORTE,PORT_Pin_3) ) {};
				BlinkLED1(5,100000,LED2);			
		}
		//right
		else if (!PORT_ReadInputDataBit(PORTB,PORT_Pin_6))
		{
			while ( ! PORT_ReadInputDataBit(PORTB,PORT_Pin_6) ) {};
				BlinkLED1(5,100000,LED1);						
		}

	}




}



/******************* (C) COPYRIGHT 2010 Phyton *******************
*
* END OF FILE main.c */

