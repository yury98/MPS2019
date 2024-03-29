/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_config.h"           // Keil::Device:Startup
#include "MDR32Fx.h"
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_rst_clk.h"
#include "1986BE9x_it.h"
#include "MDR32F9Qx_ssp.h"              // Keil::Drivers:SSP
#include "MDR32F9Qx_can.h"              // Keil::Drivers:CAN
#include "mlt_lcd.h"
//#include "font.h"
//#include "MilFlash.h"

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
#define DELAY(T) for (int i = T; i > 0; i--)

#define LED1            PORT_Pin_0
#define LED2            PORT_Pin_1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int count_title = 0;
int MenuMainItem = 0;
int CursorPosItem = 0;

char UartFlag = 0;
char SSPFlag = 0;
char TestSSPDone = 0;
char TestCanDone = 0;
char TestUartDone = 0;
char USBFlag = 0;
char CanFlag = 0;

char FullTestFlag = 0;
char FullTestUSBGetFlag = 0;
char FullTestCANFlag = 0;
char FullTestUARTFlag = 0;
char FullTestSSPFlag = 0;
char FullTestUSBSendFlag = 0;
char FullTestUSBInfoFlag = 0;
char FullTestUSBSendInfoFlag = 0;
char FullTestUARTGetFlag = 0;

uint8_t USBInfo;
uint8_t USBFinalInfo;
__IO uint32_t tx_buf = 1;
static PORT_InitTypeDef PortInit;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Delay(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
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
 	PORT_Init(MDR_PORTA, &PortInit);
	/* Configure PORTF pins 2,3 for mlt inout data  */
	PortInit.PORT_Pin   = (PORT_Pin_2 | PORT_Pin_3);
	PORT_Init(MDR_PORTF, &PortInit);

	/* Configure PORTB pins 7..10 for mlt output  */
	PortInit.PORT_Pin   = (PORT_Pin_7 | PORT_Pin_8 | PORT_Pin_9 | PORT_Pin_10);
	PortInit.PORT_OE    = PORT_OE_OUT;

	PORT_Init(MDR_PORTB, &PortInit);

	/* Configure PORTC pins 0,1 for mlt output */
	PortInit.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1);

	PORT_Init(MDR_PORTC, &PortInit);
}

void LedPinGfg (void)
{
	/* Configure PORTC pins 0,1 for output to switch LEDs on/off */
	PortInit.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;

	PORT_Init(MDR_PORTC, &PortInit);
}

void ButtonPinGfg (void)
{
	/* Configure PORTB pins 5,6 for input Buttons  */
	PortInit.PORT_Pin   = (PORT_Pin_5 | PORT_Pin_6);
	PortInit.PORT_OE    = PORT_OE_IN;
	PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;

	PORT_Init(MDR_PORTB, &PortInit);
	/* Configure PORTC pins 2 for input Buttons  */
    PortInit.PORT_Pin   = (PORT_Pin_2);

	PORT_Init(MDR_PORTC, &PortInit);
	/* Configure PORTE pins 1,3 for input Buttons  */
    PortInit.PORT_Pin   = (PORT_Pin_1 | PORT_Pin_3);

	PORT_Init(MDR_PORTE, &PortInit);
}

void Tim1Setup(void)
{
//	vu16 TIMx_ARR, TIMx_CNT, TIMx_PSG, TIMx_CNTRL; 
//	vu16 value;
//	vu16 TIMx_CCRy[4];

	/* Enables the clock on  TIMER1*/
	RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER1, ENABLE);
	MDR_RST_CLK->TIM_CLOCK = 0x07000000;

	MDR_TIMER1->CNTRL = 0x00000000;

	MDR_TIMER1->CNT = 0x00000000;	//start count value
	MDR_TIMER1->PSG = 0x0000003F;	//div main clk
	MDR_TIMER1->ARR = 0x0000AFFF;	//end count value

	MDR_TIMER1->CH1_CNTRL = 0x00000000;
	MDR_TIMER1->CH2_CNTRL = 0x00000000;
	MDR_TIMER1->CH3_CNTRL = 0x00000000;
	MDR_TIMER1->CH4_CNTRL = 0x00000000;

	MDR_TIMER1->CH1_CNTRL1 = 0x00000000;
	MDR_TIMER1->CH2_CNTRL1 = 0x00000000;
	MDR_TIMER1->CH3_CNTRL1 = 0x00000000;
	MDR_TIMER1->CH4_CNTRL1 = 0x00000000;

	MDR_TIMER1->STATUS = 0x00000000;
	MDR_TIMER1->IE  = 0x00000002;
	MDR_TIMER1->CNTRL  |= 0x00000001;
}

int CanTest (void)
{
	CAN_ports_ini();
	CAN_Setup();
	CAN_TxMsgTypeDef TxMsg;
	DELAY(1000000);
	/* transmit */
  TxMsg.IDE     = CAN_ID_EXT;
  TxMsg.DLC     = 0x04;
  TxMsg.PRIOR_0 = DISABLE;
  TxMsg.ID      = 0x15555555;
  TxMsg.Data[1] = 0;
  TxMsg.Data[0] = 0x12345678;

  CAN_Transmit(MDR_CAN1, tx_buf, &TxMsg);
	TestCanDone = 1;	
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit;
			}
		} while (1);
	exit: return 0;
}

// ????????? ???????????? UART
int UartTest (void)
{
uint8_t DataByte = 'm';

	Uart2PinCfg();
	Uart2Setup();
	DELAY(1000000);
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit;
			}
		} while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET);
    
		// ???????? ??????
    UART_SendData (MDR_UART2,DataByte);
		
		TestUartDone = 1;
		
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit;
			}
		} while (1);
	exit: return 0;
}

void SSPTest(void) {
	uint16_t Src1 = 0x0002;
	uint16_t Dst1 = 0x0000;
	PortsSSP2Init();
	SSP2StructInit();
	DELAY(1000000);
	 while (SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_TFE) == RESET)
   {
   }

   SSP_SendData(MDR_SSP2, Src1);
	TestSSPDone = 1;
	do
	{
		if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
		{
			goto exit;
		}
	} while (1);
	exit: return 0;
}

void USBTest (void)
{
	USB_main();
	while(1)
	{
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit;
			}
		} while (1);
			
		
	}
exit: 
	USB_off();
	NVIC_DisableIRQ(USB_IRQn);
	return 0;
}

void FullTest (void)
{
	FullTestUSBGetFlag = 1;
	USB_main();
	while(1)
	{
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit;
			}
		} while (FullTestUSBInfoFlag != 1);
		USB_off();	
		NVIC_DisableIRQ(USB_IRQn);
		DELAY(1000000);
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit0;
			}
		} while ( PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_2));
		FullTestUSBGetFlag = 0;
		FullTestUSBInfoFlag = 0;
		USBInfo = USBInfo + 1;
		FullTestSSPFlag = 1;
		PortsSSP2Init();
		SSP2StructInit();
		while (SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_TFE) == RESET)
		{
		}

		SSP_SendData(MDR_SSP2, USBInfo);
		DELAY(1000000);
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit0;
			}
		} while ( PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_2));
		
		FullTestSSPFlag = 0;
		
		FullTestCANFlag = 0;
		FullTestUARTFlag = 1;
		FullTestUARTGetFlag = 0;
		Uart2PinCfg();
		Uart2Setup();
    // ???????? ????? RXFF
    do
    {
        if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
        {
            goto exit0;
        }
    } while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFF)!= SET);
    // ????????? ??????
    USBInfo = UART_ReceiveData (MDR_UART2);
		FullTestUARTGetFlag = 1;
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit0;
			}
		} while ( PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_2));
		FullTestUARTFlag = 0;
		FullTestUARTGetFlag = 0;
		FullTestUSBSendInfoFlag = 1;
		FullTestUSBInfoFlag = 0;
		USB_main();
		USBFinalInfo = USBInfo + 1;
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit;
			}
		} while (FullTestUSBInfoFlag != 1);
		USB_off();	
		NVIC_DisableIRQ(USB_IRQn);
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit0;
			}
		} while ( PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_2));
		FullTestUSBSendInfoFlag = 0;
		FullTestUSBInfoFlag = 0;
		CAN_ports_ini();
		CAN_Setup();
		DELAY(1000000);
		USBInfo = USBFinalInfo + 1;
		CAN_TxMsgTypeDef TxMsg;
		DELAY(1000000);
		FullTestCANFlag = 1;
		TxMsg.IDE     = CAN_ID_EXT;
		TxMsg.DLC     = 0x04;
		TxMsg.PRIOR_0 = DISABLE;
		TxMsg.ID      = 0x15555555;
		TxMsg.Data[1] = 0;
		TxMsg.Data[0] = USBInfo;
		CAN_Transmit(MDR_CAN1, tx_buf, &TxMsg);
		do
		{
			if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
			{
				goto exit0;
			}
		} while (1);
	}
exit:
	USB_off();	
	NVIC_DisableIRQ(USB_IRQn);
	return 0;
exit0:
	return 0;
}

int main (void)
{
char s1;
	
	RST_CLK_LSEconfig(RST_CLK_LSE_ON);
    while (RST_CLK_LSEstatus() != SUCCESS);
    
	RST_CLK_HSEconfig(RST_CLK_HSE_ON);
  while (RST_CLK_HSEstatus() != SUCCESS);
	RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSIdiv2, RST_CLK_CPU_PLLmul1);
  RST_CLK_CPU_PLLcmd(ENABLE);
  if (RST_CLK_CPU_PLLstatus() != SUCCESS)
  {
    /* Trap */
    while (1)
    {
    }
  }

  /* CPU_C3_SEL = CPU_C2_SEL */
  RST_CLK_CPUclkPrescaler(RST_CLK_CPUclkDIV1);
  /* CPU_C2_SEL = PLL */
  RST_CLK_CPU_PLLuse(ENABLE);
  /* HCLK_SEL = CPU_C3_SEL */
  RST_CLK_CPUclkSelection(RST_CLK_CPUclkCPU_C3);
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
		if ( ! PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_2) )
		{
			while ( ! PORT_ReadInputDataBit(MDR_PORTC,PORT_Pin_2) ) {};
			if (MenuMainItem == 0)
			{
				CanFlag = 1;
				CanTest();
				CanFlag = 0;
				TestCanDone = 0;
			}
			if (MenuMainItem == 1)
			{
				SSPFlag = 1;
				SSPTest();
				SSPFlag = 0;
				TestSSPDone = 0;
			}
			if (MenuMainItem == 2)
			{
				USBFlag = 1;
				USBTest();
				USBFlag = 0;
			}
			if (MenuMainItem == 3)
			{
				UartFlag = 1;
				UartTest();
				UartFlag = 0;
				TestUartDone = 0;
			}
			if (MenuMainItem == 4)
			{
				FullTestFlag = 1;
				FullTest();
				FullTestFlag = 0;
				FullTestCANFlag = 0;
				FullTestUSBInfoFlag = 0;
			}
		}
		//up
		else if (!PORT_ReadInputDataBit(MDR_PORTB,PORT_Pin_5))
		{
			while ( ! PORT_ReadInputDataBit(MDR_PORTB,PORT_Pin_5) ) {};
			if (MenuMainItem != 0) MenuMainItem--;
			if (CursorPosItem != 0) CursorPosItem--;
			
			
		}
		//down
		else if (!PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_1))
		{
			while ( ! PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_1) ) {};
			if (MenuMainItem != 4) MenuMainItem++;
			if (CursorPosItem != 2) CursorPosItem++;
			
		}
	}
}

