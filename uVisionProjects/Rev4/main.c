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

#define LED1            PORT_Pin_10
#define LED2            PORT_Pin_11
#define LED3            PORT_Pin_12
#define LED4            PORT_Pin_13
#define LED5            PORT_Pin_14
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int count_title = 0;
int MenuMainItem = 0;
int CursorPosItem = 0;

uint32_t FlashResult=0;
char TestFlashDone = 0;
uint32_t RamResult=0;
char TestRamDone = 0;
uint32_t EthResult=0;
char TestEthDone = 0;
char TestLedDone = 0;
char UartFlag = 0;
char Can1Flag = 0;
char Can2Flag = 0;
char FlashFlag = 0;
char RamFlag = 0;
char EthFlag = 0;
char LedFlag = 0;
static PORT_InitTypeDef PortInit;
static UART_InitTypeDef UART_InitStructure;


typedef struct {
	__IO uint32_t MAC_CTRL;
	__IO uint32_t MinFrame;
	__IO uint32_t MaxFrame;
	__IO uint32_t CollConfig;
	__IO uint32_t IPGTx;
	__IO uint32_t MAC_ADDR_T;
	__IO uint32_t MAC_ADDR_M;
	__IO uint32_t MAC_ADDR_H;
	__IO uint32_t  HASH0;
	__IO uint32_t  HASH1;
	__IO uint32_t  HASH2;
	__IO uint32_t  HASH3;
	__IO uint32_t  INT_MSK;
	__IO uint32_t  INT_SRC;
	__IO uint32_t  PHY_CTRL;
	__IO uint32_t  PHY_STAT;
	__IO uint32_t  RXBF_HEAD;
	__IO uint32_t  RXBF_TAIL;
	__IO uint32_t  dammy0;
	__IO uint32_t  dammy1;
	__IO uint32_t  STAT_RX_ALL;
	__IO uint32_t  STAT_RX_OK;
	__IO uint32_t  STAT_RX_OVF;
	__IO uint32_t  STAT_RX_LOST;
	__IO uint32_t  STAT_TX_ALL;
	__IO uint32_t  STAT_TX_OK;
	__IO uint32_t  base_RxBF;
	__IO uint32_t  base_TxBF;
	__IO uint32_t  base_RxBD;
	__IO uint32_t  base_TxBD;
	__IO uint32_t  base_RG;
	__IO uint32_t  GCTRL;
} _ethernet;

// RX buffer base address
#define BASE_ETH_RXBF			   0x60000000 
// RX descriptor base address
#define BASE_ETH_RXDS			   0x60002000
// TX buffer base address
#define BASE_ETH_TXBF			   0x60004000 
// TX descriptor base address
#define BASE_ETH_TXDS			   0x60006000
// Control Reg Ethernet 
#define BASE_ETHERNET              0x60007F00

#define ETH1                  ((_ethernet *)BASE_ETHERNET)
#define ETH1RXBF              ((int *)BASE_ETH_RXBF) 
#define ETH1RXDC              ((int *)BASE_ETH_RXDS)
#define ETH1TXBF              ((int *)BASE_ETH_TXBF)
#define ETH1TXDC              ((int *)BASE_ETH_TXDS)
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Delay(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}

void LEDOn(uint32_t LED_Num)
{
  PORT_SetBits(PORTD, LED_Num);
}

void LEDOff(uint32_t LED_Num)
{
  PORT_ResetBits(PORTD, LED_Num);
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
void ExtBusCtrlPinCfg (void)
{

	/* Configure PORTC pins 1..6 for output ExtBus control  */
	PortInit.PORT_Pin   = (PORT_Pin_1 | PORT_Pin_3 | PORT_Pin_4 | PORT_Pin_5 | PORT_Pin_6);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;	
//	PortInit.PORT_SPEED = PORT_SPEED_SLOW;	PORT_SPEED_FAST
	PortInit.PORT_SPEED = PORT_SPEED_FAST;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
   PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTC, &PortInit);
	
}

void ExtBusPinCfg (void)
{

	/* Configure PORTE pins 0..3 for output ExtBus adress  */
	PortInit.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;	
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
//	PortInit.PORT_SPEED = PORT_SPEED_FAST;	
	PortInit.PORT_GFEN = PORT_GFEN_OFF;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTE, &PortInit);

	/* Configure PORTF pins 2..15 for output ExtBus adress  */
	//PortInit.PORT_Pin   = PORT_Pin_All;
	PortInit.PORT_Pin   = (PORT_Pin_2 | PORT_Pin_3 | PORT_Pin_4 | PORT_Pin_5 | PORT_Pin_6 | PORT_Pin_7 | PORT_Pin_8 | PORT_Pin_9 | PORT_Pin_10 | PORT_Pin_11 | PORT_Pin_12 | PORT_Pin_13 | PORT_Pin_14 | PORT_Pin_15);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
		PortInit.PORT_GFEN = PORT_GFEN_OFF;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTF, &PortInit);

    /* Configure PORTA pins 0..15 for inout ExtBus data  */
	PortInit.PORT_Pin   = PORT_Pin_All;
 	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTA, &PortInit);

    /* Configure PORTB pins 0..15 for inout ExtBus data  */
	PortInit.PORT_Pin   = PORT_Pin_All;
	//PortInit.PORT_Pin   = (PORT_Pin_8 | PORT_Pin_9 | PORT_Pin_10 | PORT_Pin_11 | PORT_Pin_12 | PORT_Pin_13 | PORT_Pin_14 | PORT_Pin_15);
	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTB, &PortInit);

	/* Configure PORTC pins 1..6 for output ExtBus control  */
	PortInit.PORT_Pin   = (PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3 | PORT_Pin_4 | PORT_Pin_5 | PORT_Pin_6);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_PD = PORT_PD_DRIVER;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTC, &PortInit);
	
}

void EthPinCfg (void)
{
	/* Configure PORTA pins 0..15 for inout EthBus data  */
	PortInit.PORT_Pin   = PORT_Pin_All;
 	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTA, &PortInit);

	/* Configure PORTE pins 12 for output EthBus CS  */
	PortInit.PORT_Pin   = PORT_Pin_12;
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_PD = PORT_PD_DRIVER;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
	PortInit.PORT_GFEN = PORT_GFEN_OFF;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTE, &PortInit);

	/* Configure PORTC pins 1,2 for output EthBus OE,WE  */
	PortInit.PORT_Pin   = (PORT_Pin_1 | PORT_Pin_2);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_PD = PORT_PD_DRIVER;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTC, &PortInit);
 
	/* Configure PORTF pins 2..15 for output EthBus adress  */
	PortInit.PORT_Pin   = (PORT_Pin_2 | PORT_Pin_3 | PORT_Pin_4 | PORT_Pin_5 | PORT_Pin_6 | PORT_Pin_7 | PORT_Pin_8 | PORT_Pin_9 | PORT_Pin_10 | PORT_Pin_11 | PORT_Pin_12 | PORT_Pin_13 | PORT_Pin_14 | PORT_Pin_15);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_MAIN;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTF, &PortInit);

	/* Configure PORTB pins 11 for output EthBus RESET  */
	PortInit.PORT_Pin   = (PORT_Pin_11);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_PD = PORT_PD_DRIVER;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;
	PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;

	PORT_Init(PORTB, &PortInit);

	PORT_SetBits(PORTB, PORT_Pin_11);
	PORT_ResetBits(PORTB, PORT_Pin_11);
    
	PortInit.PORT_OE    = PORT_OE_IN;
	PORT_Init(PORTB, &PortInit);

}

void MltPinCfg (void)
{
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
	/* Configure PORTA pins 0..7 for mlt inout data  */
	PortInit.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3 | PORT_Pin_4 | PORT_Pin_5 | PORT_Pin_6 | PORT_Pin_7);
	PortInit.PORT_OE    = PORT_OE_IN;
	PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;

	PORT_Init(PORTA, &PortInit);

	/* Configure PORTC pins 2,7 for mlt output  */
	#ifdef Rev1
	PortInit.PORT_Pin   = (PORT_Pin_2 | PORT_Pin_7 | PORT_Pin_9);
	#endif
	#ifdef Rev2
	PortInit.PORT_Pin   = (PORT_Pin_2 | PORT_Pin_7);
	#endif

	PortInit.PORT_OE    = PORT_OE_OUT;

	PORT_Init(PORTC, &PortInit);

	/* Configure PORTE pins 4,5,10,11 for mlt output */
	#ifdef Rev1
	PortInit.PORT_Pin   = (PORT_Pin_4 | PORT_Pin_5 | PORT_Pin_11);
	#endif
	#ifdef Rev2
	PortInit.PORT_Pin   = (PORT_Pin_4 | PORT_Pin_5 | PORT_Pin_10 | PORT_Pin_11);
	#endif

	PORT_Init(PORTE, &PortInit);
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

void Can1PinCfg(void)
{
	/* Fill PortInit structure*/
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_MAIN;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
    /* Configure PORTC pins 9 (CAN1_RX) as input */
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = PORT_Pin_9;
    PORT_Init(PORTC, &PortInit);
    /* Configure PORTC pins 8 (CAN1_TX) as output */
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_8;
    PORT_Init(PORTC, &PortInit);	
}

void LedPinGfg (void)
{
	/* Configure PORTD pins 10..14 for output to switch LEDs on/off */
	PortInit.PORT_Pin   = (PORT_Pin_10 | PORT_Pin_11 | PORT_Pin_12 | PORT_Pin_13 | PORT_Pin_14);
	PortInit.PORT_OE    = PORT_OE_OUT;
	PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;

	PORT_Init(PORTD, &PortInit);
}

void ButtonPinGfg (void)
{
	/* Configure PORTC pins 10..14 for input Buttons  */
	PortInit.PORT_Pin   = (PORT_Pin_10 | PORT_Pin_11 | PORT_Pin_12 | PORT_Pin_13 | PORT_Pin_14);
	PortInit.PORT_OE    = PORT_OE_IN;
	PortInit.PORT_FUNC  = PORT_FUNC_PORT;
	PortInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PortInit.PORT_SPEED = PORT_SPEED_SLOW;

	PORT_Init(PORTC, &PortInit);
}

void Uart2Setup(void)
{
	/* Select HSI/2 as CPU_CLK source*/
    RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);
    /* Enables the CPU_CLK clock on UART2 */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
    /* Set the HCLK division factor = 1 for UART2*/
	UART_BRGInit(UART2, UART_HCLKdiv1);

    UART_InitStructure.UART_BaudRate                = 12000;
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

void Can1Setup(int rate)
{
	RST_CLK_HSEconfig(RST_CLK_HSE_ON);
    while (RST_CLK_HSEstatus() != SUCCESS);
    
	/* Select HSI/2 as CPU_CLK source*/
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
	//(1 << 27) | (3 << 25) | ((5 - 1) << 22) | ((5 - 1) << 19) | ((2 - 1) << 16) | 4; //125
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

void UartTest (void)
{
uint8_t DataByte;

	Uart2PinCfg();
	Uart2Setup();
 	while(1)
	{
		/* Check RXFF flag*/
		do
		{
			if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0)
			{
				goto exitU;
			}
		} while (UART_GetFlagStatus (UART2, UART_FLAG_RXFF)!= SET);
        /* Recive data*/
		DataByte = UART_ReceiveData (UART2); 
		/* Check TXFE flag*/
		while (UART_GetFlagStatus (UART2, UART_FLAG_TXFE)!= SET);
        /* Send Data */
        UART_SendData (UART2,DataByte);
	}
exitU:
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

void Can1Test (void)
{
uint32_t DataL,DataH;

	Can1PinCfg();
	Can1Setup(1);

	while(1)
	{
		do
		{
			if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0)
			{
				goto exitC1;
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

	Can1PinCfg();
	Can1Setup(2);

	while(1)
	{
		do
		{
			if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0)
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

void RamTest (void)
{
int i;
uint32_t mem[18];

/*	for (;;)
	{
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_10) == 0) goto RamStart;
	    if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0) goto RamExit;
	}
*/	
RamStart:
	TestRamDone = 0;
	RamResult = 0;

	for (i=0;i<1000000;i++){}
	NVIC->ICER[0] = 0x00004000;
	
  	ExtBusPinCfg ();
	EXT_BUS_CNTRL->EXT_BUS_CONTROL = 0x00008002;

	for(i=0;i<1000000;i++){}

//****************************read-write
	HWREG(0x60000004) = 0x11111111;
	mem[0]  = HWREG(0x60000004);
	HWREG(0x60000008) = 0x22222222;
	mem[1]  = HWREG(0x60000008);

	HWREG(0x60000010) = 0x33333333;
	mem[2]  = HWREG(0x60000010);
	HWREG(0x60000020) = 0x44444444;
	mem[3]  = HWREG(0x60000020);
	HWREG(0x60000040) = 0x55555555;
	mem[4]  = HWREG(0x60000040);
	HWREG(0x60000080) = 0x66666666;
	mem[5]  = HWREG(0x60000080);

	HWREG(0x60000100) = 0x77777777;
	mem[6]  = HWREG(0x60000100);
	HWREG(0x60000200) = 0x88888888;
	mem[7]  = HWREG(0x60000200);
	HWREG(0x60000400) = 0x99999999;
	mem[8]  = HWREG(0x60000400);
	HWREG(0x60000800) = 0xAAAAAAAA;
	mem[9]  = HWREG(0x60000800);
	
	HWREG(0x60001000) = 0xBBBBBBBB;
	mem[10] = HWREG(0x60001000);
	HWREG(0x60002000) = 0xCCCCCCCC;
	mem[11] = HWREG(0x60002000);
	HWREG(0x60004000) = 0xDDDDDDDD;
	mem[12] = HWREG(0x60004000);
	HWREG(0x60008000) = 0xEEEEEEEE;
	mem[13] = HWREG(0x60008000);
	
	HWREG(0x60010000) = 0xFFFFFFFF;
	mem[14] = HWREG(0x60010000);
	HWREG(0x60020000) = 0x00000000;
	mem[15] = HWREG(0x60020000);
	HWREG(0x60040000) = 0x11111111;
	mem[16] = HWREG(0x60040000);
	HWREG(0x60080000) = 0x22222222;	
	mem[17] = HWREG(0x60080000); 
//************************compare
	if ((mem[0]  & 0xFFE0FFFF) != (0x11111111 & 0xFFE0FFFF)) RamResult |= 1 << 0;
	if ((mem[1]  & 0xFFE0FFFF) != (0x22222222 & 0xFFE0FFFF)) RamResult |= 1 << 1;
	if ((mem[2]  & 0xFFE0FFFF) != (0x33333333 & 0xFFE0FFFF)) RamResult |= 1 << 2;
	if ((mem[3]  & 0xFFE0FFFF) != (0x44444444 & 0xFFE0FFFF)) RamResult |= 1 << 3;

	if ((mem[4]  & 0xFFE0FFFF) != (0x55555555 & 0xFFE0FFFF)) RamResult |= 1 << 4;
	if ((mem[5]  & 0xFFE0FFFF) != (0x66666666 & 0xFFE0FFFF)) RamResult |= 1 << 5;
	if ((mem[6]  & 0xFFE0FFFF) != (0x77777777 & 0xFFE0FFFF)) RamResult |= 1 << 6;
	if ((mem[7]  & 0xFFE0FFFF) != (0x88888888 & 0xFFE0FFFF)) RamResult |= 1 << 7;

	if ((mem[8]  & 0xFFE0FFFF) != (0x99999999 & 0xFFE0FFFF)) RamResult |= 1 << 8;
	if ((mem[9]  & 0xFFE0FFFF) != (0xAAAAAAAA & 0xFFE0FFFF)) RamResult |= 1 << 9;
	if ((mem[10] & 0xFFE0FFFF) != (0xBBBBBBBB & 0xFFE0FFFF)) RamResult |= 1 << 10;
	if ((mem[11] & 0xFFE0FFFF) != (0xCCCCCCCC & 0xFFE0FFFF)) RamResult |= 1 << 11;

	if ((mem[12] & 0xFFE0FFFF) != (0xDDDDDDDD & 0xFFE0FFFF)) RamResult |= 1 << 12;
	if ((mem[13] & 0xFFE0FFFF) != (0xEEEEEEEE & 0xFFE0FFFF)) RamResult |= 1 << 13;
	if ((mem[14] & 0xFFE0FFFF) != (0xFFFFFFFF & 0xFFE0FFFF)) RamResult |= 1 << 14;
	if ((mem[15] & 0xFFE0FFFF) != (0x00000000 & 0xFFE0FFFF)) RamResult |= 1 << 15;

	if ((mem[16] & 0xFFE0FFFF) != (0x11111111 & 0xFFE0FFFF)) RamResult |= 1 << 16;
	if ((mem[17] & 0xFFE0FFFF) != (0x22222222 & 0xFFE0FFFF)) RamResult |= 1 << 17;

	TestRamDone = 1;
	PORT_DeInit(PORTA);
	PORT_DeInit(PORTB);
	PORT_DeInit(PORTF);
	MltPinCfg ();
	NVIC->ISER[0] = 0x00004000;

	for (;;)
	{
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_10) == 0) goto RamStart;
	    if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0) goto RamExit;
	}
RamExit:
	TestRamDone = 0;
}

void FlashTest (void)
{
int i;
uint32_t mem[18];

/*	for (;;)
	{
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_10) == 0) goto FlashStart;
	    if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0) goto FlashExit;
	}
*/	
FlashStart:

	TestFlashDone = 0;
	FlashResult = 0;

	for (i=0;i<10000;i++){}
	NVIC->ICER[0] = 0x00004000;
	
  	ExtBusPinCfg ();

	EXT_BUS_CNTRL->EXT_BUS_CONTROL = 0x00000001;

	for(i=0;i<1000000;i++){}
	HWREG(0x10000000) = 0xF0F0F0F0;
	
	EraseFullFLASH();

//****************************read-write
	WriteFLASH(0x10000004, 0x11111111);
	mem[0] = ReadFLASH(0x10000004);
	WriteFLASH(0x10000008, 0x22222222);
	mem[1] = ReadFLASH(0x10000008);

	WriteFLASH(0x10000010, 0x33333333);
	mem[2] = ReadFLASH(0x10000010);
	WriteFLASH(0x10000020, 0x44444444);
	mem[3] = ReadFLASH(0x10000020);
	WriteFLASH(0x10000040, 0x55555555);
	mem[4] = ReadFLASH(0x10000040);
	WriteFLASH(0x10000080, 0x66666666);
	mem[5] = ReadFLASH(0x10000080);

	WriteFLASH(0x10000100, 0x77777777);
	mem[6] = ReadFLASH(0x10000100);
	WriteFLASH(0x10000200, 0x88888888);
	mem[7] = ReadFLASH(0x10000200);
	WriteFLASH(0x10000400, 0x99999999);
	mem[8] = ReadFLASH(0x10000400);
	WriteFLASH(0x10000800, 0xAAAAAAAA);
	mem[9] = ReadFLASH(0x10000800);

	WriteFLASH(0x10001000, 0xBBBBBBBB);
	mem[10] = ReadFLASH(0x10001000);
	WriteFLASH(0x10002000, 0xCCCCCCCC);
	mem[11] = ReadFLASH(0x10002000);
	WriteFLASH(0x10004000, 0xDDDDDDDD);
	mem[12] = ReadFLASH(0x10004000);
	WriteFLASH(0x10008000, 0xEEEEEEEE);
	mem[13] = ReadFLASH(0x10008000);

	WriteFLASH(0x10010000, 0xFFFFFFFF);
	mem[14] = ReadFLASH(0x10010000);
	WriteFLASH(0x10020000, 0x00000000);
	mem[15] = ReadFLASH(0x10020000);
	WriteFLASH(0x10040000, 0x11111111);
	mem[16] = ReadFLASH(0x10040000);
	WriteFLASH(0x10080000, 0x22222222);
	mem[17] = ReadFLASH(0x10080000); 
//************************compare	
	if (mem[0]  != 0x11111111) FlashResult |= 1 << 0;
	if (mem[1]  != 0x22222222) FlashResult |= 1 << 1;
	if (mem[2]  != 0x33333333) FlashResult |= 1 << 2;
	if (mem[3]  != 0x44444444) FlashResult |= 1 << 3;
	
	if (mem[4]  != 0x55555555) FlashResult |= 1 << 4;
	if (mem[5]  != 0x66666666) FlashResult |= 1 << 5;
	if (mem[6]  != 0x77777777) FlashResult |= 1 << 6;
	if (mem[7]  != 0x88888888) FlashResult |= 1 << 7;

	if (mem[8]  != 0x99999999) FlashResult |= 1 << 8;
	if (mem[9]  != 0xAAAAAAAA) FlashResult |= 1 << 9;
	if (mem[10] != 0xBBBBBBBB) FlashResult |= 1 << 10;
	if (mem[11] != 0xCCCCCCCC) FlashResult |= 1 << 11;
	
	if (mem[12] != 0xDDDDDDDD) FlashResult |= 1 << 12;
	if (mem[13] != 0xEEEEEEEE) FlashResult |= 1 << 13;
	if (mem[14] != 0xFFFFFFFF) FlashResult |= 1 << 14;
	if (mem[15] != 0x00000000) FlashResult |= 1 << 15;

	if (mem[16] != 0x11111111) FlashResult |= 1 << 16;
	if (mem[17] != 0x22222222) FlashResult |= 1 << 17;
	
	if (FlashResult == 0) PORT_SetBits(PORTD, LED1);
	else PORT_SetBits(PORTD, LED5);	
	EraseFullFLASH();
    PORT_SetBits(PORTD, LED2);

	TestFlashDone = 1;
	PORT_DeInit(PORTA);
	PORT_DeInit(PORTB);
	PORT_DeInit(PORTF);
	MltPinCfg ();
	NVIC->ISER[0] = 0x00004000;

	for (;;)
	{
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_10) == 0) goto FlashStart;
	    if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0) goto FlashExit;
	}
FlashExit:
	TestFlashDone = 0;
}

void EthTest (void)
{
int i,temp;

EthStart:	
	TestEthDone = 0;
	EthResult = 0;

	for (i=0;i<1000000;i++){}
	NVIC->ICER[0] = 0x00004000;
	
  	EthPinCfg ();

	EXT_BUS_CNTRL->EXT_BUS_CONTROL = 0x2002; // EXT BUS ON
	
	ETH1->GCTRL = 0x5382;
	
	// test memory RXBUFFER
	for (i=0;i<0x800;i++) ETH1RXBF[i] = i;
	for (i=0;i<0x800;i++)
	{	
		temp=ETH1RXBF[i];		
		if (ETH1RXBF[i] != i)
			
		{
			EthResult = 1;
//			goto TestEnd;
		};
	 }
	 
	// test memory TXBUFFER
	for (i=0;i<0x800;i++) ETH1TXBF[i] = i;
	for (i=0;i<0x800;i++)
	{ 
		if (ETH1TXBF[i] != i)
		{
			EthResult = 1;
			goto TestEnd;
		};
	}
	// test memory RXBUFFER
	for (i=0;i<0x800;i++)
		ETH1RXBF[i] = 0x00005555;
	for (i=0;i<0x800;i++)
		if (ETH1RXBF[i] != 0x00005555)
		{
			EthResult = 1;
			goto TestEnd;
		};

	// test memory TXBUFFER
	for (i=0;i<0x800;i++)
		ETH1TXBF[i] = 0x0000AAAA;
	for (i=0;i<0x800;i++)
		if (ETH1TXBF[i] != 0x0000AAAA)
		{
			EthResult = 1;
			goto TestEnd;
		};
	
	// test memory RX DESCR
	for (i=0;i<0x80;i++)
		ETH1RXDC[i] = i;
	for (i=0;i<0x80;i++)
		if (ETH1RXDC[i] != i)
		{
			EthResult = 1;
			goto TestEnd;
		};
	
	// test memory RX DESCR
	for (i=0;i<0x80;i++)
		ETH1TXDC[i] = i;
	for (i=0;i<0x80;i++)
		if (ETH1TXDC[i] != i)
		{
			EthResult = 1;
			goto TestEnd;
		};
TestEnd:		
	TestEthDone = 1;
	PORT_DeInit(PORTA);
	PORT_DeInit(PORTB);
	PORT_DeInit(PORTF);
	MltPinCfg ();
	NVIC->ISER[0] = 0x00004000;

	for (;;)
	{
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_10) == 0) goto EthStart;
	    if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0) goto EthExit;
	}
EthExit:
	TestEthDone = 0;
}
void LedTest (void)
{
/*	for (;;)
	{
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_10) == 0) goto LedStart;
	    if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0) goto LedExit;
	}
*/	
LedStart:
	TestLedDone = 0;

	BlinkLED1(5,100000,LED1);
	BlinkLED1(5,100000,LED2);
	BlinkLED1(5,100000,LED3);
	BlinkLED1(5,100000,LED4);
	BlinkLED1(5,100000,LED5);

    TestLedDone = 1;

	for (;;)
	{
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_10) == 0) goto LedStart;
	    if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0) goto LedExit;
	}
LedExit:
	TestLedDone = 0;
}

int main (void)
{
//uint8_t DataByte=0x55;
int i;
char s1;
//uint32_t mem[18];
//uint8_t result=0;

	RST_CLK_LSEconfig(RST_CLK_LSE_ON);
//    while (RST_CLK_LSEstatus() != SUCCESS);
    
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

	ExtBusCtrlPinCfg ();
    MltPinCfg ();
	LedPinGfg ();
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
		if ( PORT_ReadInputDataBit(PORTC,PORT_Pin_10) == 0)
		{
//			for (i=0;i<200000;i++){};
			while ( ! PORT_ReadInputDataBit(PORTC,PORT_Pin_10) ) {};
//				BlinkLED1(5,100000,LED3);						
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
			if (MenuMainItem == 4)
			{
				FlashFlag = 1;
				FlashTest();
				FlashFlag = 0;
			}
			if (MenuMainItem == 5)
			{
				RamFlag = 1;
				RamTest();
				RamFlag = 0;
			}
			if (MenuMainItem == 6)
			{
				EthFlag = 1;
				EthTest();
				EthFlag = 0;
			}
		}
		else
		{
		
		}
		//up
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_11) == 0)
		{
			while ( ! PORT_ReadInputDataBit(PORTC,PORT_Pin_11) ) {};			
			if (MenuMainItem != 0) MenuMainItem--;
//			else 	MenuMainItem=6;
				
			if (CursorPosItem != 0) CursorPosItem--;
//			else CursorPosItem = 6;	
				BlinkLED1(5,100000,LED4);						
		}
		else
		{
		
		}
		//down
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_12) == 0)
		{
			while ( ! PORT_ReadInputDataBit(PORTC,PORT_Pin_12) ) {};
			if (MenuMainItem != 6) MenuMainItem++;
//				else 	MenuMainItem=0;
			if (CursorPosItem != 2) CursorPosItem++;
//				else CursorPosItem = 0;	
//				BlinkLED1(5,100000,LED2);			
			
		}
		else
		{
		
		}
		//left
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_13) == 0)
		{
			while ( ! PORT_ReadInputDataBit(PORTC,PORT_Pin_13) ) {};
				BlinkLED1(5,100000,LED5);
			}
		else
		{
		
		}
		//right
		if (PORT_ReadInputDataBit(PORTC,PORT_Pin_14) == 0)
		{
			while ( ! PORT_ReadInputDataBit(PORTC,PORT_Pin_14) ) {};
				BlinkLED1(5,100000,LED1);			
		}
		else
		{
		
		}
	}




}



/******************* (C) COPYRIGHT 2010 Phyton *******************
*
* END OF FILE main.c */

