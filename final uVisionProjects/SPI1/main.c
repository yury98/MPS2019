#include "MDR32F9Qx_board.h"
#include "MDR32F9Qx_config.h"
#include "MDR32Fx.h"
#include "MDR32F9Qx_ssp.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_port.h"
#define DELAY(T) for (int i = T; i > 0; i--)

SSP_InitTypeDef sSSP;
PORT_InitTypeDef PORT_InitStructure;

uint16_t SrcBuf1;
uint16_t SrcBuf2;

uint8_t TxIdx = 0, RxIdx = 0;

// ????????? ???????????????? PORTC ??? ????????????? ?????
void SetupPortsForDiods(void)
{
	PORT_InitTypeDef Nastroyka;
	Nastroyka.PORT_Pin = PORT_Pin_0;
	Nastroyka.PORT_OE = PORT_OE_OUT;
	Nastroyka.PORT_FUNC = PORT_FUNC_PORT;
	Nastroyka.PORT_MODE = PORT_MODE_DIGITAL;
	Nastroyka.PORT_SPEED = PORT_SPEED_SLOW;
	PORT_Init (MDR_PORTC,&Nastroyka);
}

void PortsSSP1Init(void) {
	  /* Configure SSP1 pins: FSS, CLK, RXD, TXD */

  /* Configure PORTF pins 0, 1, 2, 3 */
	PORT_InitStructure.PORT_FUNC  = PORT_FUNC_ALTER;
  PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PORT_InitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PORT_InitStructure.PORT_Pin   = (PORT_Pin_3);
  PORT_InitStructure.PORT_OE    = PORT_OE_IN;
  PORT_Init(MDR_PORTF, &PORT_InitStructure);
  PORT_InitStructure.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2);
  PORT_InitStructure.PORT_OE    = PORT_OE_OUT;
  PORT_Init(MDR_PORTF, &PORT_InitStructure);
}
	
void SSP1StructInit(void) {
	  /* Reset all SSP settings */
  SSP_DeInit(MDR_SSP1);

  SSP_BRGInit(MDR_SSP1,SSP_HCLKdiv16);
	
	 /* SSP1 MASTER configuration ------------------------------------------------*/
  SSP_StructInit (&sSSP);

  sSSP.SSP_SCR  = 0x10;
  sSSP.SSP_CPSDVSR = 2;
  sSSP.SSP_Mode = SSP_ModeMaster;
  sSSP.SSP_WordLength = SSP_WordLength16b;
  sSSP.SSP_SPH = SSP_SPH_1Edge;
  sSSP.SSP_SPO = SSP_SPO_Low;
  sSSP.SSP_FRF = SSP_FRF_SPI_Motorola;
  sSSP.SSP_HardwareFlowControl = SSP_HardwareFlowControl_SSE;
  SSP_Init (MDR_SSP1,&sSSP);
	
	  /* Enable SSP1 */
  SSP_Cmd(MDR_SSP1, ENABLE);
}

void SSP1Test(void) {
		uint16_t Src1 = 0x0002;
		uint16_t Dst1 = 0x0000;
		PortsSSP1Init();
		SSP1StructInit();
	  while (SSP_GetFlagStatus(MDR_SSP1, SSP_FLAG_TFE) == RESET)
    {
    }
    /* Send SPI1 data */
    SSP_SendData(MDR_SSP1, Src1);
}


int main (void)
{
RST_CLK_LSEconfig(RST_CLK_LSE_ON);
    // ????????? ????????????
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
		
		  /* Enable peripheral clocks --------------------------------------------------*/
		RST_CLK_PCLKcmd((RST_CLK_PCLK_RST_CLK | RST_CLK_PCLK_SSP1 | RST_CLK_PCLK_SSP2),ENABLE);
			
    /* Init NVIC */
  SCB->AIRCR = 0x05FA0000 | ((uint32_t)0x500);
  SCB->VTOR = 0x08000000;
  /* Disable all interrupt */
  NVIC->ICPR[0] = 0xFFFFFFFF;
  NVIC->ICER[0] = 0xFFFFFFFF;

  /* Disable all DMA request */
  MDR_DMA->CHNL_REQ_MASK_CLR = 0xFFFFFFFF;
  MDR_DMA->CHNL_USEBURST_CLR = 0xFFFFFFFF;

  /* Reset PORTD settings */
  PORT_DeInit(MDR_PORTD);
  /* Reset PORTF settings */
  PORT_DeInit(MDR_PORTF);
	
	
	PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
    SetupPortsForDiods();
    
	while (1)
	{
    SSP1Test();
		DELAY(1000000);
	}
}
