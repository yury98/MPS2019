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

void PortsSSP2Init(void) {
  /* Configure SSP2 pins: FSS, CLK, RXD, TXD */

  /* Configure PORTD pins 2, 3, 5, 6 */
  PORT_InitStructure.PORT_Pin   = (PORT_Pin_2 | PORT_Pin_3 | PORT_Pin_5);
  PORT_InitStructure.PORT_OE    = PORT_OE_IN;
  PORT_InitStructure.PORT_FUNC  = PORT_FUNC_ALTER;
  PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PORT_InitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PORT_Init(MDR_PORTD, &PORT_InitStructure);
  PORT_InitStructure.PORT_OE    = PORT_OE_OUT;
  PORT_InitStructure.PORT_Pin   = (PORT_Pin_6);
  PORT_Init(MDR_PORTD, &PORT_InitStructure);
}
	
void SSP2StructInit(void) {
	  /* Reset all SSP settings */
  SSP_DeInit(MDR_SSP2);

  SSP_BRGInit(MDR_SSP2,SSP_HCLKdiv16);
	
  /* SSP2 SLAVE configuration ------------------------------------------------*/
  SSP_StructInit (&sSSP);

  sSSP.SSP_SCR  = 0x10;
  sSSP.SSP_CPSDVSR = 12;
  sSSP.SSP_Mode = SSP_ModeSlave;
  sSSP.SSP_WordLength = SSP_WordLength16b;
  sSSP.SSP_SPH = SSP_SPH_1Edge;
  sSSP.SSP_SPO = SSP_SPO_Low;
  sSSP.SSP_FRF = SSP_FRF_SPI_Motorola;
  sSSP.SSP_HardwareFlowControl = SSP_HardwareFlowControl_SSE;
  SSP_Init (MDR_SSP2,&sSSP);
	
	  /* Enable SSP2 */
  SSP_Cmd(MDR_SSP2, ENABLE);
}

void SSP2Test(void) {
		uint16_t Src2 = 0x0003;
		uint16_t Dst2 = 0x0000;
		PortsSSP2Init();
		SSP2StructInit();
	  while (SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_RNE) == RESET)
    {
    }
    /* Read SPI2 received data */
    Dst2 = SSP_ReceiveData(MDR_SSP2);
		PORT_SetBits(MDR_PORTC, PORT_Pin_0);
		DELAY(1000000);
		PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
		
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
			

	
	PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
    SetupPortsForDiods();
    
	while (1)
	{
    SSP2Test();
		DELAY(1000000);
	}
}
