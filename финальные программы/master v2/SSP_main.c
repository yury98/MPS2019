//#include "MDR32F9Qx_board.h"
#include "MDR32F9Qx_config.h"
#include "MDR32Fx.h"
#include "MDR32F9Qx_ssp.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_port.h"
#define DELAY(T) for (int i = T; i > 0; i--)

SSP_InitTypeDef sSSP;
PORT_InitTypeDef PORT_InitStructur;

uint16_t SrcBuf1;
uint16_t SrcBuf2;

uint8_t TxIdx = 0, RxIdx = 0;

void PortsSSP2Init(void) {
	  /* Configure SSP2 pins: FSS, CLK, RXD, TXD */

  /* Configure PORTF pins 0, 1, 2, 3 
	PORT_InitStructur.PORT_FUNC  = PORT_FUNC_ALTER;
  PORT_InitStructur.PORT_MODE  = PORT_MODE_DIGITAL;
  PORT_InitStructur.PORT_SPEED = PORT_SPEED_FAST;
  PORT_InitStructur.PORT_Pin   = (PORT_Pin_3);
  PORT_InitStructur.PORT_OE    = PORT_OE_IN;
  PORT_Init(MDR_PORTF, &PORT_InitStructur);
  PORT_InitStructur.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2);
  PORT_InitStructur.PORT_OE    = PORT_OE_OUT;
  PORT_Init(MDR_PORTF, &PORT_InitStructur);*/
	  /* Configure PORTD pins 2, 3, 5, 6 */
  PORT_InitStructur.PORT_Pin   = (PORT_Pin_2);
  PORT_InitStructur.PORT_OE    = PORT_OE_IN;
  PORT_InitStructur.PORT_FUNC  = PORT_FUNC_ALTER;
  PORT_InitStructur.PORT_MODE  = PORT_MODE_DIGITAL;
  PORT_InitStructur.PORT_SPEED = PORT_SPEED_FAST;
  PORT_Init(MDR_PORTD, &PORT_InitStructur);
  PORT_InitStructur.PORT_OE    = PORT_OE_OUT;
  PORT_InitStructur.PORT_Pin   = (PORT_Pin_6 | PORT_Pin_3 | PORT_Pin_5);
  PORT_Init(MDR_PORTD, &PORT_InitStructur);
}
	
void SSP2StructInit(void) {
	  /* Reset all SSP settings */
  SSP_DeInit(MDR_SSP2);
			  /* Enable peripheral clocks --------------------------------------------------*/
		RST_CLK_PCLKcmd((RST_CLK_PCLK_RST_CLK | RST_CLK_PCLK_SSP1 | RST_CLK_PCLK_SSP2),ENABLE);

  SSP_BRGInit(MDR_SSP2,SSP_HCLKdiv16);
	
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
  SSP_Init (MDR_SSP2,&sSSP);
	
	  /* Enable SSP1 */
  SSP_Cmd(MDR_SSP2, ENABLE);
}


