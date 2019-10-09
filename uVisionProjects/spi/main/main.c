#include "main.h"

void SPI_ini(void)
{
	PORT_InitTypeDef PORT_InitStructure;
	SSP_InitTypeDef sSSP;
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);
	RST_CLK_PCLKcmd(RST_CLK_PCLK_SSP2, ENABLE);
	
	/* Configure SSP1 pins: FSS, CLK, RXD, TXD */
  /* Configure PORTD pins 5, 7, 8, 6 */
  PORT_InitStructure.PORT_Pin	= PORT_Pin_9 | PORT_Pin_11 | PORT_Pin_12;
  PORT_InitStructure.PORT_OE	= PORT_OE_OUT;
  PORT_InitStructure.PORT_FUNC  = PORT_FUNC_MAIN;
	PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PORT_InitStructure.PORT_PD	= PORT_PD_DRIVER;
  PORT_InitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PORT_Init(MDR_PORTC, &PORT_InitStructure);

  PORT_InitStructure.PORT_Pin   = PORT_Pin_10;
  PORT_InitStructure.PORT_OE    = PORT_OE_IN;
	PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PORT_InitStructure.PORT_PD		= PORT_PD_DRIVER;
  PORT_InitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PORT_Init(MDR_PORTC, &PORT_InitStructure);
	
	SSP_BRGInit(MDR_SSP2,SSP_HCLKdiv1);
	
	SSP_StructInit (&sSSP);
	
	sSSP.SSP_SCR  = 0x00;
  sSSP.SSP_CPSDVSR = 2;
  sSSP.SSP_Mode = SSP_ModeMaster;
  sSSP.SSP_WordLength = SSP_WordLength16b;
  sSSP.SSP_SPH = SSP_SPH_1Edge;
  sSSP.SSP_SPO = SSP_SPO_Low;
  sSSP.SSP_FRF = SSP_FRF_SPI_Motorola;
  sSSP.SSP_HardwareFlowControl = SSP_HardwareFlowControl_SSE;
  SSP_Init (MDR_SSP2,&sSSP);
	
	SSP_Cmd(MDR_SSP2, ENABLE);
	
}

void CPU_init (void)
{
	RST_CLK_DeInit();

	MDR_RST_CLK->HS_CONTROL=0x00000001;            			//включили HSE, режим осциллятор (16МГц)
	while((MDR_RST_CLK->CLOCK_STATUS&0x04)==0x00); 			//подождали пока HSE выйдет в штатный режим
	MDR_EEPROM->CMD=5<<3;	

	MDR_RST_CLK->PLL_CONTROL=(8<<8)|(1<<2);            //включили PLL CPU и задали к-т умножения;
	while((MDR_RST_CLK->CLOCK_STATUS&0x02)==0x00);      //подождали пока PLL CPU выйдет в штатный режим

	MDR_EEPROM->CMD=3<<3;                     //задали задержку для обращения к flash-памяти Delay = 3    
	 
	MDR_RST_CLK->CPU_CLOCK|=0x00000106;     
}

RST_CLK_FreqTypeDef Clocks;
int main(void)
{
	uint16_t data_i=0;
	
	CPU_init();
	
	SystemCoreClockUpdate();
	RST_CLK_GetClocksFreq(&Clocks);
	
	SPI_ini();
	
	while(1)
	{
		SSP_SendData(MDR_SSP2, 0x33);
		
		for (data_i = 0;data_i<20000;data_i++)
		{
			
		}
	}
}
