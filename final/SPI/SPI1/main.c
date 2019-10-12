#include "MDR32F9Qx_board.h"
#include "MDR32F9Qx_config.h"
#include "MDR32Fx.h"
#include "MDR32F9Qx_ssp.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_port.h"
#define DELAY(T) for (int i = T; i > 0; i--)

typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

#define BufferSize         32

SSP_InitTypeDef sSSP;
PORT_InitTypeDef PORT_InitStructure;

uint16_t SrcBuf2[BufferSize];
uint16_t DstBuf2[BufferSize];



uint8_t TxIdx = 0, RxIdx = 0;

volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;

void Init_RAM (uint16_t *addr, uint32_t size);
uint32_t ps_rand(uint32_t addr);
TestStatus Verif_mem (uint32_t BufSize, uint16_t *pBuffer1, uint16_t *pBuffer2);

int main(void)
{
  RST_CLK_DeInit();
  RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);
  /* Enable peripheral clocks --------------------------------------------------*/
  RST_CLK_PCLKcmd((RST_CLK_PCLK_RST_CLK | RST_CLK_PCLK_SSP1 | RST_CLK_PCLK_SSP2),ENABLE);

	
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

  /* Configure SSP1 pins: FSS, CLK, RXD, TXD */

  /* Configure PORTF pins 0, 1, 2, 3 */
  PORT_InitStructure.PORT_Pin   = (PORT_Pin_3);
  PORT_InitStructure.PORT_OE    = PORT_OE_IN;
  PORT_Init(MDR_PORTF, &PORT_InitStructure);
  PORT_InitStructure.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2);
  PORT_InitStructure.PORT_OE    = PORT_OE_OUT;
  PORT_Init(MDR_PORTF, &PORT_InitStructure);


  /* Init RAM */
  Init_RAM (DstBuf1, BufferSize);
  Init_RAM (SrcBuf1, BufferSize);
  Init_RAM (DstBuf2, BufferSize);
  Init_RAM (SrcBuf2, BufferSize);

  /* Reset all SSP settings */
  SSP_DeInit(MDR_SSP1);
  SSP_DeInit(MDR_SSP2);

  SSP_BRGInit(MDR_SSP1,SSP_HCLKdiv16);
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
  SSP_Init (MDR_SSP1,&sSSP);

  /* SSP2 SLAVE configuration ------------------------------------------------*/
  sSSP.SSP_SPH = SSP_SPH_1Edge;
  sSSP.SSP_SPO = SSP_SPO_Low;
  sSSP.SSP_CPSDVSR = 12;
  sSSP.SSP_Mode = SSP_ModeSlave;
  SSP_Init (MDR_SSP2,&sSSP);

  /* Enable SSP1 */
  SSP_Cmd(MDR_SSP1, ENABLE);
  /* Enable SSP2 */
  SSP_Cmd(MDR_SSP2, ENABLE);

  /* Transfer procedure */
  while (TxIdx < BufferSize)
  {
		uint16_t Src1 = 0x0002;
		uint16_t Src2 = 0x0300;
		uint16_t Dst1 = 0x0000;
		uint16_t Dst2 = 0x0000;
    /* Wait for SPI1 Tx buffer empty */
    while (SSP_GetFlagStatus(MDR_SSP1, SSP_FLAG_TFE) == RESET)
    {
    }
    /* Send SPI1 data */
    SSP_SendData(MDR_SSP1, Src1);
    /* Send SPI2 data */
    SSP_SendData(MDR_SSP2, Src2);
    /* Wait for SPI1 data reception */
    while (SSP_GetFlagStatus(MDR_SSP1, SSP_FLAG_RNE) == RESET)
    {
    }
    /* Read SPI1 received data */
    Dst1 = SSP_ReceiveData(MDR_SSP1);
    /* Wait for SPI2 data reception */
    while (SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_RNE) == RESET)
    {
    }
    /* Read SPI2 received data */
    Dst2 = SSP_ReceiveData(MDR_SSP2);
		DELAY(10);
  }

  /* Check the corectness of written dada */
  TransferStatus1 = Verif_mem ((BufferSize), SrcBuf1, DstBuf2);
  TransferStatus2 = Verif_mem ((BufferSize), SrcBuf2, DstBuf1);
  /* TransferStatus1, TransferStatus2 = PASSED, if the data transmitted and received
     are correct */
  /* TransferStatus1, TransferStatus2 = FAILED, if the data transmitted and received
     are different */

  while(1)
  {
  }
}

void Init_RAM (uint16_t *addr, uint32_t size)
{
uint32_t i;

  for (i = 0; i < size; i++)
  {
    *addr++ = ps_rand((uint32_t)addr) + (i*4);
  }
}

uint32_t ps_rand(uint32_t addr)
{
uint32_t hash = 0;
uint32_t i;
char *key = (char *)&addr;

  for (i = 0; i < 4; i++)
  {
    hash += key[i];
    hash += (hash << 10);
    hash ^= (hash >> 6);
  }

  for (i = 0; i < 256; i++)
  {
    hash += (hash << 10);
    hash ^= (hash >> 6);
  }

  hash += (hash << 3);
  hash ^= (hash >> 11);
  hash += (hash << 15);

  return hash;
}

TestStatus Verif_mem (uint32_t BufSize, uint16_t *pBuffer1, uint16_t *pBuffer2)
{
uint32_t i;

  for(i = 0; i < BufSize; i++)
  {
    if (*pBuffer1++ != *pBuffer2++)
    {
      return(FAILED);
    }
  }
  return(PASSED);
}
