#include "MDR32Fx.h"
#include "MDR32F9Qx_config.h"
//#include "MDR32F9Qx_board.h"
#include "MDR32F9Qx_can.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_port.h"
//#include "MDR32F9Qx_it.h"

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/
#define LED1            PORT_Pin_0
#define LED2            PORT_Pin_1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t ret = 0; /* for return of the interrupt handling */
__IO uint32_t rx_buf = 0;
__IO uint32_t extern tx_buf;
volatile TestStatus TestRx;
PORT_InitTypeDef PORT_InitStructure;


void CAN_ports_ini(void) //????????????? ?????? CAN1 
{ 
		PORT_InitTypeDef PortInit;
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
    PORT_Init(MDR_PORTA, &PortInit);
    /* Configure PORTA pins 6 (CAN2_TX) as output */
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_6;
    PORT_Init(MDR_PORTA, &PortInit);	
} 


/**
  * @brief  Configures the CAN, transmit and receive using interrupt.
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
void CAN_Setup(void)
{
  CAN_InitTypeDef  sCAN;
  //RST_CLK_DeInit();
  /* Select HSI/2 as CPU_CLK source*/
  RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);
  uint32_t i = 0;
  RST_CLK_PCLKcmd((RST_CLK_PCLK_RST_CLK | RST_CLK_PCLK_CAN1),ENABLE);
  /* Set the HCLK division factor = 1 for CAN1*/
  CAN_BRGInit(MDR_CAN1,CAN_HCLKdiv1);

  /* CAN register init */
  CAN_DeInit(MDR_CAN1);

  /* CAN cell init */
  CAN_StructInit (&sCAN);

  sCAN.CAN_ROP  = ENABLE;
  sCAN.CAN_SAP  = ENABLE;
  sCAN.CAN_STM  = DISABLE;
  sCAN.CAN_ROM  = DISABLE;
  sCAN.CAN_PSEG = CAN_PSEG_Mul_2TQ;
  sCAN.CAN_SEG1 = CAN_SEG1_Mul_3TQ;
  sCAN.CAN_SEG2 = CAN_SEG2_Mul_2TQ;
  sCAN.CAN_SJW  = CAN_SJW_Mul_2TQ;
  sCAN.CAN_SB   = CAN_SB_3_SAMPLE;
  sCAN.CAN_BRP  = 1;
  CAN_Init (MDR_CAN1,&sCAN);

  CAN_Cmd(MDR_CAN1, ENABLE);

  /* Enable CAN1 interrupt */
  NVIC_EnableIRQ(CAN1_IRQn);

  /* Enable CAN1 GLB_INT and RX_INT interrupts */
  CAN_ITConfig( MDR_CAN1, CAN_IT_GLBINTEN | CAN_IT_RXINTEN, DISABLE);

  /* Enable CAN1 interrupt from receive buffer */
  CAN_RxITConfig( MDR_CAN1 ,(1<<rx_buf), ENABLE);
  /* Enable CAN1 interrupt from transmit buffer */
  CAN_TxITConfig( MDR_CAN1 ,(1<<tx_buf), ENABLE);

  /* receive buffer enable */
  CAN_Receive(MDR_CAN1, rx_buf, DISABLE);
}

