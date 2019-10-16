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

// Процедура настройки PORTC на вывод
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

// Процедура настройки порта для SSP2
void PortsSSP2Init(void) {
    // Настройка выводов SSP2: FSS, CLK, RXD, TXD

    // Настройка PORTD выводов 2, 3, 5, 6
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
  
// Процедура настройки SSP2
void SSP2StructInit(void) {
    // Сбросить все настройка SSP1
    SSP_DeInit(MDR_SSP2);

    // Настройка предделителя
    SSP_BRGInit(MDR_SSP2,SSP_HCLKdiv16);

    // Настройка SSP1 Slave
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

    // Разрешение SSP2
    SSP_Cmd(MDR_SSP2, ENABLE);
}

// Процедура для тестирования SSP2
void SSP2Test(void) {
        uint16_t Src2 = 0x0002;
        uint16_t Dst2 = 0x0000;
        PortsSSP2Init();
        SSP2StructInit();
    // Пока флаг SSP_FLAG_RNE не установлен прием не начинаем
    while (SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_RNE) == RESET)
    {
    }
    // Получение данных по SSP2
    Dst2 = SSP_ReceiveData(MDR_SSP2);
    PORT_SetBits(MDR_PORTC, PORT_Pin_0);
    DELAY(1000000);
    PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
        
}


int main (void)
{
RST_CLK_LSEconfig(RST_CLK_LSE_ON);
    // Включение тактирования
    RST_CLK_LSEconfig(RST_CLK_LSE_ON);
    while (RST_CLK_LSEstatus() != SUCCESS);
    
    RST_CLK_HSEconfig(RST_CLK_HSE_ON);
    while (RST_CLK_HSEstatus() != SUCCESS);

    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);
    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE);
    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);
    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);
    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE, ENABLE);
    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF, ENABLE);
    RST_CLK_PCLKcmd(RST_CLK_PCLK_EBC, ENABLE);
    // Включение тактирования внешних устройств
    RST_CLK_PCLKcmd((RST_CLK_PCLK_RST_CLK | RST_CLK_PCLK_SSP2),ENABLE);
            
    /*
    SCB->AIRCR = 0x05FA0000 | ((uint32_t)0x500);
    SCB->VTOR = 0x08000000;

    NVIC->ICPR[0] = 0xFFFFFFFF;
    NVIC->ICER[0] = 0xFFFFFFFF;

    MDR_DMA->CHNL_REQ_MASK_CLR = 0xFFFFFFFF;
    MDR_DMA->CHNL_USEBURST_CLR = 0xFFFFFFFF;

    PORT_DeInit(MDR_PORTD);
    PORT_DeInit(MDR_PORTF);*/
    
    
    PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
    SetupPortsForDiods();
    
    while (1)
    {
    SSP2Test();
    DELAY(1000000);
    }
}
