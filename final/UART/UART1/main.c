#include "MDR32F9Qx_board.h"
#include "MDR32F9Qx_config.h"
#include "MDR32Fx.h"
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_rst_clk.h"
#define DELAY(T) for (int i = T; i > 0; i--)

static PORT_InitTypeDef PortInit;
static UART_InitTypeDef UART_InitStructure;

// Процедура конфигурирования PORTC для использования диода
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

// Процедура конфигурирования портов для UART2
void Uart2PinCfg(void)
{
	// Заполнение структуры порта
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_OVERRID;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
    // Конфигурирование PORTF PIN 0 (UART2_RX) как ввода
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = PORT_Pin_0;
    PORT_Init(MDR_PORTF, &PortInit);
    // Конфигурирование PORTF PIN 1 (UART2_TX) как вывода
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_1;
    PORT_Init(MDR_PORTF, &PortInit);	
}

// Процедура конфигурирования UART2
void Uart2Setup(void)
{
    RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);
    // Разрешение тактирования на UART2
	RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
    // Выбор предделителя частоты
	UART_BRGInit(MDR_UART2, UART_HCLKdiv1);
    // Заполнение структуры UART
    UART_InitStructure.UART_BaudRate                = 9600;
    UART_InitStructure.UART_WordLength              = UART_WordLength8b;
    UART_InitStructure.UART_StopBits                = UART_StopBits1;
    UART_InitStructure.UART_Parity                  = UART_Parity_No;
    UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
    UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;

	// Конфигурирование параметров UART2
	UART_Init (MDR_UART2,&UART_InitStructure);
    // Включение UART2
    UART_Cmd(MDR_UART2,ENABLE);
}

// Процедура тестирования UART
int UartTest (void)
{
uint8_t DataByte = 'm';

	Uart2PinCfg();
	Uart2Setup();

    // Проверка флага RXFF
    do
    {
        if (PORT_ReadInputDataBit(MDR_PORTE,PORT_Pin_3) == 0)
        {
            goto exit;
        }
    }
    while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE)!= SET);
    // Отправка данных
    UART_SendData (MDR_UART2,DataByte);
    // Выключение светодиода
    PORT_ResetBits(MDR_PORTC, PORT_Pin_0);
    DELAY(1000000);
exit: return 0;
}

int main (void)
{
    // Включение тактирования
    RST_CLK_LSEconfig(RST_CLK_LSE_ON);
    while (RST_CLK_LSEstatus() != SUCCESS);
    
	RST_CLK_HSEconfig(RST_CLK_HSE_ON);
    while (RST_CLK_HSEstatus() != SUCCESS);
    
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);
    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF, ENABLE);
    
	SetupPortsForDiods();
    
	while (1)
	{
        // Включение светодиода
		PORT_SetBits(MDR_PORTC, PORT_Pin_0);
		DELAY(1000000);
        // Запуск теста UART
		UartTest();
	}
}

