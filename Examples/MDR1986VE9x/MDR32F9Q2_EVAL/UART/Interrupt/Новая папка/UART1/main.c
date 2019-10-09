#include "MDR32F9Qx_board.h"
#include "MDR32F9Qx_config.h"
#include "MDR32Fx.h"
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_rst_clk.h"

static PORT_InitTypeDef PortInit;
static UART_InitTypeDef UART_InitStructure;
uint32_t uart1_IT_TX_flag = RESET;

// Инициализация порта для UART1
void UART1_Port_init (void)
{
    // Разрешение тактирования на PORTB
    RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB,ENABLE);
    
    // Инициализация структуры порта
    PortInit.PORT_PULL_UP = PORT_PULL_UP_OFF;
    PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
    PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
    PortInit.PORT_PD = PORT_PD_DRIVER;
    PortInit.PORT_GFEN = PORT_GFEN_OFF;
    PortInit.PORT_FUNC = PORT_FUNC_ALTER;
    PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
    PortInit.PORT_MODE = PORT_MODE_DIGITAL;
    
    // Конфигурирование PORTB pin 5 (UART1_TX) как вывода и инициализация порта
    PortInit.PORT_OE = PORT_OE_OUT;
    PortInit.PORT_Pin = PORT_Pin_5;
    PORT_Init(MDR_PORTB, &PortInit);
    
    // Конфигурирование PORTB pin 6 (UART1_RX) как ввода и инициализация порта
    PortInit.PORT_OE = PORT_OE_IN;
    PortInit.PORT_Pin = PORT_Pin_6;
    PORT_Init(MDR_PORTB, &PortInit);
}

// Инициализация UART1
void UART1_init (void)
{
    // Выбор тактовой часторы
    RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);
    
    // Разрешение тактирования UART1
    RST_CLK_PCLKcmd(RST_CLK_PCLK_UART1, ENABLE);
    
    // Инициализация делителя частоты (значение по умолчанию 1)
    UART_BRGInit(MDR_UART1, UART_HCLKdiv1);
    
    NVIC_EnableIRQ(UART1_IRQn);
    
    // Инициализания структуры UART
    UART_InitStructure.UART_BaudRate                = 115000;
    UART_InitStructure.UART_WordLength              = UART_WordLength8b;
    UART_InitStructure.UART_StopBits                = UART_StopBits1;
    UART_InitStructure.UART_Parity                  = UART_Parity_No;
    UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
    UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
    
    // Конфигурирование структуры UART1
    UART_Init (MDR_UART1,&UART_InitStructure);

    // Разрешение прерывания UART1
    UART_ITConfig (MDR_UART1, UART_IT_TX, ENABLE);

    // Включение
    UART_Cmd(MDR_UART1,ENABLE);
}

// Обработчик прерывания UART1
void UART1_IRQHandler(void)
{
  if (UART_GetITStatusMasked(MDR_UART1, UART_IT_TX) == SET)
  {
    UART_ClearITPendingBit(MDR_UART1, UART_IT_TX);
    uart1_IT_TX_flag = SET;
  }
}

void main (void)
{
    uint8_t DataByte=0x00;
    UART1_Port_init();
    UART1_init();

    while (1)
    {
        // Отправка данных
        UART_SendData (MDR_UART1,DataByte);

        // Ожидание пока флаг uart1_IT_TX_flag будет установлен
        while (uart1_IT_TX_flag != SET)
        {
        }

        // Очищение флаг uart1_IT_TX_flag
        uart1_IT_TX_flag = RESET;

        // Увеличиваем данные
        DataByte++;
    }
}
