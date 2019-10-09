// ***********************************************************************************
// Микроконтроллер: 1986ВЕ91
// Устройство: Отладочная плата К1986ВЕ92QI
// Файл: rst.c 
// Назначение: Настройка тактирования микроконтроллера
// Компилятор:  Armcc 5.03.0.76 из комплекта Keil uVision 4.72.1.0 
// ***********************************************************************************

#include "rst.h"

// Инициализация системы тактирования МК
void U_RST_Init (void)
{
  // Включить тактирование батарейного блока и внутренние генераторы, все остальное сбросить
  RST_CLK_DeInit (); 

  // Включить генератор на внешнем кварце
  RST_CLK_HSEconfig (RST_CLK_HSE_ON); 
  while (RST_CLK_HSEstatus() != SUCCESS);

  // Настроить источник и КУ PLL (CPU_C1_SEL = HSE)
  RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSEdiv1, RST_CLK_CPU_PLLmul10); 
 
  // Включить PLL, но еще не подключать к кристаллу
  RST_CLK_CPU_PLLcmd (ENABLE); 
  while (RST_CLK_CPU_PLLstatus () != SUCCESS);

  // Делитель С3( CPU_C3_SEL = CPU_C2_SEL )
  RST_CLK_CPUclkPrescaler( RST_CLK_CPUclkDIV1); 
  
  // На С2 идет с PLL, а не напрямую с С1 (CPU_C2_SEL = PLL)
  RST_CLK_CPU_PLLuse (ENABLE); 
  
  // CPU берет с выхода С3 (а может с выхода HSI,LSI,LSE) (HCLK_SEL = CPU_C3_SEL )
  RST_CLK_CPUclkSelection (RST_CLK_CPUclkCPU_C3); 

}

