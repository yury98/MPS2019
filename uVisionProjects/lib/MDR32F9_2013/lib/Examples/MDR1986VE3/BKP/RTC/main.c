/**
  ******************************************************************************
  * @file    main.c
  * @author  Milandr Application Team
  * @version V1.2.0
  * @date    13/06/2013
  * @brief   Main program body.
  ******************************************************************************
  * <br><br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MILANDR SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2013 Milandr</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_bkp.h"
#include "MDR1986VE3_IT.h"

/** @addtogroup __MDR32F9Qx_StdPeriph_Examples MDR32F9Qx StdPeriph Examples
  * @{
  */

/** @addtogroup __MDR1986VE3_EVAL MDR1986VE3 Complexity Evaluation Board
  * @{
  */

/** @addtogroup BKP_RTC_3T BKP_RTC
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RTC_LSI_CLK



#define COUNT_VALUE		0
#define ALARM_VALUE		60
#define PRESC_VALUE_HS	8000000
#define PRESC_VALUE_LS	32000
#define RTCHS_PRESC		RST_CLK_HSIclkDIV8
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PORT_InitTypeDef PORT_InitStructure;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  /* Enables the HSI clock on PORTD */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE);

  /* Configure all unused PORT pins to low power consumption */
  PORT_StructInit(&PORT_InitStructure);
  PORT_InitStructure.PORT_Pin = (PORT_Pin_All & ~(PORT_Pin_0 | PORT_Pin_1));
  PORT_Init(MDR_PORTB, &PORT_InitStructure);


  /* Configure PORTB pins 0..1 for output to switch LEDs on/off */
  PORT_InitStructure.PORT_Pin   = (PORT_Pin_0 | PORT_Pin_1);
  PORT_InitStructure.PORT_OE    = PORT_OE_OUT;
  PORT_InitStructure.PORT_FUNC  = PORT_FUNC_PORT;
  PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PORT_InitStructure.PORT_SPEED = PORT_SPEED_SLOW;

  PORT_Init(MDR_PORTB, &PORT_InitStructure);

  /* Enables the HSI clock for BKP control */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_BKP,ENABLE);

  /* RTC reset */
  BKP_RTC_Reset(ENABLE);
  BKP_RTC_Reset(DISABLE);

#ifdef RTC_HSI_CLK
  /* Configure RTCHSI as RTC clock source */
  RST_CLK_HSIadjust(25);
  RST_CLK_HSIclkPrescaler(RTCHS_PRESC);
  BKP_RTCclkSource(BKP_RTC_HSIclk);
  RST_CLK_RTC_HSIclkEnable(ENABLE);
#endif
#ifdef RTC_HSE_CLK
  /* Configure RTCHSE as RTC clock source */
  RST_CLK_HSEconfig(RST_CLK_HSE_ON);
  while (RST_CLK_HSEstatus()!=SUCCESS);
  RST_CLK_RTC_HSEclkEnable(ENABLE);
  RST_CLK_HSEclkPrescaler(RTCHS_PRESC);
  BKP_RTCclkSource(BKP_RTC_HSEclk);
#endif
#ifdef RTC_LSI_CLK
  /* Configure LSI as RTC clock source */
  RST_CLK_LSIadjust(12);
  BKP_RTCclkSource(BKP_RTC_LSIclk);
  while (RST_CLK_LSIstatus()!=SUCCESS);
#endif
#ifdef RTC_LSE_CLK
  /* Configure LSE as RTC clock source */
  RST_CLK_LSEconfig(RST_CLK_LSE_ON);
  while (RST_CLK_LSEstatus()!=SUCCESS);
  BKP_RTCclkSource(BKP_RTC_LSEclk);
#endif

  /* Set the RTC prescaler value */
  BKP_RTC_WaitForUpdate();
#ifdef RTC_HSI_CLK
  BKP_RTC_SetPrescaler(PRESC_VALUE_HS);
#endif
#ifdef RTC_HSE_CLK
  BKP_RTC_SetPrescaler(PRESC_VALUE_HS);
#endif
#ifdef RTC_LSI_CLK
  BKP_RTC_SetPrescaler(PRESC_VALUE_LS);
#endif
#ifdef RTC_LSE_CLK
  BKP_RTC_SetPrescaler(PRESC_VALUE_LS);
#endif

  /* Set the RTC alarm value */
  BKP_RTC_WaitForUpdate();
  BKP_RTC_SetAlarm(ALARM_VALUE);

  /* Set the RTC counter value */
  BKP_RTC_WaitForUpdate();
  BKP_RTC_SetCounter(COUNT_VALUE);

  /* Enable all RTC interrupts */
  BKP_RTC_ITConfig(BKP_RTC_IT_ALRF | BKP_RTC_IT_SECF | BKP_RTC_IT_OWF,ENABLE);
  NVIC_EnableIRQ(BKP_IRQn);

  /* RTC enable */
  BKP_RTC_WaitForUpdate();
  BKP_RTC_Enable(ENABLE);

  while(1)
  {
  }
}

/**
  * @brief  Reports the source file ID, the source line number
  *         and expression text (if USE_ASSERT_INFO == 2) where
  *         the assert_param error has occurred.
  * @param  file_id: pointer to the source file name
  * @param  line: assert_param error line source number
  * @param  expr:
  * @retval None
  */
#if (USE_ASSERT_INFO == 1)
void assert_failed(uint32_t file_id, uint32_t line)
{
  /* User can add his own implementation to report the source file ID and line number.
     Ex: printf("Wrong parameters value: file Id %d on line %d\r\n", file_id, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#elif (USE_ASSERT_INFO == 2)
void assert_failed(uint32_t file_id, uint32_t line, const uint8_t* expr);
{
  /* User can add his own implementation to report the source file ID, line number and
     expression text.
     Ex: printf("Wrong parameters value (%s): file Id %d on line %d\r\n", expr, file_id, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_ASSERT_INFO */

/** @} */ /* End of group BKP_RTC_3T */

/** @} */ /* End of group __MDR1986VE3_EVAL */

/** @} */ /* End of group __MDR32F9Qx_StdPeriph_Examples */

/******************* (C) COPYRIGHT 2013 Milandr *********/

/* END OF FILE main.c */
