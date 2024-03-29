/**
  ******************************************************************************
  * @file    opora_rst_clk.h
  * @author  Phyton Application Team and Milandr Application Team
  * @version V1.0.0
  * @date    16/06/2010
  * @brief   This file contains all the functions prototypes for the RST_CLK
  *          firmware library.
  ******************************************************************************
  * <br><br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON AND MILANDR SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 Phyton</center></h2>
  ******************************************************************************
  * FILE opora_rst_clk.h
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OPORA_RST_CLK_H
#define __OPORA_RST_CLK_H

/* Includes ------------------------------------------------------------------*/
#include "opora.h"
#include "1986BE9x_lib.h"

/** @addtogroup __1986BE9x_StdPeriph_Driver 1986BE9x Standard Peripherial Driver
  * @{
  */

/** @addtogroup RST_CLK
  * @{
  */

/** @defgroup RST_CLK_Exported_Types RST_CLK Exported Types
  * @{
  */

/**
  * @brief Structure type for modules clocks frequencies expressed in Hz
  */

typedef struct
{
  uint32_t CPU_CLK_Frequency;
  uint32_t USB_CLK_Frequency;
  uint32_t ADC_CLK_Frequency;
  uint32_t RTCHSI_Frequency;
  uint32_t RTCHSE_Frequency;
}RST_CLK_FreqTypeDef;

/**
  * @brief RST_CLK unit non-volatile settings init structure
  */

typedef struct {
     uint32_t REG_0F;
} Init_NonVolatile_RST_CLK_TypeDef;

/** @} */ /* End of group RST_CLK_Exported_Types */

/** @defgroup RST_CLK_Exported_Constants RST_CLK Exported Constants
  * @{
  */

/** @defgroup HSE_configuration HSE configuration
  * @{
  */

/**
  * @brief RST_CLK HSE (High Speed External) clock mode and source selection constants
  */

#define RST_CLK_HSE_OFF                      ((uint32_t)0x00000000)
#define RST_CLK_HSE_ON                       ((uint32_t)0x00000001)
#define RST_CLK_HSE_Bypass                   ((uint32_t)0x00000002)

#define IS_RST_CLK_HSE(HSE)                  (((HSE) == RST_CLK_HSE_OFF) || \
                                              ((HSE) == RST_CLK_HSE_ON)  || \
                                              ((HSE) == RST_CLK_HSE_Bypass))

/** @} */ /* End of group HSE_configuration */

/** @defgroup LSE_configuration LSE configuration
  * @{
  */

/**
  * @brief RST_CLK LSE (Low Speed External) clock mode and source selection constants
  */

#define RST_CLK_LSE_OFF                      ((uint32_t)0x00000000)
#define RST_CLK_LSE_ON                       ((uint32_t)0x00000001)
#define RST_CLK_LSE_Bypass                   ((uint32_t)0x00000002)

#define IS_RST_CLK_LSE(LSE)                  (((LSE) == RST_CLK_LSE_OFF) || \
                                              ((LSE) == RST_CLK_LSE_ON)  || \
                                              ((LSE) == RST_CLK_LSE_Bypass))

/** @} */ /* End of group LSE_configuration */

/** @defgroup CPU_PLL_entry_clock_source CPU PLL entry clock source
  * @{
  */

/**
  * @brief RST_CLK CPU_PLL clock mode and source selection constants
  */

#define RST_CLK_CPU_PLLsrcHSIdiv1           ((uint32_t)0x00000000)
#define RST_CLK_CPU_PLLsrcHSIdiv2           ((uint32_t)0x00000001)
#define RST_CLK_CPU_PLLsrcHSEdiv1           ((uint32_t)0x00000002)
#define RST_CLK_CPU_PLLsrcHSEdiv2           ((uint32_t)0x00000003)

#define IS_RST_CLK_CPU_PLL_SOURCE(SRC)      (((SRC) == RST_CLK_CPU_PLLsrcHSIdiv1) || \
                                             ((SRC) == RST_CLK_CPU_PLLsrcHSIdiv2) || \
                                             ((SRC) == RST_CLK_CPU_PLLsrcHSEdiv1) || \
                                             ((SRC) == RST_CLK_CPU_PLLsrcHSEdiv2))

/** @} */ /* End of group CPU_PLL_entry_clock_source */

/** @defgroup CPU_PLL_clock_multiplier CPU_PLL clock multiplier
  * @{
  */

/**
  * @brief RST_CLK PLL_CPU_MUL multiplier of the CPU_C1 clock constants
  */

#define RST_CLK_CPU_PLLmul1                     ((uint32_t)0x00000000)
#define RST_CLK_CPU_PLLmul2                     ((uint32_t)0x00000001)
#define RST_CLK_CPU_PLLmul3                     ((uint32_t)0x00000002)
#define RST_CLK_CPU_PLLmul4                     ((uint32_t)0x00000003)
#define RST_CLK_CPU_PLLmul5                     ((uint32_t)0x00000004)
#define RST_CLK_CPU_PLLmul6                     ((uint32_t)0x00000005)
#define RST_CLK_CPU_PLLmul7                     ((uint32_t)0x00000006)
#define RST_CLK_CPU_PLLmul8                     ((uint32_t)0x00000007)
#define RST_CLK_CPU_PLLmul9                     ((uint32_t)0x00000008)
#define RST_CLK_CPU_PLLmul10                    ((uint32_t)0x00000009)
#define RST_CLK_CPU_PLLmul11                    ((uint32_t)0x0000000A)
#define RST_CLK_CPU_PLLmul12                    ((uint32_t)0x0000000B)
#define RST_CLK_CPU_PLLmul13                    ((uint32_t)0x0000000C)
#define RST_CLK_CPU_PLLmul14                    ((uint32_t)0x0000000D)
#define RST_CLK_CPU_PLLmul15                    ((uint32_t)0x0000000E)
#define RST_CLK_CPU_PLLmul16                    ((uint32_t)0x0000000F)

#define IS_RST_CLK_CPU_PLL_MUL(SRC)             ((SRC>=0) && (SRC<16))

/** @} */ /* End of group CPU_PLL_clock_multiplier */


/** @defgroup USB_PLL_entry_clock_source USB PLL entry clock source
  * @{
  */

/**
  * @brief RST_CLK USB_PLL clock mode and source selection constants
  */

#define RST_CLK_USB_PLLsrcHSIdiv1           ((uint32_t)0x00000000)
#define RST_CLK_USB_PLLsrcHSIdiv2           ((uint32_t)0x00000001)
#define RST_CLK_USB_PLLsrcHSEdiv1           ((uint32_t)0x00000002)
#define RST_CLK_USB_PLLsrcHSEdiv2           ((uint32_t)0x00000003)

#define IS_RST_CLK_USB_PLL_SOURCE(SRC)      (((SRC) == RST_CLK_USB_PLLsrcHSIdiv1) ||\
                                             ((SRC) == RST_CLK_USB_PLLsrcHSIdiv2) ||\
                                             ((SRC) == RST_CLK_USB_PLLsrcHSEdiv1) ||\
                                             ((SRC) == RST_CLK_USB_PLLsrcHSEdiv2))

/** @} */ /* End of group USB_PLL_entry_clock_source */

/** @defgroup USB_PLL_clock_multiplier USB_PLL clock multiplier
  * @{
  */

/**
  * @brief RST_CLK USB_C1 clock PLL_USB_MUL multiplier constants
  */

#define RST_CLK_USB_PLLmul1                     ((uint32_t)0x00000000)
#define RST_CLK_USB_PLLmul2                     ((uint32_t)0x00000001)
#define RST_CLK_USB_PLLmul3                     ((uint32_t)0x00000002)
#define RST_CLK_USB_PLLmul4                     ((uint32_t)0x00000003)
#define RST_CLK_USB_PLLmul5                     ((uint32_t)0x00000004)
#define RST_CLK_USB_PLLmul6                     ((uint32_t)0x00000005)
#define RST_CLK_USB_PLLmul7                     ((uint32_t)0x00000006)
#define RST_CLK_USB_PLLmul8                     ((uint32_t)0x00000007)
#define RST_CLK_USB_PLLmul9                     ((uint32_t)0x00000008)
#define RST_CLK_USB_PLLmul10                    ((uint32_t)0x00000009)
#define RST_CLK_USB_PLLmul11                    ((uint32_t)0x0000000A)
#define RST_CLK_USB_PLLmul12                    ((uint32_t)0x0000000B)
#define RST_CLK_USB_PLLmul13                    ((uint32_t)0x0000000C)
#define RST_CLK_USB_PLLmul14                    ((uint32_t)0x0000000D)
#define RST_CLK_USB_PLLmul15                    ((uint32_t)0x0000000E)
#define RST_CLK_USB_PLLmul16                    ((uint32_t)0x0000000F)

#define IS_RST_CLK_USB_PLL_MUL(SRC)             ((SRC>=0) && (SRC<16))

/** @} */ /* End of group USB_PLL_clock_multiplier */


/** @defgroup CPU_CLK_divider CPU CLK divider
  * @{
  */

/**
  * @brief RST_CLK CPU_PLL output clock CPU_CLK_DIV divider constants
  */

#define RST_CLK_CPUclkDIV1                    ((uint32_t)0x00000000)
#define RST_CLK_CPUclkDIV2                    ((uint32_t)0x00000080)
#define RST_CLK_CPUclkDIV4                    ((uint32_t)0x00000090)
#define RST_CLK_CPUclkDIV8                    ((uint32_t)0x000000A0)
#define RST_CLK_CPUclkDIV16                   ((uint32_t)0x000000B0)
#define RST_CLK_CPUclkDIV32                   ((uint32_t)0x000000C0)
#define RST_CLK_CPUclkDIV64                   ((uint32_t)0x000000D0)
#define RST_CLK_CPUclkDIV128                  ((uint32_t)0x000000E0)
#define RST_CLK_CPUclkDIV256                  ((uint32_t)0x000000F0)

#define IS_RST_CLK_CPUclkDIV(DIV)             (((DIV) == RST_CLK_CPUclkDIV1)   || \
                                               ((DIV) == RST_CLK_CPUclkDIV2)   || \
                                               ((DIV) == RST_CLK_CPUclkDIV4)   || \
                                               ((DIV) == RST_CLK_CPUclkDIV8)   || \
                                               ((DIV) == RST_CLK_CPUclkDIV16)  || \
                                               ((DIV) == RST_CLK_CPUclkDIV32)  || \
                                               ((DIV) == RST_CLK_CPUclkDIV64)  || \
                                               ((DIV) == RST_CLK_CPUclkDIV128) || \
                                               ((DIV) == RST_CLK_CPUclkDIV256))

/** @} */ /* End of group CPU_CLK_divider */

/** @defgroup CPU_CLK_selector CPU CLK selector
  * @{
  */

/**
  * @brief RST_CLK CPU_CLK source CPU_CLK selector constants
  */

#define RST_CLK_CPUclkHSI                     ((uint32_t)0x00000000)
#define RST_CLK_CPUclkCPU_C3                  ((uint32_t)0x00000100)
#define RST_CLK_CPUclkLSE                     ((uint32_t)0x00000200)
#define RST_CLK_CPUclkLSI                     ((uint32_t)0x00000300)

#define IS_RST_CPU_CLK(SRC)                   (((SRC) == RST_CLK_CPUclkHSI)    || \
                                               ((SRC) == RST_CLK_CPUclkCPU_C3) || \
                                               ((SRC) == RST_CLK_CPUclkLSE)    || \
                                               ((SRC) == RST_CLK_CPUclkLSI))

/** @} */ /* End of group CPU_CLK_selector */

/** @defgroup ADC_MCO_CLOCK_source ADC clock source
  * @{
  */

/**
  * @brief RST_CLK ADC_CLK selector constants
  */
#define RST_CLK_ADCclkCPU_C1                  ((uint32_t)0x00000020)
#define RST_CLK_ADCclkUSB_C1                  ((uint32_t)0x00000021)
#define RST_CLK_ADCclkCPU_C2                  ((uint32_t)0x00000022)
#define RST_CLK_ADCclkUSB_C2                  ((uint32_t)0x00000023)
#define RST_CLK_ADCclkLSE                     ((uint32_t)0x00000000)
#define RST_CLK_ADCclkLSI                     ((uint32_t)0x00000010)
#define RST_CLK_ADCclkHSI_C1                  ((uint32_t)0x00000030)

#define IS_RST_CLK_ADCclk(SRC)                (((SRC) == RST_CLK_ADCclkCPU_C1) || \
                                               ((SRC) == RST_CLK_ADCclkUSB_C1) || \
                                               ((SRC) == RST_CLK_ADCclkCPU_C2) || \
                                               ((SRC) == RST_CLK_ADCclkUSB_C2) || \
                                               ((SRC) == RST_CLK_ADCclkLSE)    || \
                                               ((SRC) == RST_CLK_ADCclkLSI)    || \
                                               ((SRC) == RST_CLK_ADCclkHSI_C1))
/** @} */ /* End of group ADC_MCO_CLOCK_source */

/** @defgroup ADC_MCO_CS3_SEL ADC clock divider
  * @{
  */

/**
  * @brief RST_CLK ADC_CS2_SEL output clock ADC_CS3_SEL divider constants
  */

#define RST_CLK_ADCclkDIV1                    ((uint32_t)0x00000000)
#define RST_CLK_ADCclkDIV2                    ((uint32_t)0x00000001)
#define RST_CLK_ADCclkDIV3                    ((uint32_t)0x00000002)
#define RST_CLK_ADCclkDIV4                    ((uint32_t)0x00000003)
#define RST_CLK_ADCclkDIV5                    ((uint32_t)0x00000004)
#define RST_CLK_ADCclkDIV6                    ((uint32_t)0x00000005)
#define RST_CLK_ADCclkDIV7                    ((uint32_t)0x00000006)
#define RST_CLK_ADCclkDIV8                    ((uint32_t)0x00000007)
#define RST_CLK_ADCclkDIV9                    ((uint32_t)0x00000008)
#define RST_CLK_ADCclkDIV10                   ((uint32_t)0x00000009)
#define RST_CLK_ADCclkDIV11                   ((uint32_t)0x0000000A)
#define RST_CLK_ADCclkDIV12                   ((uint32_t)0x0000000B)
#define RST_CLK_ADCclkDIV13                   ((uint32_t)0x0000000C)
#define RST_CLK_ADCclkDIV14                   ((uint32_t)0x0000000D)
#define RST_CLK_ADCclkDIV15                   ((uint32_t)0x0000000E)
#define RST_CLK_ADCclkDIV16                   ((uint32_t)0x0000000F)

#define IS_RST_CLK_ADCclkDivValue(SRC)        ((SRC>=0) && (SRC<16))

/** @} */ /* End of group ADC_MCO_CS3_SEL */


/** @defgroup CLK_peripheral CLK peripheral
  * @{
  */

/**
  * @brief RST_CLK peripheral modules clock constants
  */

#define PCLK_BIT(BASE)              ((uint32_t)(1 << ((((uint32_t)BASE) >> 15) & 0x1F)))

#define RST_CLK_PCLK_CAN1           PCLK_BIT(CAN1_BASE)
#define RST_CLK_PCLK_CAN2           PCLK_BIT(CAN2_BASE)
#define RST_CLK_PCLK_USB            PCLK_BIT(USB_BASE)
#define RST_CLK_PCLK_EEPROM         PCLK_BIT(EEPROM_BASE)
#define RST_CLK_PCLK_RST_CLK        PCLK_BIT(RST_CLK_BASE)
#define RST_CLK_PCLK_DMA            PCLK_BIT(DMA_BASE)
#define RST_CLK_PCLK_UART1          PCLK_BIT(UART1_BASE)
#define RST_CLK_PCLK_UART2          PCLK_BIT(UART2_BASE)
#define RST_CLK_PCLK_SPI1           PCLK_BIT(SPI1_BASE)
#define RST_CLK_PCLK_MIL_STD_15531  PCLK_BIT(MIL_STD_15531_BASE)
#define RST_CLK_PCLK_MIL_STD_15532  PCLK_BIT(MIL_STD_15532_BASE)
#define RST_CLK_PCLK_POWER          PCLK_BIT(POWER_BASE)
#define RST_CLK_PCLK_WWDG           PCLK_BIT(WWDG_BASE)
#define RST_CLK_PCLK_IWDG           PCLK_BIT(IWDG_BASE)
#define RST_CLK_PCLK_TIMER1         PCLK_BIT(TIMER1_BASE)
#define RST_CLK_PCLK_TIMER2         PCLK_BIT(TIMER2_BASE)
#define RST_CLK_PCLK_TIMER3         PCLK_BIT(TIMER3_BASE)
#define RST_CLK_PCLK_ADC            PCLK_BIT(ADC_BASE)
#define RST_CLK_PCLK_DAC            PCLK_BIT(DAC_BASE)
#define RST_CLK_PCLK_TIMER4         PCLK_BIT(TIMER4_BASE)
#define RST_CLK_PCLK_SPI2           PCLK_BIT(SPI2_BASE)
#define RST_CLK_PCLK_PORTA          PCLK_BIT(PORTA_BASE)
#define RST_CLK_PCLK_PORTB          PCLK_BIT(PORTB_BASE)
#define RST_CLK_PCLK_PORTC          PCLK_BIT(PORTC_BASE)
#define RST_CLK_PCLK_PORTD          PCLK_BIT(PORTD_BASE)
#define RST_CLK_PCLK_PORTE          PCLK_BIT(PORTE_BASE)
#define RST_CLK_PCLK_ARINC429R      PCLK_BIT(ARINC429R_BASE)
#define RST_CLK_PCLK_BKP            PCLK_BIT(BKP_BASE)
#define RST_CLK_PCLK_ARINC429T      PCLK_BIT(ARINC429T_BASE)
#define RST_CLK_PCLK_PORTF          PCLK_BIT(PORTF_BASE)
#define RST_CLK_PCLK_EXT_BUS_CNTRL  PCLK_BIT(EXT_BUS_BASE)
#define RST_CLK_PCLK_SPI3           PCLK_BIT(SPI3_BASE)

#define IS_RST_CLK_PCLK(PCLK)       (((PCLK)>=0) && ((PCLK)<32))

/** @} */ /* End of group CLK_peripheral */

/** @defgroup RST_CLK_Flag RST_CLK Flag
  * @{
  */

/**
  * @brief RST_CLK flag identifiers
  */

#define RST_CLK_FLAG_HSIRDY                   ((uint32_t)(0x00 | 23))
#define RST_CLK_FLAG_LSIRDY                   ((uint32_t)(0x00 | 21))
#define RST_CLK_FLAG_LSERDY                   ((uint32_t)(0x00 | 13))

#define RST_CLK_FLAG_HSERDY                   ((uint32_t)(0x20 |  2))
#define RST_CLK_FLAG_PLLCPURDY                ((uint32_t)(0x20 |  1))
#define RST_CLK_FLAG_PLLUSBRDY                ((uint32_t)(0x20 |  0))

#define IS_RST_CLK_FLAG(FLAG)                 (((FLAG) == RST_CLK_FLAG_HSIRDY)    || \
                                               ((FLAG) == RST_CLK_FLAG_LSIRDY)    || \
                                               ((FLAG) == RST_CLK_FLAG_HSERDY)    || \
                                               ((FLAG) == RST_CLK_FLAG_LSERDY)    || \
                                               ((FLAG) == RST_CLK_FLAG_PLLCPURDY) || \
                                               ((FLAG) == RST_CLK_FLAG_PLLUSBRDY))

/** @} */ /* End of group RST_CLK_Flag */

#define IS_RCC_CLK_HSI_TRIM_VALUE(TRIM)       ((TRIM) <= 0x3F)
#define IS_RCC_CLK_LSI_TRIM_VALUE(TRIM)       ((TRIM) <= 0x1F)

/** @defgroup RST_CLK_HSI_C1_SEL HSI clock divider
  * @{
  */

/**
  * @brief RST_CLK HSI clock HSI_C1_SEL divider constants
  */

#define RST_CLK_HSIclkDIV1                    ((uint32_t)0x00000000)
#define RST_CLK_HSIclkDIV2                    ((uint32_t)0x00000001)
#define RST_CLK_HSIclkDIV3                    ((uint32_t)0x00000002)
#define RST_CLK_HSIclkDIV4                    ((uint32_t)0x00000003)
#define RST_CLK_HSIclkDIV5                    ((uint32_t)0x00000004)
#define RST_CLK_HSIclkDIV6                    ((uint32_t)0x00000005)
#define RST_CLK_HSIclkDIV7                    ((uint32_t)0x00000006)
#define RST_CLK_HSIclkDIV8                    ((uint32_t)0x00000007)
#define RST_CLK_HSIclkDIV9                    ((uint32_t)0x00000008)
#define RST_CLK_HSIclkDIV10                   ((uint32_t)0x00000009)
#define RST_CLK_HSIclkDIV11                   ((uint32_t)0x0000000A)
#define RST_CLK_HSIclkDIV12                   ((uint32_t)0x0000000B)
#define RST_CLK_HSIclkDIV13                   ((uint32_t)0x0000000C)
#define RST_CLK_HSIclkDIV14                   ((uint32_t)0x0000000D)
#define RST_CLK_HSIclkDIV15                   ((uint32_t)0x0000000E)
#define RST_CLK_HSIclkDIV16                   ((uint32_t)0x0000000F)

#define IS_RST_CLK_HSIclkDivValue(SRC)        ((SRC >= 0) && (SRC < 16))

/** @} */ /* End of group RST_CLK_HSI_C1_SEL */

/** @defgroup RST_CLK_HSE_C1_SEL HSE clock divider
  * @{
  */

/**
  * @brief RST_CLK HSE clock HSE_C1_SEL divider constants for RTC_CLOCK
  */

#define RST_CLK_HSEclkDIV1                    ((uint32_t)0x00000000)
#define RST_CLK_HSEclkDIV2                    ((uint32_t)0x00000001)
#define RST_CLK_HSEclkDIV3                    ((uint32_t)0x00000002)
#define RST_CLK_HSEclkDIV4                    ((uint32_t)0x00000003)
#define RST_CLK_HSEclkDIV5                    ((uint32_t)0x00000004)
#define RST_CLK_HSEclkDIV6                    ((uint32_t)0x00000005)
#define RST_CLK_HSEclkDIV7                    ((uint32_t)0x00000006)
#define RST_CLK_HSEclkDIV8                    ((uint32_t)0x00000007)
#define RST_CLK_HSEclkDIV9                    ((uint32_t)0x00000008)
#define RST_CLK_HSEclkDIV10                   ((uint32_t)0x00000009)
#define RST_CLK_HSEclkDIV11                   ((uint32_t)0x0000000A)
#define RST_CLK_HSEclkDIV12                   ((uint32_t)0x0000000B)
#define RST_CLK_HSEclkDIV13                   ((uint32_t)0x0000000C)
#define RST_CLK_HSEclkDIV14                   ((uint32_t)0x0000000D)
#define RST_CLK_HSEclkDIV15                   ((uint32_t)0x0000000E)
#define RST_CLK_HSEclkDIV16                   ((uint32_t)0x0000000F)

#define IS_RST_CLK_HSEclkDivValue(SRC)        ((SRC >= 0) && (SRC < 16))

/** @} */ /* End of group RST_CLK_HSE_C1_SEL */

/** @} */ /* End of group RST_CLK_Exported_Constants */

/** @defgroup RST_CLK_Exported_Macros RST_CLK Exported Macros
  * @{
  */

/** @} */ /* End of group RST_CLK_Exported_Macros */

/** @defgroup RST_CLK_Exported_Functions RST_CLK Exported Functions
  * @{
  */

void RST_CLK_DeInit(void);
void RST_CLK_WarmDeInit(void);

void RST_CLK_HSEconfig(uint32_t RST_CLK_HSE);
ErrorStatus RST_CLK_HSEstatus(void);

void RST_CLK_LSEconfig(uint32_t RST_CLK_LSE);
ErrorStatus RST_CLK_LSEstatus(void);

void RST_CLK_HSIcmd(FunctionalState NewState);
void RST_CLK_HSIadjust(uint8_t HSItrimValue);
ErrorStatus RST_CLK_HSIstatus(void);

void RST_CLK_LSIcmd(FunctionalState NewState);
ErrorStatus RST_CLK_LSIstatus(void);

void RST_CLK_CPU_PLLconfig(uint32_t RST_CLK_CPU_PLLsource, uint32_t RST_CLK_CPU_PLLmul);
void RST_CLK_CPU_PLLuse(FunctionalState UsePLL);
void RST_CLK_CPU_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_CPU_PLLstatus(void);

void RST_CLK_CPUclkPrescaler(uint32_t CPUclkDivValue);
void RST_CLK_CPUclkSelection(uint32_t CPU_CLK);

void RST_CLK_USB_PLLconfig(uint32_t RST_CLK_USB_PLLsource, uint32_t RST_CLK_USB_PLLmul);
void RST_CLK_USB_PLLuse(FunctionalState UsePLL);
void RST_CLK_USB_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_USB_PLLstatus(void);

void RST_CLK_USBclkPrescaler(FunctionalState NewState);
void RST_CLK_USBclkEnable(FunctionalState NewState);

void RST_CLK_ADCclkSelection(uint32_t ADC_CLK);
void RST_CLK_ADCclkPrescaler(uint32_t ADCclkDivValue);
void RST_CLK_ADCclkEnable(FunctionalState NewState);

void RST_CLK_HSIclkPrescaler(uint32_t HSIclkDivValue);
void RST_CLK_RTC_HSIclkEnable(FunctionalState NewState);

void RST_CLK_HSEclkPrescaler(uint32_t HSEclkDivValue);
void RST_CLK_RTC_HSEclkEnable(FunctionalState NewState);

void RST_CLK_PCLKcmd(uint32_t RST_CLK_PCLK, FunctionalState NewState);

void RST_CLK_GetClocksFreq(RST_CLK_FreqTypeDef* RST_CLK_Clocks);

FlagStatus RST_CLK_GetFlagStatus(uint32_t RST_CLK_FLAG);

/** @} */ /* End of group RST_CLK_Exported_Functions */

/** @} */ /* End of group RST_CLK */

/** @} */ /* End of group __1986BE9x_StdPeriph_Driver */

#endif /* __1986BE9x_RST_CLK_H */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE 1986BE9x_rst_clk.h */

