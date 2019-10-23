/**
  ******************************************************************************
  * @file    1986BE9x_ext_bus.h
  * @author  Phyton Application Team
  * @version V1.0.0
  * @date    16/02/2011
  * @brief   This file contains all the functions prototypes for the EXT_BUS
  *          firmware library.
  ******************************************************************************
  * <br><br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 Phyton</center></h2>
  ******************************************************************************
  * FILE 1986BE9x_ext_bus.h
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __1986BE9X_EXT_BUS_H
#define __1986BE9X_EXT_BUS_H

/* Includes ------------------------------------------------------------------*/
#include "1986BE9x.h"
#include "1986BE9x_lib.h"

/** @addtogroup __1986BE9x_StdPeriph_Driver 1986BE9x Standard Peripherial Driver
  * @{
  */

/** @addtogroup EXT_BUS
  * @{
  */

/** @defgroup EXT_BUS_Exported_Types EXT_BUS Exported Types
  * @{
  */

/**
  * @brief  EXT_BUS Init structure definition
  */

typedef struct
{
  uint32_t EXT_BUS_Mode;        /*!< Specifies external bus mode.
                                     This parameter can be a value of @ref EXT_BUS_MODE. */
  uint32_t EXT_BUS_Cpol;        /*!< Specifies CLOCK signal polarity.
                                     This parameter can be a value of @ref EXT_BUS_CPOL. */
  uint8_t  EXT_BUS_WaitState;   /*!< Specifies wait states number.
                                     This parameter can be a value of @ref EXT_BUS_WAIT_STATE. */
  uint8_t  EXT_BUS_NandTrc;     /*!< Specifies NAND read cycle time t_rc.
                                     This parameter can be a value of @ref EXT_BUS_NAND_CYCLES. */
  uint8_t  EXT_BUS_NandTwc;     /*!< Specifies NAND write cycle time t_wc.
                                     This parameter can be a value of @ref EXT_BUS_NAND_CYCLES. */
  uint8_t  EXT_BUS_NandTrea;    /*!< Specifies NAND read access time t_rea.
                                     This parameter can be a value of @ref EXT_BUS_NAND_CYCLES. */
  uint8_t  EXT_BUS_NandTwp;     /*!< Specifies NAND write access time t_wp.
                                     This parameter can be a value of @ref EXT_BUS_NAND_CYCLES. */
  uint8_t  EXT_BUS_NandTwhr;    /*!< Specifies NAND status register access time t_whr.
                                     This parameter can be a value of @ref EXT_BUS_NAND_CYCLES. */
  uint8_t  EXT_BUS_NandTalea;   /*!< Specifies NAND ID registers access time t_alea.
                                     This parameter can be a value of @ref EXT_BUS_NAND_CYCLES. */
  uint8_t  EXT_BUS_NandTrr;     /*!< Specifies NAND delay from Busy release to read operation.
                                     This parameter can be a value of @ref EXT_BUS_NAND_CYCLES. */
}EXT_BUS_InitTypeDef;

/** @} */ /* End of group EXT_BUS_Exported_Types */

/** @defgroup EXT_BUS_Exported_Constants EXT_BUS Exported Constants
  * @{
  */

/** @defgroup EXT_BUS_MODE EXT_BUS mode
  * @{
  */

#define EXT_BUS_MODE_OFF            ((uint32_t)0x0)                 /*!< EXT_BUS disabled */
#define EXT_BUS_MODE_ROM            (((uint32_t)0x1) << 0)          /*!< EXT_BUS works with external ROM */
#define EXT_BUS_MODE_RAM            (((uint32_t)0x1) << 1)          /*!< EXT_BUS works with external RAM */
#define EXT_BUS_MODE_NAND           (((uint32_t)0x1) << 2)          /*!< EXT_BUS works with external NAND Flash memory */

#define EXT_BUS_MODE_MSK            (EXT_BUS_MODE_ROM | EXT_BUS_MODE_RAM | EXT_BUS_MODE_NAND)

#define IS_EXT_BUS_MODE(MODE) (((MODE) == EXT_BUS_MODE_OFF ) || \
                               ((MODE) == EXT_BUS_MODE_ROM ) || \
                               ((MODE) == EXT_BUS_MODE_RAM ) || \
                               ((MODE) == EXT_BUS_MODE_NAND))

/** @} */ /* End of group EXT_BUS_MODE */

/** @defgroup EXT_BUS_CPOL EXT_BUS CLOCK polarity
  * @{
  */

#define EXT_BUS_CPOL_POSITIVE       (((uint32_t)0x0) << 3)          /*!< EXT_BUS generates the positive CLOCK signal */
#define EXT_BUS_CPOL_NEGATIVE       (((uint32_t)0x1) << 3)          /*!< EXT_BUS generates the negative CLOCK signal */

#define IS_EXT_BUS_CPOL(CPOL) (((CPOL) == EXT_BUS_CPOL_POSITIVE) || \
                               ((CPOL) == EXT_BUS_CPOL_NEGATIVE))

/** @} */ /* End of group EXT_BUS_CPOL */

/** @defgroup EXT_BUS_WAIT_STATE EXT_BUS Wait States number
  * @{
  */

#define EXT_BUS_WAIT_STATE_3HCLK    ((uint32_t)0x0)                 /*!< Wait State = 3 HCLK clocks */
#define EXT_BUS_WAIT_STATE_4HCLK    ((uint32_t)0x1)                 /*!< Wait State = 4 HCLK clocks */
#define EXT_BUS_WAIT_STATE_5HCLK    ((uint32_t)0x2)                 /*!< Wait State = 5 HCLK clocks */
#define EXT_BUS_WAIT_STATE_6HCLK    ((uint32_t)0x3)                 /*!< Wait State = 6 HCLK clocks */
#define EXT_BUS_WAIT_STATE_7HCLK    ((uint32_t)0x4)                 /*!< Wait State = 7 HCLK clocks */
#define EXT_BUS_WAIT_STATE_8HCLK    ((uint32_t)0x5)                 /*!< Wait State = 8 HCLK clocks */
#define EXT_BUS_WAIT_STATE_9HCLK    ((uint32_t)0x6)                 /*!< Wait State = 9 HCLK clocks */
#define EXT_BUS_WAIT_STATE_10HCLK   ((uint32_t)0x7)                 /*!< Wait State = 10 HCLK clocks */
#define EXT_BUS_WAIT_STATE_11HCLK   ((uint32_t)0x8)                 /*!< Wait State = 11 HCLK clocks */
#define EXT_BUS_WAIT_STATE_12HCLK   ((uint32_t)0x9)                 /*!< Wait State = 12 HCLK clocks */
#define EXT_BUS_WAIT_STATE_13HCLK   ((uint32_t)0xA)                 /*!< Wait State = 13 HCLK clocks */
#define EXT_BUS_WAIT_STATE_14HCLK   ((uint32_t)0xB)                 /*!< Wait State = 14 HCLK clocks */
#define EXT_BUS_WAIT_STATE_15HCLK   ((uint32_t)0xC)                 /*!< Wait State = 15 HCLK clocks */
#define EXT_BUS_WAIT_STATE_16HCLK   ((uint32_t)0xD)                 /*!< Wait State = 16 HCLK clocks */
#define EXT_BUS_WAIT_STATE_17HCLK   ((uint32_t)0xE)                 /*!< Wait State = 17 HCLK clocks */
#define EXT_BUS_WAIT_STATE_18HCLK   ((uint32_t)0xF)                 /*!< Wait State = 18 HCLK clocks */

#define EXT_BUS_WAIT_STATE_MSK      ((uint32_t)0xF)                 /*!< Wait State value mask */

#define IS_EXT_BUS_WAIT_STATE(WAIT_STATE) (((WAIT_STATE) & ~EXT_BUS_WAIT_STATE_MSK) == 0 )

/** @} */ /* End of group EXT_BUS_WAIT_STATE */

/** @defgroup EXT_BUS_NAND_CYCLES EXT_BUS NAND Cycles
  * @{
  */

#define EXT_BUS_NAND_CYCLES_0HCLK   ((uint32_t)0x0)                 /*!< NAND Cycles = 0 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_1HCLK   ((uint32_t)0x1)                 /*!< NAND Cycles = 1 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_2HCLK   ((uint32_t)0x2)                 /*!< NAND Cycles = 2 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_3HCLK   ((uint32_t)0x3)                 /*!< NAND Cycles = 3 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_4HCLK   ((uint32_t)0x4)                 /*!< NAND Cycles = 4 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_5HCLK   ((uint32_t)0x5)                 /*!< NAND Cycles = 5 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_6HCLK   ((uint32_t)0x6)                 /*!< NAND Cycles = 6 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_7HCLK   ((uint32_t)0x7)                 /*!< NAND Cycles = 7 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_8HCLK   ((uint32_t)0x8)                 /*!< NAND Cycles = 8 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_9HCLK   ((uint32_t)0x9)                 /*!< NAND Cycles = 9 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_10HCLK  ((uint32_t)0xA)                 /*!< NAND Cycles = 10 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_11HCLK  ((uint32_t)0xB)                 /*!< NAND Cycles = 11 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_12HCLK  ((uint32_t)0xC)                 /*!< NAND Cycles = 12 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_13HCLK  ((uint32_t)0xD)                 /*!< NAND Cycles = 13 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_14HCLK  ((uint32_t)0xE)                 /*!< NAND Cycles = 14 HCLK clocks */
#define EXT_BUS_NAND_CYCLES_15HCLK  ((uint32_t)0xF)                 /*!< NAND Cycles = 15 HCLK clocks */

#define EXT_BUS_NAND_CYCLES_MSK     ((uint32_t)0xF)                 /*!< NAND Cycles value mask */

#define IS_EXT_BUS_NAND_CYCLES(NAND_CYCLES) (((NAND_CYCLES) & ~EXT_BUS_NAND_CYCLES_MSK) == 0 )

/** @} */ /* End of group EXT_BUS_NAND_CYCLES */

/** @} */ /* End of group EXT_BUS_Exported_Constants */

/** @defgroup EXT_BUS_Exported_Macros EXT_BUS Exported Macros
  * @{
  */

/** @} */ /* End of group EXT_BUS_Exported_Macros */

/** @defgroup EXT_BUS_Exported_Functions EXT_BUS Exported Functions
  * @{
  */

void EXT_BUS_DeInit(void);
void EXT_BUS_Init(const EXT_BUS_InitTypeDef* EXT_BUS_InitStruct);
void EXT_BUS_StructInit(EXT_BUS_InitTypeDef* EXT_BUS_InitStruct);
uint32_t EXT_BUS_CalcWaitStates(uint32_t HCLK_Frequency_KHz, uint32_t Time_ns);
uint32_t EXT_BUS_CalcNandCycles(uint32_t HCLK_Frequency_KHz, uint32_t Time_ns);
FlagStatus EXT_BUS_GetBusyStatus(void);

/** @} */ /* End of group EXT_BUS_Exported_Functions */

/** @} */ /* End of group EXT_BUS */

/** @} */ /* End of group __1986‚…9x_StdPeriph_Driver */

#endif /* __1986BE9X_EXT_BUS_H */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE 1986BE9x_ext_bus.h */

