/**
  ******************************************************************************
  * @file    1986BE9x_ext_bus.c
  * @author  Phyton Application Team
  * @version V1.0.0
  * @date    22/02/2011
  * @brief   This file provides all the EXT_BUS firmware functions.
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
  * FILE 1986BE9x_ext_bus.c
  */

/* Includes ------------------------------------------------------------------*/
#include "1986BE9x_ext_bus.h"
#include "1986BE9x_config.h"

#define ASSERT_INFO_FILE_ID FILEID__1986BE9X_EXT_BUS_C

/** @addtogroup __1986BE9x_StdPeriph_Driver 1986BE9x Standard Peripherial Driver
  * @{
  */

/** @defgroup EXT_BUS EXT_BUS
  * @{
  */

/** @defgroup EXT_BUS_Private_Defines EXT_BUS Private Defines
  * @{
  */

#define WAIT_STATES_COEF        1000000
#define WAIT_STATES_MIN         3
#define WAIT_STATES_MAX         18

#define NAND_SYCLES_COEF        1000000
#define NAND_SYCLES_MAX         15

/** @} */ /* End of group EXT_BUS_Private_Defines */

/** @defgroup EXT_BUS_Private_Functions EXT_BUS Private Functions
  * @{
  */

/**
  * @brief  Resets the EXT_BUS peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
void EXT_BUS_DeInit(void)
{
  EXT_BUS_TypeDef *EXT_BUSx;

  EXT_BUSx = EXT_BUS;

  EXT_BUSx->CONTROL = 0;
  EXT_BUSx->NAND_CYCLES = 0;
}

/**
  * @brief  Initializes the EXT_BUS peripheral according to the specified
  *         parameters in the EXT_BUS_InitStruct.
  * @param  EXT_BUS_InitStruct: pointer to a EXT_BUS_InitTypeDef structure that
  *         contains the configuration information for the specified EXT_BUS peripheral.
  * @retval None
  */
void EXT_BUS_Init(const EXT_BUS_InitTypeDef* EXT_BUS_InitStruct)
{
  EXT_BUS_TypeDef *EXT_BUSx;
  uint32_t tmpreg_CONTROL;
  uint32_t tmpreg_CYCLES;

  /* Check the parameters */
  assert_param(IS_EXT_BUS_MODE(EXT_BUS_InitStruct->EXT_BUS_Mode));
  assert_param(IS_EXT_BUS_CPOL(EXT_BUS_InitStruct->EXT_BUS_Cpol));
  assert_param(IS_EXT_BUS_WAIT_STATE(EXT_BUS_InitStruct->EXT_BUS_WaitState));
  assert_param(IS_EXT_BUS_NAND_CYCLES(EXT_BUS_InitStruct->EXT_BUS_NandTrc));
  assert_param(IS_EXT_BUS_NAND_CYCLES(EXT_BUS_InitStruct->EXT_BUS_NandTwc));
  assert_param(IS_EXT_BUS_NAND_CYCLES(EXT_BUS_InitStruct->EXT_BUS_NandTrea));
  assert_param(IS_EXT_BUS_NAND_CYCLES(EXT_BUS_InitStruct->EXT_BUS_NandTwp));
  assert_param(IS_EXT_BUS_NAND_CYCLES(EXT_BUS_InitStruct->EXT_BUS_NandTwhr));
  assert_param(IS_EXT_BUS_NAND_CYCLES(EXT_BUS_InitStruct->EXT_BUS_NandTalea));
  assert_param(IS_EXT_BUS_NAND_CYCLES(EXT_BUS_InitStruct->EXT_BUS_NandTrr));

  /* Form new value for the EXT_BUS_CONTROL register */
  tmpreg_CONTROL = (EXT_BUS_InitStruct->EXT_BUS_Mode)
                 | (EXT_BUS_InitStruct->EXT_BUS_Cpol)
                 | (EXT_BUS_InitStruct->EXT_BUS_WaitState << EXT_BUS_CONTROL_WAIT_STATE_Pos);

  /* Form new value for the EXT_BUS_NAND_CYCLES register */
  tmpreg_CYCLES  = (EXT_BUS_InitStruct->EXT_BUS_NandTrc   << EXT_BUS_NAND_CYCLES_TRC_Pos)
                 | (EXT_BUS_InitStruct->EXT_BUS_NandTwc   << EXT_BUS_NAND_CYCLES_TWC_Pos)
                 | (EXT_BUS_InitStruct->EXT_BUS_NandTrea  << EXT_BUS_NAND_CYCLES_TREA_Pos)
                 | (EXT_BUS_InitStruct->EXT_BUS_NandTwp   << EXT_BUS_NAND_CYCLES_TWP_Pos)
                 | (EXT_BUS_InitStruct->EXT_BUS_NandTwhr  << EXT_BUS_NAND_CYCLES_TWHR_Pos)
                 | (EXT_BUS_InitStruct->EXT_BUS_NandTalea << EXT_BUS_NAND_CYCLES_TALEA_Pos)
                 | (EXT_BUS_InitStruct->EXT_BUS_NandTrr   << EXT_BUS_NAND_CYCLES_TRR_Pos);

  EXT_BUSx = EXT_BUS;

  /* Configure EXT_BUS registers with new values */
  EXT_BUSx->NAND_CYCLES = tmpreg_CYCLES;
  EXT_BUSx->CONTROL = tmpreg_CONTROL;
}

/**
  * @brief  Fills each EXT_BUS_InitStruct member with its default value.
  * @param  EXT_BUS_InitStruct: pointer to a EXT_BUS_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void EXT_BUS_StructInit(EXT_BUS_InitTypeDef* EXT_BUS_InitStruct)
{
  /* Reset EXT_BUS initialization structure parameters values */
  EXT_BUS_InitStruct->EXT_BUS_Mode      = EXT_BUS_MODE_OFF;
  EXT_BUS_InitStruct->EXT_BUS_Cpol      = EXT_BUS_CPOL_POSITIVE;
  EXT_BUS_InitStruct->EXT_BUS_WaitState = EXT_BUS_WAIT_STATE_3HCLK;
  EXT_BUS_InitStruct->EXT_BUS_NandTrc   = EXT_BUS_NAND_CYCLES_0HCLK;
  EXT_BUS_InitStruct->EXT_BUS_NandTwc   = EXT_BUS_NAND_CYCLES_0HCLK;
  EXT_BUS_InitStruct->EXT_BUS_NandTrea  = EXT_BUS_NAND_CYCLES_0HCLK;
  EXT_BUS_InitStruct->EXT_BUS_NandTwp   = EXT_BUS_NAND_CYCLES_0HCLK;
  EXT_BUS_InitStruct->EXT_BUS_NandTwhr  = EXT_BUS_NAND_CYCLES_0HCLK;
  EXT_BUS_InitStruct->EXT_BUS_NandTalea = EXT_BUS_NAND_CYCLES_0HCLK;
  EXT_BUS_InitStruct->EXT_BUS_NandTrr   = EXT_BUS_NAND_CYCLES_0HCLK;
}

/**
  * @brief  Calculates the Wait States number for selected HCLK frequency and
  *         time interval.
  * @param  HCLK_Frequency_KHz: specifies the HCLK frequency.
  * @param  Time_ns: specifies the time interval.
  * @retval The Wait States number in range 0..15 or 0xFFFFFFFF if result is
  *         out of range 0..15.
  */
uint32_t EXT_BUS_CalcWaitStates(uint32_t HCLK_Frequency_KHz, uint32_t Time_ns)
{
  uint32_t Cycles;

  if ( HCLK_Frequency_KHz == 0 )
  {
    Cycles = 0;
  }
  else if ( Time_ns > WAIT_STATES_MAX * WAIT_STATES_COEF / HCLK_Frequency_KHz )
  {
    Cycles = 0xFFFFFFFF;
  }
  else
  {
    Cycles = (HCLK_Frequency_KHz * Time_ns + WAIT_STATES_COEF - 1) / WAIT_STATES_COEF;
    if ( Cycles > WAIT_STATES_MAX)
    {
      Cycles = 0xFFFFFFFF;
    }
    else if ( Cycles >= WAIT_STATES_MIN)
    {
      Cycles -= WAIT_STATES_MIN;
    }
    else
    {
      Cycles = 0;
    }
  }

  return Cycles;
}

/**
  * @brief  Calculates the NAND Sycles number for selected HCLK frequency and
  *         time interval.
  * @param  HCLK_Frequency_KHz: specifies the HCLK frequency.
  * @param  Time_ns: specifies the time interval.
  * @retval The NAND Sycles number in range 0..15 or 0xFFFFFFFF if result is
  *         out of range 0..15.
  */
uint32_t EXT_BUS_CalcNandCycles(uint32_t HCLK_Frequency_KHz, uint32_t Time_ns)
{
  uint32_t Cycles;

  if ( HCLK_Frequency_KHz == 0 )
  {
    Cycles = 0;
  }
  else if ( Time_ns > NAND_SYCLES_MAX * NAND_SYCLES_COEF / HCLK_Frequency_KHz )
  {
    Cycles = 0xFFFFFFFF;
  }
  else
  {
    Cycles = (HCLK_Frequency_KHz * Time_ns + NAND_SYCLES_COEF - 1) / NAND_SYCLES_COEF;
    if ( Cycles > NAND_SYCLES_MAX)
    {
      Cycles = 0xFFFFFFFF;
    }
  }

  return Cycles;
}

/**
  * @brief  Returns the BUSY status of the NAND Flash.
  * @param  None.
  * @retval The NAND Flash BUSY status (NandFlashReady or NandFlashBusy).
  */
FlagStatus EXT_BUS_GetBusyStatus(void)
{
  EXT_BUS_TypeDef *EXT_BUSx;
  FlagStatus tmpreg_BUSY_STS;

  EXT_BUSx = EXT_BUS;

  if ((EXT_BUSx->CONTROL & EXT_BUS_CONTROL_BUSY) == 0)
  {
    tmpreg_BUSY_STS = RESET;
  }
  else
  {
    tmpreg_BUSY_STS = SET;
  }

  return tmpreg_BUSY_STS;
}

/** @} */ /* End of group EXT_BUS_Private_Functions */

/** @} */ /* End of group EXT_BUS */

/** @} */ /* End of group __1986BE9x_StdPeriph_Driver */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE 1986BE9x_ext_bus.c */

