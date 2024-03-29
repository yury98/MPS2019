/**
  ******************************************************************************
  * @file    1986BE9x_adc.c
  * @author  Phyton Application Team
  * @version V1.0.0
  * @date    22/07/2011
  * @brief   This file provides all the ADC firmware functions.
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
  * <h2><center>&copy; COPYRIGHT 2011 Phyton</center></h2>
  ******************************************************************************
  * FILE 1986BE9x_adc.c
  */

/* Includes ------------------------------------------------------------------*/
#include "1986BE9x_adc.h"
#include "1986BE9x_config.h"

#define ASSERT_INFO_FILE_ID FILEID__1986BE9X_ADC_C

/** @addtogroup __1986BE9x_StdPeriph_Driver 1986BE9x Standard Peripherial Driver
  * @{
  */

/** @defgroup ADC ADC
  * @{
  */

/** @defgroup ADC_Private_Functions ADC Private Functions
  * @{
  */

/**
  * @brief  Deinitializes the ADC peripheral registers to their default reset values.
  * @param  None.
  * @retval None.
  */
void ADC_DeInit(void)
{
  ADC->ADC1_CFG = 0;
  ADC->ADC2_CFG = 0;
  ADC->ADC1_H_LEVEL = 0;
  ADC->ADC2_H_LEVEL = 0;
  ADC->ADC1_L_LEVEL = 0;
  ADC->ADC2_L_LEVEL = 0;
  ADC->ADC1_RESULT;
  ADC->ADC2_RESULT;
  ADC->ADC1_STATUS = 0;
  ADC->ADC2_STATUS = 0;
  ADC->ADC1_CHSEL = 0;
  ADC->ADC2_CHSEL = 0;
}

/**
  * @brief  Initializes the ADC peripheral according to
  *         the specified parameters in the ADC_InitStruct.
  * @param  ADC_InitStruct: pointer to a ADC_InitTypeDef structure
  *         that contains the configuration information for the specified ADC
  *         peripheral.
  * @retval None
  */
void ADC_Init(const ADC_InitTypeDef* ADC_InitStruct)
{
  uint32_t tmpreg_CFG;
  uint32_t tmpreg_MSK;

  /* Check the parameters */
  assert_param(IS_ADC_SYNC_MODE(ADC_InitStruct->ADC_SynchronousMode));
  assert_param(IS_ADC_START_DELAY_VALUE(ADC_InitStruct->ADC_StartDelay));
  assert_param(IS_ADC_TEMP_SENSOR_CONFIG(ADC_InitStruct->ADC_TempSensor));
  assert_param(IS_ADC_TEMP_SENSOR_AMP_CONFIG(ADC_InitStruct->ADC_TempSensorAmplifier));
  assert_param(IS_ADC_TEMP_SENSOR_CONVERSION_CONFIG(ADC_InitStruct->ADC_TempSensorConversion));
  assert_param(IS_ADC_VREF_CONVERSION_CONFIG(ADC_InitStruct->ADC_IntVRefConversion));
  assert_param(IS_ADC_VREF_TRIMMING_VALUE(ADC_InitStruct->ADC_IntVRefTrimming));

  tmpreg_CFG = ADC_InitStruct->ADC_SynchronousMode
             + (ADC_InitStruct->ADC_StartDelay << ADC1_CFG_Delay_ADC_Pos)
             + ADC_InitStruct->ADC_TempSensor
             + ADC_InitStruct->ADC_TempSensorAmplifier
             + ADC_InitStruct->ADC_TempSensorConversion
             + ADC_InitStruct->ADC_IntVRefConversion
             + (ADC_InitStruct->ADC_IntVRefTrimming << ADC1_CFG_TR_Pos);

  tmpreg_MSK = ADC1_CFG_Cfg_Sync_Conver
             | ADC1_CFG_Delay_ADC_Msk
             | ADC1_CFG_TS_EN
             | ADC1_CFG_TS_BUF_EN
             | ADC1_CFG_SEL_TS
             | ADC1_CFG_SEL_VREF
             | ADC1_CFG_TR_Msk;

  ADC->ADC1_CFG = (ADC->ADC1_CFG & ~tmpreg_MSK) + tmpreg_CFG;
}

/**
  * @brief  Fills each ADC_InitStruct member with its default value.
  * @param  ADC_InitStruct: pointer to a ADC_InitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
  ADC_InitStruct->ADC_SynchronousMode      = ADC_SyncMode_Independent;
  ADC_InitStruct->ADC_StartDelay           = 0;
  ADC_InitStruct->ADC_TempSensor           = ADC_TEMP_SENSOR_Disable;
  ADC_InitStruct->ADC_TempSensorAmplifier  = ADC_TEMP_SENSOR_AMPLIFIER_Disable;
  ADC_InitStruct->ADC_TempSensorConversion = ADC_TEMP_SENSOR_CONVERSION_Disable;
  ADC_InitStruct->ADC_IntVRefConversion    = ADC_VREF_CONVERSION_Disable;
  ADC_InitStruct->ADC_IntVRefTrimming      = 0;
}

/**
  * @brief  Sets the Internal Voltage Reference trimming.
  * @param  Trim: trimming value in range 0..7.
  * @retval None
  */
void ADC_SetTrim(uint32_t Trim)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC_VREF_TRIMMING_VALUE(Trim));

  tmpreg_CFG = ADC->ADC1_CFG & ~ADC1_CFG_TR_Msk;
  ADC->ADC1_CFG = tmpreg_CFG + (Trim << ADC1_CFG_TR_Pos);
}

/**
  * @brief  Initializes the ADC1 peripheral according to
  *         the specified parameters in the ADCx_InitStruct.
  * @param  ADCx_InitStruct: pointer to a ADCx_InitTypeDef structure
  *         that contains the configuration information for the ADC1
  *         peripheral.
  * @retval None
  */
void ADC1_Init(const ADCx_InitTypeDef* ADCx_InitStruct)
{
  uint32_t tmpreg_CFG1;
  uint32_t tmpreg_CFG2;

  /* Check the parameters */
  assert_param(IS_ADC_CLOCK_SOURCE_CONFIG(ADCx_InitStruct->ADC_ClockSource));
  assert_param(IS_ADC_SAMPLING_MODE_CONFIG(ADCx_InitStruct->ADC_SamplingMode));
  assert_param(IS_ADC_CH_SWITCHING_CONFIG(ADCx_InitStruct->ADC_ChannelSwitching));
  assert_param(IS_ADC1_CH_NUM(ADCx_InitStruct->ADC_ChannelNumber));
  assert_param(IS_ADC1_CH_MASK(ADCx_InitStruct->ADC_Channels));
  assert_param(IS_ADC_LEVEL_CONTROL_CONFIG(ADCx_InitStruct->ADC_LevelControl));
  assert_param(IS_ADC_VALUE(ADCx_InitStruct->ADC_LowLevel));
  assert_param(IS_ADC_VALUE(ADCx_InitStruct->ADC_HighLevel));
  assert_param(IS_ADC_VREF_SOURCE_CONFIG(ADCx_InitStruct->ADC_VRefSource));
  assert_param(IS_ADC_INT_VREF_SOURCE_CONFIG(ADCx_InitStruct->ADC_IntVRefSource));
  assert_param(IS_ADC_CLK_div_VALUE(ADCx_InitStruct->ADC_Prescaler));
  assert_param(IS_ADC_DELAY_GO_VALUE(ADCx_InitStruct->ADC_DelayGo));

  tmpreg_CFG1 = ADC->ADC1_CFG;

  tmpreg_CFG1 &= ~(ADC1_CFG_Cfg_REG_CLKS
                 | ADC1_CFG_Cfg_REG_SAMPLE
                 | ADC1_CFG_Cfg_REG_CHCH
                 | ADC1_CFG_Cfg_REG_CHS_Msk
                 | ADC1_CFG_Cfg_REG_RNGC
                 | ADC1_CFG_Cfg_M_REF
                 | ADC1_CFG_Cfg_REG_DIVCLK_Msk
                 | ADC1_CFG_Delay_Go_Msk);

  tmpreg_CFG1 += ADCx_InitStruct->ADC_ClockSource
               + ADCx_InitStruct->ADC_SamplingMode
               + ADCx_InitStruct->ADC_ChannelSwitching
               + (ADCx_InitStruct->ADC_ChannelNumber << ADC1_CFG_Cfg_REG_CHS_Pos)
               + ADCx_InitStruct->ADC_LevelControl
               + ADCx_InitStruct->ADC_VRefSource
               + ADCx_InitStruct->ADC_Prescaler
               + (ADCx_InitStruct->ADC_DelayGo << ADC1_CFG_Delay_Go_Pos);

  tmpreg_CFG2 = ADC->ADC2_CFG;
  tmpreg_CFG2 &= ~ADC2_CFG_ADC1_OP;
  tmpreg_CFG2 += ADCx_InitStruct->ADC_IntVRefSource << ADC2_CFG_ADC1_OP_Pos;

  ADC->ADC1_CFG = tmpreg_CFG1;
  ADC->ADC2_CFG = tmpreg_CFG2;

  ADC->ADC1_L_LEVEL = ADCx_InitStruct->ADC_LowLevel;
  ADC->ADC1_H_LEVEL = ADCx_InitStruct->ADC_HighLevel;
  ADC->ADC1_CHSEL   = ADCx_InitStruct->ADC_Channels;
}

/**
  * @brief  Initializes the ADC2 peripheral according to
  *         the specified parameters in the ADCx_InitStruct.
  * @param  ADCx_InitStruct: pointer to a ADCx_InitTypeDef structure
  *         that contains the configuration information for the ADC2
  *         peripheral.
  * @retval None
  */
void ADC2_Init(const ADCx_InitTypeDef* ADCx_InitStruct)
{
  uint32_t tmpreg_CFG2;

  /* Check the parameters */
  assert_param(IS_ADC_CLOCK_SOURCE_CONFIG(ADCx_InitStruct->ADC_ClockSource));
  assert_param(IS_ADC_SAMPLING_MODE_CONFIG(ADCx_InitStruct->ADC_SamplingMode));
  assert_param(IS_ADC_CH_SWITCHING_CONFIG(ADCx_InitStruct->ADC_ChannelSwitching));
  assert_param(IS_ADC2_CH_NUM(ADCx_InitStruct->ADC_ChannelNumber));
  assert_param(IS_ADC2_CH_MASK(ADCx_InitStruct->ADC_Channels));
  assert_param(IS_ADC_LEVEL_CONTROL_CONFIG(ADCx_InitStruct->ADC_LevelControl));
  assert_param(IS_ADC_VALUE(ADCx_InitStruct->ADC_LowLevel));
  assert_param(IS_ADC_VALUE(ADCx_InitStruct->ADC_HighLevel));
  assert_param(IS_ADC_VREF_SOURCE_CONFIG(ADCx_InitStruct->ADC_VRefSource));
  assert_param(IS_ADC_INT_VREF_SOURCE_CONFIG(ADCx_InitStruct->ADC_IntVRefSource));
  assert_param(IS_ADC_CLK_div_VALUE(ADCx_InitStruct->ADC_Prescaler));
  assert_param(IS_ADC_DELAY_GO_VALUE(ADCx_InitStruct->ADC_DelayGo));

  tmpreg_CFG2 = ADC->ADC2_CFG;

  tmpreg_CFG2 &= ~(ADC2_CFG_Cfg_REG_CLKS
                 | ADC2_CFG_Cfg_REG_SAMPLE
                 | ADC2_CFG_Cfg_REG_CHCH
                 | ADC2_CFG_Cfg_REG_CHS_Msk
                 | ADC2_CFG_Cfg_REG_RNGC
                 | ADC2_CFG_Cfg_M_REF
                 | ADC2_CFG_ADC2_OP
                 | ADC2_CFG_Cfg_REG_DIVCLK_Msk
                 | ADC2_CFG_Delay_Go_Msk);

  tmpreg_CFG2 += ADCx_InitStruct->ADC_ClockSource
               + ADCx_InitStruct->ADC_SamplingMode
               + ADCx_InitStruct->ADC_ChannelSwitching
               + (ADCx_InitStruct->ADC_ChannelNumber << ADC2_CFG_Cfg_REG_CHS_Pos)
               + ADCx_InitStruct->ADC_LevelControl
               + ADCx_InitStruct->ADC_VRefSource
               + (ADCx_InitStruct->ADC_IntVRefSource << ADC2_CFG_ADC2_OP_Pos)
               + ADCx_InitStruct->ADC_Prescaler
               + (ADCx_InitStruct->ADC_DelayGo << ADC2_CFG_Delay_Go_Pos);

  ADC->ADC2_CFG = tmpreg_CFG2;
  ADC->ADC2_L_LEVEL = ADCx_InitStruct->ADC_LowLevel;
  ADC->ADC2_H_LEVEL = ADCx_InitStruct->ADC_HighLevel;
  ADC->ADC2_CHSEL   = ADCx_InitStruct->ADC_Channels;
}

/**
  * @brief  Fills each ADCx_InitStruct member with its default value.
  * @param  ADCx_InitStruct: pointer to a ADCx_InitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
void ADCx_StructInit(ADCx_InitTypeDef* ADCx_InitStruct)
{
  ADCx_InitStruct->ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
  ADCx_InitStruct->ADC_SamplingMode     = ADC_SAMPLING_MODE_SINGLE_CONV;
  ADCx_InitStruct->ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
  ADCx_InitStruct->ADC_ChannelNumber    = ADC_CH_ADC0;
  ADCx_InitStruct->ADC_Channels         = 0;
  ADCx_InitStruct->ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;
  ADCx_InitStruct->ADC_LowLevel         = 0;
  ADCx_InitStruct->ADC_HighLevel        = 0;
  ADCx_InitStruct->ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;
  ADCx_InitStruct->ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;
  ADCx_InitStruct->ADC_Prescaler        = ADC_CLK_div_None;
  ADCx_InitStruct->ADC_DelayGo          = 0;
}

/**
  * @brief  Enables or disables the ADC1 peripheral.
  * @param  NewState: new state of the ADC1 peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void ADC1_Cmd(FunctionalState NewState)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  tmpreg_CFG = ADC->ADC1_CFG;

  /* Form new value */
  if (NewState != DISABLE)
  {
    /* Enable ADC1 by setting the Cfg_REG_ADON bit in the ADC1_CFG register */
    tmpreg_CFG |= ADC1_CFG_Cfg_REG_ADON;
  }
  else
  {
    /* Disable ADC1 by resetting the Cfg_REG_ADON bit in the ADC1_CFG register */
    tmpreg_CFG &= ~ADC1_CFG_Cfg_REG_ADON;
  }

  /* Configure ADC1_CFG register with new value */
  ADC->ADC1_CFG = tmpreg_CFG;
}

/**
  * @brief  Enables or disables the ADC1 peripheral.
  * @param  NewState: new state of the ADC1 peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void ADC2_Cmd(FunctionalState NewState)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  tmpreg_CFG = ADC->ADC2_CFG;

  /* Form new value */
  if (NewState != DISABLE)
  {
    /* Enable ADC2 by setting the Cfg_REG_ADON bit in the ADC2_CFG register */
    tmpreg_CFG |= ADC2_CFG_Cfg_REG_ADON;
  }
  else
  {
    /* Disable ADC2 by resetting the Cfg_REG_ADON bit in the ADC2_CFG register */
    tmpreg_CFG &= ~ADC2_CFG_Cfg_REG_ADON;
  }

  /* Configure ADC2_CFG register with new value */
  ADC->ADC2_CFG = tmpreg_CFG;
}

/**
  * @brief  Selects the ADC1 Channel number for Single Channel Mode conversion.
  * @param  Channel: specifies the ADC Channel number.
  * @retval None
  */
void ADC1_SetChannel(uint32_t Channel)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC1_CH_NUM(Channel));

  tmpreg_CFG = ADC->ADC1_CFG;
  tmpreg_CFG &= ~ADC1_CFG_Cfg_REG_CHS_Msk;
  tmpreg_CFG += Channel << ADC1_CFG_Cfg_REG_CHS_Pos;
  ADC->ADC1_CFG = tmpreg_CFG;
}

/**
  * @brief  Selects the ADC2 Channel number for Single Channel Mode conversion.
  * @param  Channel: specifies the ADC Channel number.
  * @retval None
  */
void ADC2_SetChannel(uint32_t Channel)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC2_CH_NUM(Channel));

  tmpreg_CFG = ADC->ADC2_CFG;
  tmpreg_CFG &= ~ADC2_CFG_Cfg_REG_CHS_Msk;
  tmpreg_CFG += Channel << ADC2_CFG_Cfg_REG_CHS_Pos;
  ADC->ADC2_CFG = tmpreg_CFG;
}

/**
  * @brief  Selects the ADC1 Channels for conversion with Channels switching.
  * @param  ChannelMask: specifies the ADC Channels Mask.
  * @retval None
  */
void ADC1_SetChannels(uint32_t ChannelMask)
{
  /* Check the parameters */
  assert_param(IS_ADC1_CH_MASK(ChannelMask));

  ADC->ADC1_CHSEL = ChannelMask;
}

/**
  * @brief  Selects the ADC2 Channels for conversion with Channels switching.
  * @param  ChannelMask: specifies the ADC Channels Mask.
  * @retval None
  */
void ADC2_SetChannels(uint32_t ChannelMask)
{
  /* Check the parameters */
  assert_param(IS_ADC2_CH_MASK(ChannelMask));

  ADC->ADC2_CHSEL = ChannelMask;
}

/**
  * @brief  Sets the ADC1 operation mode.
  * @param  SamplingMode: specifies the ADC1 sampling.
  * @param  SwitchingMode: specifies the ADC1 channel switching.
  * @retval None
  */
void ADC1_OperationModeConfig(uint32_t SamplingMode, uint32_t SwitchingMode)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC_SAMPLING_MODE_CONFIG(SamplingMode));
  assert_param(IS_ADC_CH_SWITCHING_CONFIG(SwitchingMode));

  tmpreg_CFG = ADC->ADC1_CFG;
  tmpreg_CFG &= ~(ADC1_CFG_Cfg_REG_SAMPLE | ADC1_CFG_Cfg_REG_CHCH);
  tmpreg_CFG += SamplingMode + SwitchingMode;
  ADC->ADC1_CFG = tmpreg_CFG;
}

/**
  * @brief  Sets the ADC2 operation mode.
  * @param  SamplingMode: specifies the ADC2 sampling.
  * @param  SwitchingMode: specifies the ADC2 channel switching.
  * @retval None
  */
void ADC2_OperationModeConfig(uint32_t SamplingMode, uint32_t SwitchingMode)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC_SAMPLING_MODE_CONFIG(SamplingMode));
  assert_param(IS_ADC_CH_SWITCHING_CONFIG(SwitchingMode));

  tmpreg_CFG = ADC->ADC2_CFG;
  tmpreg_CFG &= ~(ADC2_CFG_Cfg_REG_SAMPLE | ADC2_CFG_Cfg_REG_CHCH);
  tmpreg_CFG += SamplingMode + SwitchingMode;
  ADC->ADC2_CFG = tmpreg_CFG;
}

/**
  * @brief  Sets the ADC1 sampling mode.
  * @param  SamplingMode: specifies the ADC1 sampling.
  * @retval None
  */
void ADC1_SamplingModeConfig(uint32_t SamplingMode)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC_SAMPLING_MODE_CONFIG(SamplingMode));

  tmpreg_CFG = ADC->ADC1_CFG;
  tmpreg_CFG &= ~ADC1_CFG_Cfg_REG_SAMPLE;
  tmpreg_CFG += SamplingMode;
  ADC->ADC1_CFG = tmpreg_CFG;
}

/**
  * @brief  Sets the ADC2 sampling mode.
  * @param  SamplingMode: specifies the ADC2 sampling.
  * @retval None
  */
void ADC2_SamplingModeConfig(uint32_t SamplingMode)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC_SAMPLING_MODE_CONFIG(SamplingMode));

  tmpreg_CFG = ADC->ADC2_CFG;
  tmpreg_CFG &= ~ADC2_CFG_Cfg_REG_SAMPLE;
  tmpreg_CFG += SamplingMode;
  ADC->ADC2_CFG = tmpreg_CFG;
}

/**
  * @brief  Sets the ADC1 channel switching mode.
  * @param  SwitchingMode: specifies the ADC1 channel switching.
  * @retval None
  */
void ADC1_ChannelSwithingConfig(uint32_t SwitchingMode)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC_CH_SWITCHING_CONFIG(SwitchingMode));

  tmpreg_CFG = ADC->ADC1_CFG;
  tmpreg_CFG &= ~ADC1_CFG_Cfg_REG_CHCH;
  tmpreg_CFG += SwitchingMode;
  ADC->ADC1_CFG = tmpreg_CFG;
}

/**
  * @brief  Sets the ADC2 channel switching mode.
  * @param  SwitchingMode: specifies the ADC2 channel switching.
  * @retval None
  */
void ADC2_ChannelSwithingConfig(uint32_t SwitchingMode)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC_CH_SWITCHING_CONFIG(SwitchingMode));

  tmpreg_CFG = ADC->ADC2_CFG;
  tmpreg_CFG &= ~ADC2_CFG_Cfg_REG_CHCH;
  tmpreg_CFG += SwitchingMode;
  ADC->ADC2_CFG = tmpreg_CFG;
}

/**
  * @brief  Configures the ADC1 threshould levels.
  * @param  LowLevel: specifies the ADC1 low level value.
  * @param  HighLevel: specifies the ADC1 high level value.
  * @param  NewState: enables or disables levels control.
  * @retval None
  */
void ADC1_LevelsConfig(uint32_t LowLevel, uint32_t HighLevel, uint32_t NewState)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC_LEVEL_CONTROL_CONFIG(NewState));
  assert_param(IS_ADC_VALUE(LowLevel));
  assert_param(IS_ADC_VALUE(HighLevel));

  tmpreg_CFG = ADC->ADC1_CFG;
  tmpreg_CFG &= ~ADC1_CFG_Cfg_REG_RNGC;
  tmpreg_CFG += NewState;
  ADC->ADC1_CFG = tmpreg_CFG;

  ADC->ADC1_L_LEVEL = LowLevel;
  ADC->ADC1_H_LEVEL = HighLevel;
}

/**
  * @brief  Configures the ADC2 threshould levels.
  * @param  LowLevel: specifies the ADC2 low level value.
  * @param  HighLevel: specifies the ADC2 high level value.
  * @param  NewState: enables or disables levels control.
  * @retval None
  */
void ADC2_LevelsConfig(uint32_t LowLevel, uint32_t HighLevel, uint32_t NewState)
{
  uint32_t tmpreg_CFG;

  /* Check the parameters */
  assert_param(IS_ADC_LEVEL_CONTROL_CONFIG(NewState));
  assert_param(IS_ADC_VALUE(LowLevel));
  assert_param(IS_ADC_VALUE(HighLevel));

  tmpreg_CFG = ADC->ADC2_CFG;
  tmpreg_CFG &= ~ADC2_CFG_Cfg_REG_RNGC;
  tmpreg_CFG += NewState;
  ADC->ADC2_CFG = tmpreg_CFG;

  ADC->ADC2_L_LEVEL = LowLevel;
  ADC->ADC2_H_LEVEL = HighLevel;
}

/**
  * @brief  Sets the ADC1 low level.
  * @param  LowLevel: specifies the ADC1 low level value.
  * @retval None
  */
void ADC1_SetLowLevel(uint32_t LowLevel)
{
  /* Check the parameters */
  assert_param(IS_ADC_VALUE(LowLevel));

  ADC->ADC1_L_LEVEL = LowLevel;
}

/**
  * @brief  Sets the ADC2 low level.
  * @param  LowLevel: specifies the ADC2 low level value.
  * @retval None
  */
void ADC2_SetLowLevel(uint32_t LowLevel)
{
  /* Check the parameters */
  assert_param(IS_ADC_VALUE(LowLevel));

  ADC->ADC2_L_LEVEL = LowLevel;
}

/**
  * @brief  Sets the ADC1 high level.
  * @param  HighLevel: specifies the ADC1 high level value.
  * @retval None
  */
void ADC1_SetHighLevel(uint32_t HighLevel)
{
  /* Check the parameters */
  assert_param(IS_ADC_VALUE(HighLevel));

  ADC->ADC1_H_LEVEL = HighLevel;
}

/**
  * @brief  Sets the ADC2 high level.
  * @param  HighLevel: specifies the ADC2 high level value.
  * @retval None
  */
void ADC2_SetHighLevel(uint32_t HighLevel)
{
  /* Check the parameters */
  assert_param(IS_ADC_VALUE(HighLevel));

  ADC->ADC2_H_LEVEL = HighLevel;
}

/**
  * @brief  Starts the ADC1 conversion.
  * @param  None.
  * @retval None.
  */
void ADC1_Start(void)
{
  ADC->ADC1_CFG |= ADC1_CFG_Cfg_REG_GO;
}

/**
  * @brief  Starts the ADC2 conversion.
  * @param  None.
  * @retval None.
  */
void ADC2_Start(void)
{
  ADC->ADC2_CFG |= ADC2_CFG_Cfg_REG_GO;
}

/**
  * @brief  Returns the ADC1 result.
  * @param  None.
  * @retval ADC1 Result Register value.
  */
uint32_t ADC1_GetResult(void)
{
  return ADC->ADC1_RESULT;
}

/**
  * @brief  Returns the ADC2 result.
  * @param  None.
  * @retval ADC2 Result Register value.
  */
uint32_t ADC2_GetResult(void)
{
  return ADC->ADC2_RESULT;
}

/**
  * @brief  Returns the ADC1, ADC2 Status Registers combined value.
  * @param  None.
  * @retval The ADC1_STATUS, ADC2_STATUS Registers combined value.
  */
uint32_t ADC_GetStatus(void)
{
  return ADC->ADC1_STATUS + (ADC->ADC2_STATUS << 16);
}

/**
  * @brief  Returns the ADC1 Status Register value.
  * @param  None.
  * @retval The ADC1_STATUS Register value.
  */
uint32_t ADC1_GetStatus(void)
{
  return ADC->ADC1_STATUS;
}

/**
  * @brief  Returns the ADC2 Status Register value.
  * @param  None.
  * @retval The ADC2_STATUS Register value.
  */
uint32_t ADC2_GetStatus(void)
{
  return ADC->ADC2_STATUS;
}

/**
  * @brief  Checks whether the specified ADC1, ADC2 Status flag is set or not.
  * @param  Flag: specifies the flag to check.
  *         This parameter can be one of the following values:
  *           @arg ADC1_FLAG_OVERWRITE:         the previous ADC1 measured value was overwritten;
  *           @arg ADC1_FLAG_OUT_OF_RANGE:      the ADC1 measured value is out of range;
  *           @arg ADC1_FLAG_END_OF_CONVERSION: the ADC1 conversion finished;
  *           @arg ADC2_FLAG_OVERWRITE:         the previous ADC2 measured value was overwritten;
  *           @arg ADC2_FLAG_OUT_OF_RANGE:      the ADC2 measured value is out of range;
  *           @arg ADC2_FLAG_END_OF_CONVERSION: the ADC2 conversion finished.
  * @retval Current Status flag state (SET or RESET).
  */
FlagStatus ADC_GetFlagStatus(uint32_t Flag)
{
  FlagStatus bitstatus;

  /* Check the parameters */
  assert_param(IS_ADC_STATUS_FLAG(Flag));

  if ((ADC_GetStatus() & Flag) == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}

/**
  * @brief  Checks whether the specified ADC1 Status flag is set or not.
  * @param  Flag: specifies the flag to check.
  *         This parameter can be one of the following values:
  *           @arg ADCx_FLAG_OVERWRITE:         the previous ADC1 measured value was overwritten;
  *           @arg ADCx_FLAG_OUT_OF_RANGE:      the ADC1 measured value is out of range;
  *           @arg ADCx_FLAG_END_OF_CONVERSION: the ADC1 conversion finished.
  * @retval Current Status flag state (SET or RESET).
  */
FlagStatus ADC1_GetFlagStatus(uint32_t Flag)
{
  FlagStatus bitstatus;

  /* Check the parameters */
  assert_param(IS_ADCx_STATUS_FLAG(Flag));

  if ((ADC->ADC1_STATUS & Flag) == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}

/**
  * @brief  Checks whether the specified ADC2 Status flag is set or not.
  * @param  Flag: specifies the flag to check.
  *         This parameter can be one of the following values:
  *           @arg ADCx_FLAG_OVERWRITE:         the previous ADC2 measured value was overwritten;
  *           @arg ADCx_FLAG_OUT_OF_RANGE:      the ADC2 measured value is out of range;
  *           @arg ADCx_FLAG_END_OF_CONVERSION: the ADC2 conversion finished.
  * @retval Current Status flag state (SET or RESET).
  */
FlagStatus ADC2_GetFlagStatus(uint32_t Flag)
{
  FlagStatus bitstatus;

  /* Check the parameters */
  assert_param(IS_ADCx_STATUS_FLAG(Flag));

  if ((ADC->ADC2_STATUS & Flag) == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}

/**
  * @brief  Clears the ADC1 Overwrite flag.
  * @param  None.
  * @retval None
  */
void ADC1_ClearOverwriteFlag(void)
{
  ADC->ADC1_STATUS &= ~ADCx_FLAG_OVERWRITE;
}

/**
  * @brief  Clears the ADC2 Overwrite flag.
  * @param  None.
  * @retval None
  */
void ADC2_ClearOverwriteFlag(void)
{
  ADC->ADC2_STATUS &= ~ADCx_FLAG_OVERWRITE;
}

/**
  * @brief  Clears the ADC1 AWOIFEN flag.
  * @param  None.
  * @retval None
  */
void ADC1_ClearOutOfRangeFlag(void)
{
  ADC->ADC1_STATUS &= ~ADCx_FLAG_OUT_OF_RANGE;
}

/**
  * @brief  Clears the ADC2 AWOIFEN flag.
  * @param  None.
  * @retval None
  */
void ADC2_ClearOutOfRangeFlag(void)
{
  ADC->ADC2_STATUS &= ~ADCx_FLAG_OUT_OF_RANGE;
}

/**
  * @brief  Enables or disables the ADC1, ADC2 interrupts.
  * @param  ADC_IT: specifies the ADC interrupts sources to be enabled or disabled.
  *         This parameter can be any combination of the following values:
  *           @arg ADC1_IT_OUT_OF_RANGE:        the ADC1 measured value is out of range;
  *           @arg ADC1_IT_END_OF_CONVERSION:   the ADC1 conversion finished;
  *           @arg ADC2_IT_OUT_OF_RANGE:        the ADC2 measured value is out of range;
  *           @arg ADC2_IT_END_OF_CONVERSION:   the ADC2 conversion finished.
  * @param  NewState: new state of the ADC interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void ADC_ITConfig(uint32_t ADC_IT, FunctionalState NewState)
{
  uint32_t tmpreg_ADC1_IE;
  uint32_t tmpreg_ADC2_IE;
  uint32_t tmpreg_ADC_IT;

  /* Check the parameters */
  assert_param(IS_ADC_CONFIG_IT(ADC_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  tmpreg_ADC1_IE = ADC->ADC1_STATUS;
  tmpreg_ADC2_IE = ADC->ADC2_STATUS;
  tmpreg_ADC_IT = ADC_IT << 2;

  /* Form new value */
  if (NewState != DISABLE)
  {
    /* Enable the ADC Interrupt requests by setting bits in the ADCx_STATUS registers */
    tmpreg_ADC1_IE |= tmpreg_ADC_IT & 0xFFFF;
    tmpreg_ADC2_IE |= tmpreg_ADC_IT >> 16;
  }
  else
  {
    /* Disable the ADC Interrupt requests by clearing bits in the ADCx_STATUS registers */
    tmpreg_ADC1_IE &= ~(tmpreg_ADC_IT & 0xFFFF);
    tmpreg_ADC2_IE &= ~(tmpreg_ADC_IT >> 16);
  }

  /* Configure ADCx_STATUS registers with new value */
  ADC->ADC1_STATUS = tmpreg_ADC1_IE;
  ADC->ADC2_STATUS = tmpreg_ADC2_IE;
}

/**
  * @brief  Enables or disables the ADC1 interrupts.
  * @param  ADC_IT: specifies the ADC1 interrupts sources to be enabled or disabled.
  *         This parameter can be any combination of the following values:
  *           @arg ADC1_IT_OUT_OF_RANGE:        the ADC1 measured value is out of range;
  *           @arg ADC1_IT_END_OF_CONVERSION:   the ADC1 conversion finished.
  * @param  NewState: new state of the ADC1 interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void ADC1_ITConfig(uint32_t ADC_IT, FunctionalState NewState)
{
  uint32_t tmpreg_ADC1_IE;

  /* Check the parameters */
  assert_param(IS_ADCx_CONFIG_IT(ADC_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  tmpreg_ADC1_IE = ADC->ADC1_STATUS;

  /* Form new value */
  if (NewState != DISABLE)
  {
    /* Enable the ADC Interrupt requests by setting bits in the ADC1_STATUS register */
    tmpreg_ADC1_IE |= (ADC_IT << 2);
  }
  else
  {
    /* Disable the ADC Interrupt requests by clearing bits in the ADC1_STATUS register */
    tmpreg_ADC1_IE &= ~(ADC_IT << 2);
  }

  /* Configure ADC1_STATUS registers with new value */
  ADC->ADC1_STATUS = tmpreg_ADC1_IE;
}

/**
  * @brief  Enables or disables the ADC2 interrupts.
  * @param  ADC_IT: specifies the ADC2 interrupts sources to be enabled or disabled.
  *         This parameter can be any combination of the following values:
  *           @arg ADC2_IT_OUT_OF_RANGE:        the ADC2 measured value is out of range;
  *           @arg ADC2_IT_END_OF_CONVERSION:   the ADC2 conversion finished.
  * @param  NewState: new state of the ADC2 interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void ADC2_ITConfig(uint32_t ADC_IT, FunctionalState NewState)
{
  uint32_t tmpreg_ADC2_IE;

  /* Check the parameters */
  assert_param(IS_ADCx_CONFIG_IT(ADC_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  tmpreg_ADC2_IE = ADC->ADC2_STATUS;

  /* Form new value */
  if (NewState != DISABLE)
  {
    /* Enable the ADC Interrupt requests by setting bits in the ADC2_STATUS register */
    tmpreg_ADC2_IE |= (ADC_IT << 2);
  }
  else
  {
    /* Disable the ADC Interrupt requests by clearing bits in the ADC2_STATUS register */
    tmpreg_ADC2_IE &= ~(ADC_IT << 2);
  }

  /* Configure ADC2_STATUS registers with new value */
  ADC->ADC2_STATUS = tmpreg_ADC2_IE;
}

/**
  * @brief  Checks whether the ADC1, ADC2 interrupt has occurred or not.
  * @param  ADC_IT: specifies the ADC interrupt source to check.
  *         This parameter can be one of the following values:
  *           @arg ADC1_IT_OUT_OF_RANGE:        the ADC1 measured value is out of range;
  *           @arg ADC1_IT_END_OF_CONVERSION:   the ADC1 conversion finished;
  *           @arg ADC2_IT_OUT_OF_RANGE:        the ADC2 measured value is out of range;
  *           @arg ADC2_IT_END_OF_CONVERSION:   the ADC2 conversion finished.
  * @retval The new state of the ADC_IT (SET or RESET).
  */
ITStatus ADC_GetITStatus(uint32_t ADC_IT)
{
  ITStatus bitstatus;
  uint32_t tmpreg;

  /* Check the parameters */
  assert_param(IS_ADC_CONFIG_IT(ADC_IT));

  tmpreg = ADC_GetStatus();
  tmpreg &= (tmpreg >> 2) & ADC_IT;

  if (tmpreg == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}

/**
  * @brief  Checks whether the ADC1 interrupt has occurred or not.
  * @param  ADC_IT: specifies the ADC interrupt source to check.
  *         This parameter can be one of the following values:
  *           @arg ADC1_IT_OUT_OF_RANGE:        the ADC1 measured value is out of range;
  *           @arg ADC1_IT_END_OF_CONVERSION:   the ADC1 conversion finished.
  * @retval The new state of the ADC_IT (SET or RESET).
  */
ITStatus ADC1_GetITStatus(uint32_t ADC_IT)
{
  ITStatus bitstatus;
  uint32_t tmpreg;

  /* Check the parameters */
  assert_param(IS_ADCx_CONFIG_IT(ADC_IT));

  tmpreg = ADC->ADC1_STATUS;
  tmpreg &= (tmpreg >> 2) & ADC_IT;

  if (tmpreg == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}

/**
  * @brief  Checks whether the ADC2 interrupt has occurred or not.
  * @param  ADC_IT: specifies the ADC interrupt source to check.
  *         This parameter can be one of the following values:
  *           @arg ADC2_IT_OUT_OF_RANGE:        the ADC2 measured value is out of range;
  *           @arg ADC2_IT_END_OF_CONVERSION:   the ADC2 conversion finished.
  * @retval The new state of the ADC_IT (SET or RESET).
  */
ITStatus ADC2_GetITStatus(uint32_t ADC_IT)
{
  ITStatus bitstatus;
  uint32_t tmpreg;

  /* Check the parameters */
  assert_param(IS_ADCx_CONFIG_IT(ADC_IT));

  tmpreg = ADC->ADC2_STATUS;
  tmpreg &= (tmpreg >> 2) & ADC_IT;

  if (tmpreg == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}

/** @} */ /* End of group ADC_Private_Functions */

/** @} */ /* End of group ADC */

/** @} */ /* End of group __1986BE9x_StdPeriph_Driver */

/******************* (C) COPYRIGHT 2011 Phyton *********************************
*
* END OF FILE 1986BE9x_adc.c */

