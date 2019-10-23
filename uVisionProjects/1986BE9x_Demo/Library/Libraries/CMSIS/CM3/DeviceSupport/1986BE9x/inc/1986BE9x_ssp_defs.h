/**
  ******************************************************************************
  * @file    1986BE9x_ssp_defs.h
  * @author  Phyton Application Team
  * @version V1.0.0
  * @date    22/02/2011
  * @brief   This file contains all the Special Function Registers definitions
  *          for the SSP peripheral unit used in the Milandr 1986BE9x
  *          microcontrollers.
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
  * FILE 1986BE9x_ssp_defs.h
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __1986BE9X_SSP_DEFS_H
#define __1986BE9X_SSP_DEFS_H

/** @addtogroup __CMSIS CMSIS
  * @{
  */

/** @addtogroup __1986BE9x_Peripheral_Units 1986BE9x Peripheral Units
  * @{
  */

/** @defgroup Periph_SSP SSP
  * @{
  */

/** @defgroup Periph_SSP_Data_Structures Data Structures
  * @{
  */

/** @defgroup Periph_SSP_TypeDef SSP_TypeDef
  * @{
  */

typedef struct
{
  __IO uint32_t CR0;
  __IO uint32_t CR1;
  __IO uint32_t DR;
  __IO uint32_t SR;
  __IO uint32_t CPSR;
  __IO uint32_t IMSC;
  __IO uint32_t RIS;
  __IO uint32_t MIS;
  __IO uint32_t ICR;
  __IO uint32_t DMACR;
}SSP_TypeDef;

/** @} */ /* End of group Periph_SSP_TypeDef */

/** @} */ /* End of group Periph_SSP_Data_Structures */

/** @defgroup Periph_SSP_Defines Defines
  * @{
  */

/** @defgroup Periph_SSP_SSP_CR0_Bits SSP_CR0
  * @{
  */

#define SSP_CR0_DSS_Pos                         0
#define SSP_CR0_DSS_Msk                         ((uint32_t)0x0000000F)

#define SSP_CR0_FRF_Pos                         4
#define SSP_CR0_FRF_Msk                         ((uint32_t)0x00000030)

#define SSP_CR0_SPO_Pos                         6
#define SSP_CR0_SPO                             ((uint32_t)0x00000040)

#define SSP_CR0_SPH_Pos                         7
#define SSP_CR0_SPH                             ((uint32_t)0x00000080)

#define SSP_CR0_SCR_Pos                         8
#define SSP_CR0_SCR_Msk                         ((uint32_t)0x0000FF00)


/** @} */ /* End of group Periph_SSP_SSP_CR0_Bits */

/** @} */ /* End of group Periph_SSP_Defines */

/** @defgroup Periph_SSP_Defines Defines
  * @{
  */

/** @defgroup Periph_SSP_SSP_CR1_Bits SSP_CR1
  * @{
  */

#define SSP_CR1_LBM_Pos                         0
#define SSP_CR1_LBM                             ((uint32_t)0x00000001)

#define SSP_CR1_SSE_Pos                         1
#define SSP_CR1_SSE                             ((uint32_t)0x00000002)

#define SSP_CR1_MS_Pos                          2
#define SSP_CR1_MS                              ((uint32_t)0x00000004)

#define SSP_CR1_SOD_Pos                         3
#define SSP_CR1_SOD                             ((uint32_t)0x00000008)


/** @} */ /* End of group Periph_SSP_SSP_CR1_Bits */

/** @} */ /* End of group Periph_SSP_Defines */

/** @defgroup Periph_SSP_Defines Defines
  * @{
  */

/** @defgroup Periph_SSP_SSP_SR_Bits SSP_SR
  * @{
  */

#define SSP_SR_TFE_Pos                          0
#define SSP_SR_TFE                              ((uint32_t)0x00000001)

#define SSP_SR_TNF_Pos                          1
#define SSP_SR_TNF                              ((uint32_t)0x00000002)

#define SSP_SR_RNE_Pos                          2
#define SSP_SR_RNE                              ((uint32_t)0x00000004)

#define SSP_SR_RFF_Pos                          3
#define SSP_SR_RFF                              ((uint32_t)0x00000008)

#define SSP_SR_BSY_Pos                          4
#define SSP_SR_BSY                              ((uint32_t)0x00000010)


/** @} */ /* End of group Periph_SSP_SSP_SR_Bits */

/** @} */ /* End of group Periph_SSP_Defines */

/** @defgroup Periph_SSP_Defines Defines
  * @{
  */

/** @defgroup Periph_SSP_SSP_IMSC_Bits SSP_IMSC
  * @{
  */

#define SSP_IMSC_RORIM_Pos                      0
#define SSP_IMSC_RORIM                          ((uint32_t)0x00000001)

#define SSP_IMSC_RTIM_Pos                       1
#define SSP_IMSC_RTIM                           ((uint32_t)0x00000002)

#define SSP_IMSC_RXIM_Pos                       2
#define SSP_IMSC_RXIM                           ((uint32_t)0x00000004)

#define SSP_IMSC_TXIM_Pos                       3
#define SSP_IMSC_TXIM                           ((uint32_t)0x00000008)


/** @} */ /* End of group Periph_SSP_SSP_IMSC_Bits */

/** @} */ /* End of group Periph_SSP_Defines */

/** @defgroup Periph_SSP_Defines Defines
  * @{
  */

/** @defgroup Periph_SSP_SSP_RIS_Bits SSP_RIS
  * @{
  */

#define SSP_RIS_RORRIS_Pos                      0
#define SSP_RIS_RORRIS                          ((uint32_t)0x00000001)

#define SSP_RIS_RTRIS_Pos                       1
#define SSP_RIS_RTRIS                           ((uint32_t)0x00000002)

#define SSP_RIS_RXRIS_Pos                       2
#define SSP_RIS_RXRIS                           ((uint32_t)0x00000004)

#define SSP_RIS_TXRIS_Pos                       3
#define SSP_RIS_TXRIS                           ((uint32_t)0x00000008)


/** @} */ /* End of group Periph_SSP_SSP_RIS_Bits */

/** @} */ /* End of group Periph_SSP_Defines */

/** @defgroup Periph_SSP_Defines Defines
  * @{
  */

/** @defgroup Periph_SSP_SSP_MIS_Bits SSP_MIS
  * @{
  */

#define SSP_MIS_RORMIS_Pos                      0
#define SSP_MIS_RORMIS                          ((uint32_t)0x00000001)

#define SSP_MIS_RTMIS_Pos                       1
#define SSP_MIS_RTMIS                           ((uint32_t)0x00000002)

#define SSP_MIS_RXMIS_Pos                       2
#define SSP_MIS_RXMIS                           ((uint32_t)0x00000004)

#define SSP_MIS_TXMIS_Pos                       3
#define SSP_MIS_TXMIS                           ((uint32_t)0x00000008)


/** @} */ /* End of group Periph_SSP_SSP_MIS_Bits */

/** @} */ /* End of group Periph_SSP_Defines */

/** @defgroup Periph_SSP_Defines Defines
  * @{
  */

/** @defgroup Periph_SSP_SSP_ICR_Bits SSP_ICR
  * @{
  */

#define SSP_ICR_RORIC_Pos                       0
#define SSP_ICR_RORIC                           ((uint32_t)0x00000001)

#define SSP_ICR_RTIC_Pos                        1
#define SSP_ICR_RTIC                            ((uint32_t)0x00000002)


/** @} */ /* End of group Periph_SSP_SSP_ICR_Bits */

/** @} */ /* End of group Periph_SSP_Defines */

/** @defgroup Periph_SSP_Defines Defines
  * @{
  */

/** @defgroup Periph_SSP_SSP_DMACR_Bits SSP_DMACR
  * @{
  */

#define SSP_DMACR_RXDMAE_Pos                    0
#define SSP_DMACR_RXDMAE                        ((uint32_t)0x00000001)

#define SSP_DMACR_TXDMAE_Pos                    1
#define SSP_DMACR_TXDMAE                        ((uint32_t)0x00000002)


/** @} */ /* End of group Periph_SSP_SSP_DMACR_Bits */

/** @} */ /* End of group Periph_SSP_Defines */

/** @} */ /* End of group Periph_SSP */

/** @} */ /* End of group __1986BE9x_Peripheral_Units */

/** @} */ /* End of group __CMSIS */

#endif /* __1986BE9X_SSP_DEFS_H */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE 1986BE9x_ssp_defs.h */
