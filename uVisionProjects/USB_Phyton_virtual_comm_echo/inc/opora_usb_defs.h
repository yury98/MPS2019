/**
  ******************************************************************************
  * @file    opora_usb_defs.h
  * @author  Phyton Application Team
  * @version V1.0.0
  * @date
  * @brief   This file contains all the Special Function Registers definitions
  *          for the USB peripheral unit used in the Milandr OPORA
  *          microcontrollers.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 Phyton</center></h2>
  ******************************************************************************
  * FILE opora_usb_defs.h
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OPORA_USB_DEFS_H
#define __OPORA_USB_DEFS_H

/** @addtogroup __CMSIS CMSIS
  * @{
  */

/** @addtogroup __OPORA_Peripheral_Units OPORA Peripheral Units
  * @{
  */

/** @defgroup Periph_USB USB
  * @{
  */

/**
  * @brief USB_SEP_TypeDef ??? structure
  */

typedef struct {
  __IO uint32_t CTRL;
  __IO uint32_t STS;
  __IO uint32_t TS;
  __IO uint32_t NTS;
} _USB_SEP_TypeDef;

/**
  * @brief USB_SEP_TypeDef0 ??? structure
  */

typedef struct {
  __IO uint32_t RXFD;
       uint32_t RESERVED0;
  __IO uint32_t RXFDC_L;
  __IO uint32_t RXFDC_H;
  __IO uint32_t RXFC;
       uint32_t RESERVED1[11];
  __IO uint32_t TXFD;
       uint32_t RESERVED2[3];
  __IO uint32_t TXFDC;
  __IO uint32_t RESERVED6[11];
} _USB_SEP_TypeDef0;

/** @defgroup Periph_USB_Data_Structures Data Structures
  * @{
  */

/** @defgroup Periph_USB_TypeDef USB_TypeDef
  * @{
  */

typedef struct {
  __IO uint32_t HTXC;
  __IO uint32_t HTXT;
  __IO uint32_t HTXLC;
  __IO uint32_t HTXSE;
  __IO uint32_t HTXA;
  __IO uint32_t HTXE;
  __IO uint32_t HFN_L;
  __IO uint32_t HFN_H;
  __IO uint32_t HIS;
  __IO uint32_t HIM;
  __IO uint32_t HRXS;
  __IO uint32_t HRXP;
  __IO uint32_t HRXA;
  __IO uint32_t HRXE;
  __IO uint32_t HRXCS;
  __IO uint32_t HSTM;
       uint32_t RESERVED0[16];
  __IO uint32_t HRXFD;
       uint32_t RESERVED1;
  __IO uint32_t HRXFDC_L;
  __IO uint32_t HRXFDC_H;
  __IO uint32_t HRXFC;
       uint32_t RESERVED2[11];
  __IO uint32_t HTXFD;
       uint32_t RESERVED3[3];
  __IO uint32_t HTXFDC;
       uint32_t RESERVED4[11];
  _USB_SEP_TypeDef USB_SEP[4];
  __IO uint32_t SC;
  __IO uint32_t SLS;
  __IO uint32_t SIS;
  __IO uint32_t SIM;
  __IO uint32_t SA;
  __IO uint32_t SFN_L;
  __IO uint32_t SFN_H;
       uint32_t RESERVED5[9];
  _USB_SEP_TypeDef0 USB_SEP_FIFO[4];
  __IO uint32_t HSCR;
  __IO uint32_t HSVR;
} USB_TypeDef;

/** @} */ /* End of group Periph_USB_TypeDef */

/** @} */ /* End of group Periph_USB_Data_Structures */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_HTXC_Bits USB_HTXC
  * @{
  */

#define USB_HTXC_TREQ_OFFS                      0
#define USB_HTXC_TREQ                           ((uint32_t)0x00000001)

#define USB_HTXC_SOFS_OFFS                      1
#define USB_HTXC_SOFS                           ((uint32_t)0x00000002)

#define USB_HTXC_PREEN_OFFS                     2
#define USB_HTXC_PREEN                          ((uint32_t)0x00000004)

#define USB_HTXC_ISOEN_OFFS                     3
#define USB_HTXC_ISOEN                          ((uint32_t)0x00000008)


/** @} */ /* End of group Periph_USB_USB_HTXC_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_HTXLC_Bits USB_HTXLC
  * @{
  */

#define USB_HTXLC_OFFS                          0
#define USB_HTXLC_MASK                          ((uint32_t)0x00000003)

#define USB_HTXLC_DC_OFFS                       2
#define USB_HTXLC_DC                            ((uint32_t)0x00000004)

#define USB_HTXLC_FSPL_OFFS                     3
#define USB_HTXLC_FSPL                          ((uint32_t)0x00000008)

#define USB_HTXLC_FSLR_OFFS                     4
#define USB_HTXLC_FSLR                          ((uint32_t)0x00000010)


/** @} */ /* End of group Periph_USB_USB_HTXLC_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_HIS_Bits USB_HIS
  * @{
  */

#define USB_HIS_TDONE_OFFS                      0
#define USB_HIS_TDONE                           ((uint32_t)0x00000001)

#define USB_HIS_RESUME_OFFS                     1
#define USB_HIS_RESUME                          ((uint32_t)0x00000002)

#define USB_HIS_CONEV_OFFS                      2
#define USB_HIS_CONEV                           ((uint32_t)0x00000004)

#define USB_HIS_SOFS_OFFS                       3
#define USB_HIS_SOFS                            ((uint32_t)0x00000008)


/** @} */ /* End of group Periph_USB_USB_HIS_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_HIM_Bits USB_HIM
  * @{
  */

#define USB_HIM_TDONEIE_OFFS                    0
#define USB_HIM_TDONEIE                         ((uint32_t)0x00000001)

#define USB_HIM_RESUMEIE_OFFS                   1
#define USB_HIM_RESUMEIE                        ((uint32_t)0x00000002)

#define USB_HIM_CONEVIE_OFFS                    2
#define USB_HIM_CONEVIE                         ((uint32_t)0x00000004)

#define USB_HIM_SOFIE_OFFS                      3
#define USB_HIM_SOFIE                           ((uint32_t)0x00000008)


/** @} */ /* End of group Periph_USB_USB_HIM_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_HRXS_Bits USB_HRXS
  * @{
  */

#define USB_HRXS_CRCERR_OFFS                    0
#define USB_HRXS_CRCERR                         ((uint32_t)0x00000001)

#define USB_HRXS_BSERR_OFFS                     1
#define USB_HRXS_BSERR                          ((uint32_t)0x00000002)

#define USB_HRXS_RXOF_OFFS                      2
#define USB_HRXS_RXOF                           ((uint32_t)0x00000004)

#define USB_HRXS_RXTO_OFFS                      3
#define USB_HRXS_RXTO                           ((uint32_t)0x00000008)

#define USB_HRXS_NAKRXED_OFFS                   4
#define USB_HRXS_NAKRXED                        ((uint32_t)0x00000010)

#define USB_HRXS_STALLRXED_OFFS                 5
#define USB_HRXS_STALLRXED                      ((uint32_t)0x00000020)

#define USB_HRXS_ACKRXED_OFFS                   6
#define USB_HRXS_ACKRXED                        ((uint32_t)0x00000040)

#define USB_HRXS_DATASEQ_OFFS                   7
#define USB_HRXS_DATASEQ                        ((uint32_t)0x00000080)


/** @} */ /* End of group Periph_USB_USB_HRXS_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_SEP_CTRL_Bits USB_SEP_CTRL
  * @{
  */

#define USB_SEP_CTRL_EPEN_OFFS                  0
#define USB_SEP_CTRL_EPEN                       ((uint32_t)0x00000001)

#define USB_SEP_CTRL_EPRDY_OFFS                 1
#define USB_SEP_CTRL_EPRDY                      ((uint32_t)0x00000002)

#define USB_SEP_CTRL_EPDATASEQ_OFFS             2
#define USB_SEP_CTRL_EPDATASEQ                  ((uint32_t)0x00000004)

#define USB_SEP_CTRL_EPSSTALL_OFFS              3
#define USB_SEP_CTRL_EPSSTALL                   ((uint32_t)0x00000008)

#define USB_SEP_CTRL_EPISOEN_OFFS               4
#define USB_SEP_CTRL_EPISOEN                    ((uint32_t)0x00000010)


/** @} */ /* End of group Periph_USB_USB_SEP_CTRL_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_SEP_STS_Bits USB_SEP_STS
  * @{
  */

#define USB_SEP_STS_SCCRCERR_OFFS               0
#define USB_SEP_STS_SCCRCERR                    ((uint32_t)0x00000001)

#define USB_SEP_STS_SCBSERR_OFFS                1
#define USB_SEP_STS_SCBSERR                     ((uint32_t)0x00000002)

#define USB_SEP_STS_SCRXOF_OFFS                 2
#define USB_SEP_STS_SCRXOF                      ((uint32_t)0x00000004)

#define USB_SEP_STS_SCRXTO_OFFS                 3
#define USB_SEP_STS_SCRXTO                      ((uint32_t)0x00000008)

#define USB_SEP_STS_SCNAKSENT_OFFS              4
#define USB_SEP_STS_SCNAKSENT                   ((uint32_t)0x00000010)

#define USB_SEP_STS_SCSTALLSENT_OFFS            5
#define USB_SEP_STS_SCSTALLSENT                 ((uint32_t)0x00000020)

#define USB_SEP_STS_SCACKRXED_OFFS              6
#define USB_SEP_STS_SCACKRXED                   ((uint32_t)0x00000040)

#define USB_SEP_STS_SCDATASEQ_OFFS              7
#define USB_SEP_STS_SCDATASEQ                   ((uint32_t)0x00000080)


/** @} */ /* End of group Periph_USB_USB_SEP_STS_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_SC_Bits USB_SC
  * @{
  */

#define USB_SC_SCGEN_OFFS                       0
#define USB_SC_SCGEN                            ((uint32_t)0x00000001)

#define USB_SC_SCTXLS_OFFS                      1
#define USB_SC_SCTXLS_MASK                      ((uint32_t)0x00000006)

#define USB_SC_SCDC_OFFS                        3
#define USB_SC_SCDC                             ((uint32_t)0x00000008)

#define USB_SC_SCFSP_OFFS                       4
#define USB_SC_SCFSP                            ((uint32_t)0x00000010)

#define USB_SC_SCFSR_OFFS                       5
#define USB_SC_SCFSR                            ((uint32_t)0x00000020)


/** @} */ /* End of group Periph_USB_USB_SC_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_SIS_Bits USB_SIS
  * @{
  */

#define USB_SIS_SCTDONE_OFFS                    0
#define USB_SIS_SCTDONE                         ((uint32_t)0x00000001)

#define USB_SIS_SCRESUME_OFFS                   1
#define USB_SIS_SCRESUME                        ((uint32_t)0x00000002)

#define USB_SIS_SCRESETEV_OFFS                  2
#define USB_SIS_SCRESETEV                       ((uint32_t)0x00000004)

#define USB_SIS_SCSOFREC_OFFS                   3
#define USB_SIS_SCSOFREC                        ((uint32_t)0x00000008)

#define USB_SIS_SCNAKSENT_OFFS                  4
#define USB_SIS_SCNAKSENT                       ((uint32_t)0x00000010)


/** @} */ /* End of group Periph_USB_USB_SIS_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_SIM_Bits USB_SIM
  * @{
  */

#define USB_SIM_SCTDONEIE_OFFS                  0
#define USB_SIM_SCTDONEIE                       ((uint32_t)0x00000001)

#define USB_SIM_SCRESUMEIE_OFFS                 1
#define USB_SIM_SCRESUMEIE                      ((uint32_t)0x00000002)

#define USB_SIM_SCRESETEVIE_OFFS                2
#define USB_SIM_SCRESETEVIE                     ((uint32_t)0x00000004)

#define USB_SIM_SCSOFRECIE_OFFS                 3
#define USB_SIM_SCSOFRECIE                      ((uint32_t)0x00000008)

#define USB_SIM_SCNAKSENTIE_OFFS                4
#define USB_SIM_SCNAKSENTIE                     ((uint32_t)0x00000010)


/** @} */ /* End of group Periph_USB_USB_SIM_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_HSCR_Bits USB_HSCR
  * @{
  */

#define USB_HSCR_HOST_MODE_OFFS                 0
#define USB_HSCR_HOST_MODE                      ((uint32_t)0x00000001)

#define USB_HSCR_RESET_CORE_OFFS                1
#define USB_HSCR_RESET_CORE                     ((uint32_t)0x00000002)

#define USB_HSCR_EN_TX_OFFS                     2
#define USB_HSCR_EN_TX                          ((uint32_t)0x00000004)

#define USB_HSCR_EN_RX_OFFS                     3
#define USB_HSCR_EN_RX                          ((uint32_t)0x00000008)

#define USB_HSCR_DP_PULLUP_OFFS                 4
#define USB_HSCR_DP_PULLUP                      ((uint32_t)0x00000010)

#define USB_HSCR_DP_PULLDOWN_OFFS               5
#define USB_HSCR_DP_PULLDOWN                    ((uint32_t)0x00000020)

#define USB_HSCR_DM_PULLUP_OFFS                 6
#define USB_HSCR_DM_PULLUP                      ((uint32_t)0x00000040)

#define USB_HSCR_DM_PULLDOWN_OFFS               7
#define USB_HSCR_DM_PULLDOWN                    ((uint32_t)0x00000080)


/** @} */ /* End of group Periph_USB_USB_HSCR_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @defgroup Periph_USB_Defines Defines
  * @{
  */

/** @defgroup Periph_USB_USB_HSVR_Bits USB_HSVR
  * @{
  */

#define USB_HSVR_VERSION_OFFS                   0
#define USB_HSVR_VERSION_MASK                   ((uint32_t)0x0000000F)

#define USB_HSVR_REVISION_OFFS                  4
#define USB_HSVR_REVISION_MASK                  ((uint32_t)0x000000F0)


/** @} */ /* End of group Periph_USB_USB_HSVR_Bits */

/** @} */ /* End of group Periph_USB_Defines */

/** @} */ /* End of group Periph_USB */

/** @} */ /* End of group __OPORA_Peripheral_Units */

/** @} */ /* End of group __CMSIS */

#endif /* __OPORA_USB_DEFS_H */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE opora_usb_defs.h */
