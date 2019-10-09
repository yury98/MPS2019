///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.40.2.53884/W32 for ARM    22/Sep/2015  17:25:56 /
// Copyright 1999-2012 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  C:\WORK\Milandr.MDR1986BExx.1.4.0\Libraries\MDR32F9Qx_S /
//                    tdPeriph_Driver\src\MDR32F9Qx_dma.c                     /
//    Command line =  C:\WORK\Milandr.MDR1986BExx.1.4.0\Libraries\MDR32F9Qx_S /
//                    tdPeriph_Driver\src\MDR32F9Qx_dma.c --preprocess        /
//                    C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\Debug\List\ -lCN          /
//                    C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\Debug\List\ -lB           /
//                    C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\Debug\List\ -o            /
//                    C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\Debug\Obj\ --debug        /
//                    --endian=little --cpu=Cortex-M3 -e --fpu=None           /
//                    --dlib_config "C:\Program Files (x86)\IAR               /
//                    Systems\Embedded Workbench                              /
//                    6.4\arm\INC\c\DLib_Config_Normal.h" -I                  /
//                    C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\..\..\config\ -I          /
//                    C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\..\..\..\..\Libraries\MDR /
//                    32F9Qx_StdPeriph_Driver\inc\ -I                         /
//                    C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\..\..\..\..\Libraries\MDR /
//                    32F9Qx_StdPeriph_Driver\inc\USB_Library\ -I             /
//                    C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\..\..\..\..\Libraries\CMS /
//                    IS\CM3\CoreSupport\ -I C:\WORK\Milandr.MDR1986BExx.1.4. /
//                    0\Example_Projects\MDR1986VE91_Eval\Demo\IAR_Systems\.. /
//                    \..\..\..\Libraries\CMSIS\CM3\DeviceSupport\MDR32F9Qx\i /
//                    nc\ -I C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projec /
//                    ts\MDR1986VE91_Eval\Demo\IAR_Systems\..\..\..\..\Librar /
//                    ies\CMSIS\CM3\DeviceSupport\MDR32F9Qx\startup\iar\ -I   /
//                    C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\..\..\..\..\Example_Proje /
//                    cts\MDR1986VE91_Eval\inc\ -Ohz                          /
//    List file    =  C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\Demo\IAR_Systems\Debug\List\MDR32F9Qx_dma. /
//                    s                                                       /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME MDR32F9Qx_dma

        #define SHT_PROGBITS 0x1

        PUBLIC DMA_ClearError
        PUBLIC DMA_Cmd
        PUBLIC DMA_ControlTable
        PUBLIC DMA_CtrlDataInit
        PUBLIC DMA_CtrlInit
        PUBLIC DMA_DeInit
        PUBLIC DMA_GetCurrTransferCounter
        PUBLIC DMA_GetFlagStatus
        PUBLIC DMA_Init
        PUBLIC DMA_Request
        PUBLIC DMA_SG_CtrlInit
        PUBLIC DMA_SG_Init
        PUBLIC DMA_SG_StructInit
        PUBLIC DMA_StructInit

// C:\WORK\Milandr.MDR1986BExx.1.4.0\Libraries\MDR32F9Qx_StdPeriph_Driver\src\MDR32F9Qx_dma.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    MDR32F9Qx_dma.c
//    4   * @author  Phyton Application Team
//    5   * @version V1.4.0
//    6   * @date    11/06/2010
//    7   * @brief   This file contains all the DMA firmware functions.
//    8   ******************************************************************************
//    9   * <br><br>
//   10   *
//   11   * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//   12   * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
//   13   * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
//   14   * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
//   15   * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
//   16   * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//   17   *
//   18   * <h2><center>&copy; COPYRIGHT 2010 Phyton</center></h2>
//   19   ******************************************************************************
//   20   * FILE MDR32F9Qx_dma.c
//   21   */
//   22 
//   23 /* Includes ------------------------------------------------------------------*/
//   24 #include "MDR32F9Qx_config.h"
//   25 #include "MDR32F9Qx_dma.h"
//   26 
//   27 #define ASSERT_INFO_FILE_ID FILEID__MDR32F9X_DMA_C
//   28 
//   29 /** @addtogroup __MDR32F9Qx_StdPeriph_Driver MDR32F9Qx Standard Peripherial Driver
//   30   * @{
//   31   */
//   32 
//   33 /** @defgroup DMA DMA
//   34   * @{
//   35   */
//   36 
//   37 /** @defgroup DMA_Private_Constants DMA Private Constants
//   38   * @{
//   39   */
//   40 
//   41 #define DMA_CONTROL_MINUS_1   ((uint32_t)0x3FF0)      /* DMA control "n_minus_1" field mask */
//   42 
//   43 /** @} */ /* End of group DMA_Private_Constants */
//   44 
//   45 /** @defgroup DMA_Private_Variables DMA Private Variables
//   46   * @{
//   47   */
//   48 
//   49 /**
//   50   * @brief  DMA Channel Control Data Table
//   51   */
//   52 
//   53 /* Select data alignment */
//   54 #if (DMA_Channels_Number == 1)
//   55 #define DATA_ALIGN 32
//   56 #elif (DMA_Channels_Number == 2)
//   57 #define DATA_ALIGN 64
//   58 #elif ((DMA_Channels_Number >= 3) && (DMA_Channels_Number <= 4))
//   59 #define DATA_ALIGN 128
//   60 #elif ((DMA_Channels_Number >= 5) && (DMA_Channels_Number <= 8))
//   61 #define DATA_ALIGN 256
//   62 #elif ((DMA_Channels_Number >= 9) && (DMA_Channels_Number <= 16))
//   63 #define DATA_ALIGN 512
//   64 #elif ((DMA_Channels_Number >= 17) && (DMA_Channels_Number <= 32))
//   65 #define DATA_ALIGN 1024
//   66 #endif
//   67 
//   68 #if defined ( __ICCARM__ )
//   69   #pragma data_alignment = DATA_ALIGN
//   70 #if defined (  USE_MDR1986VE3 ) || defined (USE_MDR1986VE1T)
//   71 	DMA_CtrlDataTypeDef DMA_ControlTable[DMA_Channels_Number * (1 + DMA_AlternateData)] IAR_SECTION ("EXECUTABLE_MEMORY_SECTION");
//   72 #else

        SECTION `.bss`:DATA:REORDER:NOROOT(10)
//   73 	DMA_CtrlDataTypeDef DMA_ControlTable[DMA_Channels_Number * (1 + DMA_AlternateData)];
DMA_ControlTable:
        DS8 1024
//   74 #endif // #if defined (  USE_MDR1986VE3 )
//   75 
//   76 #elif defined ( __CMCARM__ )
//   77   #pragma locate DMA_ControlTable 0x20000000 noinit
//   78 	DMA_CtrlDataTypeDef DMA_ControlTable[DMA_Channels_Number * (1 + DMA_AlternateData)];
//   79 #elif defined ( __CC_ARM )
//   80 	#if defined (USE_MDR1986VE3) || defined (USE_MDR1986VE1T)
//   81 		DMA_CtrlDataTypeDef DMA_ControlTable[DMA_Channels_Number * (1 + DMA_AlternateData)] __attribute__((section("EXECUTABLE_MEMORY_SECTION"))) __attribute__ ((aligned (DATA_ALIGN)));
//   82 	#else
//   83 		DMA_CtrlDataTypeDef DMA_ControlTable[DMA_Channels_Number * (1 + DMA_AlternateData)] __attribute__ ((aligned (DATA_ALIGN)));
//   84 	#endif
//   85 #endif
//   86 
//   87 
//   88 
//   89 
//   90 /** @} */ /* End of group DMA_Private_Variables */
//   91 
//   92 /** @defgroup DMA_Private_FunctionPrototypes DMA Private Function Prototypes
//   93   * @{
//   94   */
//   95 
//   96 void DMA_CtrlDataInit(DMA_CtrlDataInitTypeDef *DMA_ctrl_data_ptr, DMA_CtrlDataTypeDef *DMA_ctrl_table_ptr);
//   97 
//   98 /** @} */ /* End of group DMA_Private_FunctionPrototypes */
//   99 
//  100 /** @defgroup DMA_Private_Functions DMA Private Functions
//  101   * @{
//  102   */
//  103 
//  104 /**
//  105   * @brief  Initializes the DMA control data structure according to the specified
//  106   *         parameters.
//  107   * @param  DMA_ctrl_data_ptr: pointer to a DMA_CtrlDataInitTypeDef structure that
//  108   *         contains the control data structure to initialize
//  109   * @param  DMA_ctrl_table_ptr: pointer to a DMA_CtrlDataTypeDef structure that
//  110   *         contains the initial control data configuration, provided by the application.
//  111   * @retval None
//  112   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  113 void DMA_CtrlDataInit(DMA_CtrlDataInitTypeDef *DMA_ctrl_data_ptr, DMA_CtrlDataTypeDef *DMA_ctrl_table_ptr)
//  114 {
DMA_CtrlDataInit:
        PUSH     {R4,LR}
//  115   /* Check DMA configuration parameters */
//  116   assert_param(IS_DMA_CHANNELS(DMA_Channels_Number));
//  117   assert_param(IS_DMA_ALTERNATE_DATA(DMA_AlternateData));
//  118 
//  119   /* Check the parameters */
//  120   assert_param(IS_DMA_SOURCE_INC_STATE(DMA_ctrl_data_ptr->DMA_SourceIncSize));
//  121   assert_param(IS_DMA_DEST_INC_STATE(DMA_ctrl_data_ptr->DMA_DestIncSize));
//  122   assert_param(IS_DMA_MEMORY_DATA_SIZE(DMA_ctrl_data_ptr->DMA_MemoryDataSize));
//  123   assert_param(IS_DMA_MODE(DMA_ctrl_data_ptr->DMA_Mode));
//  124   assert_param(IS_DMA_CYCLE_SIZE(DMA_ctrl_data_ptr->DMA_CycleSize));
//  125   assert_param(IS_DMA_CONTINUOUS_NUMBER(DMA_ctrl_data_ptr->DMA_NumContinuous));
//  126   assert_param(IS_DMA_SOURCE_PROT(DMA_ctrl_data_ptr->DMA_SourceProtCtrl));
//  127   assert_param(IS_DMA_DEST_PROT(DMA_ctrl_data_ptr->DMA_DestProtCtrl));
//  128 
//  129   /* DMA Source Data End Address */
//  130   if (DMA_ctrl_data_ptr->DMA_SourceIncSize == DMA_SourceIncNo)
        LDR      R2,[R0, #+8]
        LDR      R3,[R0, #+0]
        CMP      R2,#+201326592
        BEQ.N    ??DMA_CtrlDataInit_0
//  131   {
//  132     DMA_ctrl_table_ptr->DMA_SourceEndAddr = DMA_ctrl_data_ptr->DMA_SourceBaseAddr;
//  133   }
//  134   else
//  135   {
//  136     DMA_ctrl_table_ptr->DMA_SourceEndAddr = ( DMA_ctrl_data_ptr->DMA_SourceBaseAddr +
//  137         ((DMA_ctrl_data_ptr->DMA_CycleSize - 1) << (DMA_ctrl_data_ptr->DMA_SourceIncSize >> 26)));
        LDR      R4,[R0, #+24]
        SUBS     R4,R4,#+1
        LSRS     R2,R2,#+26
        LSL      R2,R4,R2
        ADDS     R3,R2,R3
??DMA_CtrlDataInit_0:
        STR      R3,[R1, #+0]
//  138   }
//  139 
//  140   /* DMA Destination Data End Address */
//  141   if (DMA_ctrl_data_ptr->DMA_Mode == DMA_Mode_MemScatterPri)
        LDR      R2,[R0, #+4]
        LDR      R3,[R0, #+20]
        CMP      R3,#+4
        ITEE     EQ 
        ADDEQ    R2,R2,#+12
//  142   {
//  143     /* Memory Scatter-Gather mode */
//  144     DMA_ctrl_table_ptr->DMA_DestEndAddr = (DMA_ctrl_data_ptr->DMA_DestBaseAddr + 12);
        LDRNE    R3,[R0, #+12]
        CMNNE    R3,#+1073741824
//  145   }
//  146   else
//  147   {
//  148     /* Other modes */
//  149     if (DMA_ctrl_data_ptr->DMA_DestIncSize == DMA_DestIncNo)
        BEQ.N    ??DMA_CtrlDataInit_1
//  150     {
//  151       DMA_ctrl_table_ptr->DMA_DestEndAddr = DMA_ctrl_data_ptr->DMA_DestBaseAddr;
//  152     }
//  153     else
//  154     {
//  155       DMA_ctrl_table_ptr->DMA_DestEndAddr = ( DMA_ctrl_data_ptr->DMA_DestBaseAddr +
//  156         ((DMA_ctrl_data_ptr->DMA_CycleSize - 1) << (DMA_ctrl_data_ptr->DMA_DestIncSize >> 30)));
        LDR      R4,[R0, #+24]
        SUBS     R4,R4,#+1
        LSRS     R3,R3,#+30
        LSL      R3,R4,R3
        ADDS     R2,R3,R2
??DMA_CtrlDataInit_1:
        STR      R2,[R1, #+4]
//  157     }
//  158   }
//  159 
//  160   /* DMA Control Data */
//  161   DMA_ctrl_table_ptr->DMA_Control = (DMA_ctrl_data_ptr->DMA_DestIncSize     |
//  162                                      DMA_ctrl_data_ptr->DMA_MemoryDataSize  |
//  163                                      DMA_ctrl_data_ptr->DMA_SourceIncSize   |
//  164                                      DMA_ctrl_data_ptr->DMA_DestProtCtrl    |
//  165                                      DMA_ctrl_data_ptr->DMA_SourceProtCtrl  |
//  166                                      DMA_ctrl_data_ptr->DMA_NumContinuous   |
//  167                                      ((DMA_ctrl_data_ptr->DMA_CycleSize - 1) << 4)  |
//  168                                      DMA_ctrl_data_ptr->DMA_Mode);
        LDR      R2,[R0, #+12]
        LDR      R3,[R0, #+16]
        ORRS     R2,R3,R2
        LDR      R3,[R0, #+8]
        ORRS     R2,R3,R2
        LDR      R3,[R0, #+36]
        ORRS     R2,R3,R2
        LDR      R3,[R0, #+32]
        ORRS     R2,R3,R2
        LDR      R3,[R0, #+28]
        ORRS     R2,R3,R2
        LDR      R3,[R0, #+24]
        SUBS     R3,R3,#+1
        ORR      R2,R2,R3, LSL #+4
        LDR      R0,[R0, #+20]
        ORRS     R0,R0,R2
        STR      R0,[R1, #+8]
//  169 }
        POP      {R4,PC}          ;; return
//  170 
//  171 /**
//  172   * @brief  Deinitializes the DMA registers to their default reset values.
//  173   * @param  None
//  174   * @retval None
//  175   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  176 void DMA_DeInit(void)
//  177 {
//  178   MDR_DMA->CFG = 0;                           /* Master Enable Off */
DMA_DeInit:
        LDR.N    R0,??DataTable8  ;; 0x40028004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  179   MDR_DMA->CTRL_BASE_PTR = 0;                 /* Control data base pointer */
        STR      R1,[R0, #+4]
//  180   MDR_DMA->CHNL_SW_REQUEST = 0;               /* Disable all sw requests */
        STR      R1,[R0, #+16]
//  181   MDR_DMA->CHNL_USEBURST_CLR = 0xFFFFFFFF;    /* Disable burst mode */
        MOV      R1,#-1
        STR      R1,[R0, #+24]
//  182   MDR_DMA->CHNL_REQ_MASK_CLR = 0xFFFFFFFF;    /* Clear mask request */
        STR      R1,[R0, #+32]
//  183   MDR_DMA->CHNL_ENABLE_CLR = 0xFFFFFFFF;      /* Clear channel enable */
        STR      R1,[R0, #+40]
//  184   MDR_DMA->CHNL_PRI_ALT_CLR = 0xFFFFFFFF;     /* Reset to primary data structure */
        STR      R1,[R0, #+48]
//  185   MDR_DMA->CHNL_PRIORITY_CLR = 0xFFFFFFFF;    /* Reset to default priority */
        STR      R1,[R0, #+56]
//  186   MDR_DMA->ERR_CLR = 0x01;                    /* Clear dma_err status */
        MOVS     R1,#+1
        STR      R1,[R0, #+72]
//  187 }
        BX       LR               ;; return
//  188 
//  189 /**
//  190   * @brief  Initializes the DMA Channel DMA_ControlTable structure according to the specified
//  191   *         parameters.
//  192   * @param  DMA_Channel: can be 0 to 31 or a value of @ref DMA_valid_channels to select the DMA Channel.
//  193   * @param  DMA_CtrlDataType: can be DMA_CTRL_DATA_PRIMARY or DMA_CTRL_DATA_ALTERNATE
//  194   * @param  DMA_CtrlStruct: pointer to a DMA_CtrlDataInitTypeDef structure that
//  195   *         contains the initial control data configuration, provided by the application
//  196   * 				for the specified DMA Channel.
//  197   * @retval None
//  198   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  199 void DMA_CtrlInit(uint8_t DMA_Channel, uint8_t DMA_CtrlDataType,  DMA_CtrlDataInitTypeDef* DMA_CtrlStruct)
//  200 {
//  201   /* Primary Control Data Init */
//  202   if (DMA_CtrlDataType == DMA_CTRL_DATA_PRIMARY)
DMA_CtrlInit:
        CMP      R1,#+0
        ITEE     EQ 
        LDREQ.N  R1,??DataTable8_1
        LDRNE.N  R1,??DataTable8_2  ;; 0x4002800c
        LDRNE    R1,[R1, #+0]
//  203   {
//  204     DMA_CtrlDataInit(DMA_CtrlStruct, &DMA_ControlTable[DMA_Channel]);
//  205   }
//  206 #if (DMA_AlternateData == 1)
//  207   /* Alternate Control Data Init */
//  208   else
//  209   {
//  210     uint32_t ptr = (MDR_DMA->ALT_CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
//  211     DMA_CtrlDataInit(DMA_CtrlStruct, (DMA_CtrlDataTypeDef *)ptr);
        B.N      ?Subroutine0
//  212   }
//  213 #endif
//  214 }
//  215 
//  216 /**
//  217   * @brief  Initializes the DMA Scatter-Gather Task structure according to the specified
//  218   *         parameters.
//  219   * @param  DMA_Task: specifies the current task number. Begins with 0.
//  220   * @param  DMA_CtrlStruct: pointer to a DMA_CtrlDataInitTypeDef structure that
//  221   *         contains the control data information for the specified task.
//  222   * @param  DMA_SG_TaskArray: pointer to a DMA_CtrlDataTypeDef array that
//  223   *         contains the copy of the alternate control data information for the specified task.
//  224   * @retval None
//  225   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  226 void DMA_SG_CtrlInit(uint32_t DMA_Task, DMA_CtrlDataTypeDef *DMA_SG_TaskArray,  DMA_CtrlDataInitTypeDef* DMA_CtrlStruct)
DMA_SG_CtrlInit:
        Nop      
//  227 {
//  228   DMA_CtrlDataInit(DMA_CtrlStruct, &DMA_SG_TaskArray[DMA_Task]);
        REQUIRE ?Subroutine0
        ;; // Fall through to label ?Subroutine0

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
?Subroutine0:
        ADD      R1,R1,R0, LSL #+4
        MOV      R0,R2
        B.N      DMA_CtrlDataInit
//  229 }
//  230 
//  231 /**
//  232   * @brief  Initializes the DMA Channel in Memory Scatter-Gather mode
//  233   * @param  DMA_Channel: can be 0 to 31 or a value of @ref DMA_valid_channels to select the DMA Channel.
//  234   * @param  DMA_SG_InitStruct: pointer to a DMA_Channel_SG_InitTypeDef array that
//  235   *         contains the configuration information for the specified DMA Channel.
//  236   * @retval None
//  237   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  238 void DMA_SG_Init( uint8_t DMA_Channel, DMA_Channel_SG_InitTypeDef *DMA_SG_InitStruct)
//  239 {
DMA_SG_Init:
        PUSH     {R4-R6,LR}
        SUB      SP,SP,#+40
        MOV      R4,R0
        MOV      R5,R1
//  240   DMA_CtrlDataInitTypeDef DMA_PriCtrlData;
//  241 
//  242   /* Check the parameters */
//  243   assert_param(DMA_AlternateData == 1);
//  244   assert_param(DMA_SG_InitStruct != 0);
//  245   assert_param(DMA_SG_InitStruct->DMA_SG_TaskNumber != 0);
//  246   assert_param(IS_DMA_CHANNEL(DMA_Channel));
//  247   assert_param(IS_DMA_AHB_PROT(DMA_SG_InitStruct->DMA_ProtCtrl));
//  248   assert_param(IS_DMA_PRIORITY(DMA_SG_InitStruct->DMA_Priority));
//  249   assert_param(IS_DMA_SOURCE_PROT(DMA_SG_InitStruct->DMA_SourceProtCtrl));
//  250   assert_param(IS_DMA_DEST_PROT(DMA_SG_InitStruct->DMA_DestProtCtrl));
//  251   assert_param(IS_DMA_BURST(DMA_SG_InitStruct->DMA_UseBurst));
//  252 
//  253   /* Check the CTRL_BASE_PTR initialisation */
//  254   MDR_DMA->CTRL_BASE_PTR = (uint32_t)DMA_ControlTable;
        LDR.N    R6,??DataTable8  ;; 0x40028004
        LDR.N    R0,??DataTable8_1
        STR      R0,[R6, #+4]
//  255 
//  256   /* Primary Control Data Init */
//  257   DMA_PriCtrlData.DMA_SourceBaseAddr = (uint32_t)(DMA_SG_InitStruct->DMA_SG_TaskArray);
        LDR      R1,[R5, #+0]
        STR      R1,[SP, #+0]
//  258   DMA_PriCtrlData.DMA_DestBaseAddr = (MDR_DMA->ALT_CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
        LDR      R1,[R6, #+8]
        ADD      R1,R1,R4, LSL #+4
        STR      R1,[SP, #+4]
//  259   DMA_PriCtrlData.DMA_SourceIncSize = DMA_SourceIncWord;
        MOV      R1,#+134217728
        STR      R1,[SP, #+8]
//  260   DMA_PriCtrlData.DMA_DestIncSize = DMA_DestIncWord;
        MOV      R1,#-2147483648
        STR      R1,[SP, #+12]
//  261   DMA_PriCtrlData.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        MOV      R1,#+570425344
        STR      R1,[SP, #+16]
//  262   DMA_PriCtrlData.DMA_Mode = DMA_Mode_MemScatterPri;
        MOVS     R1,#+4
        STR      R1,[SP, #+20]
//  263   DMA_PriCtrlData.DMA_CycleSize = DMA_SG_InitStruct->DMA_SG_TaskNumber << 2;
        LDR      R1,[R5, #+4]
        LSLS     R1,R1,#+2
        STR      R1,[SP, #+24]
//  264   DMA_PriCtrlData.DMA_NumContinuous = DMA_Transfers_4;
        MOV      R1,#+32768
        STR      R1,[SP, #+28]
//  265   DMA_PriCtrlData.DMA_SourceProtCtrl = DMA_SG_InitStruct->DMA_SourceProtCtrl;
        LDR      R1,[R5, #+8]
        STR      R1,[SP, #+32]
//  266   DMA_PriCtrlData.DMA_DestProtCtrl = DMA_SG_InitStruct->DMA_DestProtCtrl;
        LDR      R1,[R5, #+12]
        STR      R1,[SP, #+36]
//  267 
//  268   DMA_CtrlDataInit(&DMA_PriCtrlData, &DMA_ControlTable[DMA_Channel]);
        ADD      R1,R0,R4, LSL #+4
        ADD      R0,SP,#+0
        BL       DMA_CtrlDataInit
//  269 
//  270   /* DMA configuration register */
//  271   MDR_DMA->CFG = DMA_CFG_MASTER_ENABLE | DMA_SG_InitStruct->DMA_ProtCtrl;
        LDR      R0,[R5, #+16]
        ORR      R0,R0,#0x1
        STR      R0,[R6, #+0]
        MOVS     R0,#+1
        LSLS     R0,R0,R4
        LDRB     R1,[R5, #+21]
        CMP      R1,#+1
        ITE      EQ 
        STREQ    R0,[R6, #+20]
        STRNE    R0,[R6, #+24]
//  272 
//  273   /* Burst mode */
//  274   if (DMA_SG_InitStruct->DMA_UseBurst == DMA_BurstSet)
//  275   {
//  276     MDR_DMA->CHNL_USEBURST_SET = (1 << DMA_Channel);
//  277   }
//  278   else
//  279   {
//  280     MDR_DMA->CHNL_USEBURST_CLR = (1 << DMA_Channel);
//  281   }
//  282 
//  283   /* Channel mask clear */
//  284   MDR_DMA->CHNL_REQ_MASK_CLR = (1 << DMA_Channel);
        STR      R0,[R6, #+32]
//  285 
//  286   /* Enable channel */
//  287   MDR_DMA->CHNL_ENABLE_SET = (1 << DMA_Channel);
        STR      R0,[R6, #+36]
//  288 
//  289   /* Primary - Alternate control data structure selection */
//  290   MDR_DMA->CHNL_PRI_ALT_CLR = (1 << DMA_Channel);       /* Use Primary */
        STR      R0,[R6, #+48]
//  291 
//  292   /* Channel priority set */
//  293   if (DMA_SG_InitStruct->DMA_Priority == DMA_Priority_High)
        LDRB     R1,[R5, #+20]
        CMP      R1,#+1
        ITE      EQ 
        STREQ    R0,[R6, #+52]
        STRNE    R0,[R6, #+56]
//  294   {
//  295     MDR_DMA->CHNL_PRIORITY_SET = (1 << DMA_Channel);      /* High priority */
//  296   }
//  297   else
//  298   {
//  299     MDR_DMA->CHNL_PRIORITY_CLR = (1 << DMA_Channel);      /* Default priority */
//  300   }
//  301 }
        ADD      SP,SP,#+40
        POP      {R4-R6,PC}       ;; return
//  302 
//  303 /**
//  304   * @brief  Initializes the DMA Channel according to the specified
//  305   *         parameters in the DMA_InitStruct.
//  306   * @param  DMA_Channel: can be 0 to 31 or a value of @ref DMA_valid_channels to select the DMA Channel.
//  307   * @param  DMA_InitStruct: pointer to a DMA_ChannelInitTypeDef structure that
//  308   *         contains the configuration information for the specified DMA Channel.
//  309   * @retval None
//  310   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  311 void DMA_Init(uint8_t DMA_Channel, DMA_ChannelInitTypeDef* DMA_InitStruct)
//  312 {
DMA_Init:
        PUSH     {R4-R6,LR}
        MOV      R6,R0
        MOV      R4,R1
//  313   /* Check the parameters */
//  314   assert_param(IS_DMA_CHANNEL(DMA_Channel));
//  315   assert_param(IS_DMA_AHB_PROT(DMA_InitStruct->DMA_ProtCtrl));
//  316   assert_param(IS_DMA_PRIORITY(DMA_InitStruct->DMA_Priority));
//  317   assert_param(IS_DMA_BURST(DMA_InitStruct->DMA_UseBurst));
//  318   assert_param(IS_DMA_SELECT_DATA_STRUCTURE(DMA_InitStruct->DMA_SelectDataStructure));
//  319 
//  320   /* Check the CTRL_BASE_PTR initialisation */
//  321   MDR_DMA->CTRL_BASE_PTR = (uint32_t)DMA_ControlTable;
        LDR.N    R5,??DataTable8  ;; 0x40028004
        LDR.N    R1,??DataTable8_1
        STR      R1,[R5, #+4]
//  322 
//  323   /* Primary Control Data Init */
//  324   if (DMA_InitStruct->DMA_PriCtrlData)
        LDR      R0,[R4, #+0]
        CBZ.N    R0,??DMA_Init_0
//  325   {
//  326     DMA_CtrlDataInit(DMA_InitStruct->DMA_PriCtrlData, &DMA_ControlTable[DMA_Channel]);
        ADD      R1,R1,R6, LSL #+4
        BL       DMA_CtrlDataInit
//  327   }
//  328 
//  329 #if (DMA_AlternateData == 1)
//  330   /* Alternate Control Data Init */
//  331   if (DMA_InitStruct->DMA_AltCtrlData)
??DMA_Init_0:
        LDR      R0,[R4, #+4]
        CBZ.N    R0,??DMA_Init_1
//  332   {
//  333     uint32_t ptr = (MDR_DMA->ALT_CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
        LDR      R1,[R5, #+8]
//  334     DMA_CtrlDataInit(DMA_InitStruct->DMA_AltCtrlData, (DMA_CtrlDataTypeDef *)ptr);
        ADD      R1,R1,R6, LSL #+4
        BL       DMA_CtrlDataInit
//  335   }
//  336 #endif
//  337 
//  338   /* DMA configuration register */
//  339   MDR_DMA->CFG = DMA_CFG_MASTER_ENABLE | DMA_InitStruct->DMA_ProtCtrl;
??DMA_Init_1:
        LDR      R0,[R4, #+8]
        ORR      R0,R0,#0x1
        STR      R0,[R5, #+0]
        MOVS     R0,#+1
        LSLS     R0,R0,R6
        LDRB     R1,[R4, #+13]
        CMP      R1,#+1
        ITE      EQ 
        STREQ    R0,[R5, #+20]
        STRNE    R0,[R5, #+24]
//  340 
//  341   /* Burst mode */
//  342   if (DMA_InitStruct->DMA_UseBurst == DMA_BurstSet)
//  343   {
//  344     MDR_DMA->CHNL_USEBURST_SET = (1 << DMA_Channel);
//  345   }
//  346   else
//  347   {
//  348     MDR_DMA->CHNL_USEBURST_CLR = (1 << DMA_Channel);
//  349   }
//  350 
//  351   /* Channel mask clear */
//  352   MDR_DMA->CHNL_REQ_MASK_CLR = (1 << DMA_Channel);
        STR      R0,[R5, #+32]
//  353 
//  354   /* Enable channel */
//  355   MDR_DMA->CHNL_ENABLE_SET = (1 << DMA_Channel);
        STR      R0,[R5, #+36]
//  356 
//  357   /* Primary - Alternate control data structure selection */
//  358   if (DMA_InitStruct->DMA_SelectDataStructure == DMA_CTRL_DATA_PRIMARY)
        LDRB     R1,[R4, #+14]
        CMP      R1,#+0
        ITE      EQ 
        STREQ    R0,[R5, #+48]
        STRNE    R0,[R5, #+44]
//  359   {
//  360     MDR_DMA->CHNL_PRI_ALT_CLR = (1 << DMA_Channel);       /* Use Primary */
//  361   }
//  362   else
//  363   {
//  364     MDR_DMA->CHNL_PRI_ALT_SET = (1 << DMA_Channel);       /* Use Alternate */
//  365   }
//  366 
//  367   /* Channel priority set */
//  368   if (DMA_InitStruct->DMA_Priority == DMA_Priority_High)
        LDRB     R1,[R4, #+12]
        CMP      R1,#+1
        BNE.N    ??DMA_Init_2
//  369   {
//  370     MDR_DMA->CHNL_PRIORITY_SET = (1 << DMA_Channel);      /* High priority */
        STR      R0,[R5, #+52]
        POP      {R4-R6,PC}
//  371   }
//  372   else
//  373   {
//  374     MDR_DMA->CHNL_PRIORITY_CLR = (1 << DMA_Channel);      /* Default priority */
??DMA_Init_2:
        STR      R0,[R5, #+56]
//  375   }
//  376 }
        POP      {R4-R6,PC}       ;; return
//  377 
//  378 /**
//  379   * @brief  Fills each DMA_InitStruct member with its default value.
//  380   * @param  DMA_InitStruct : pointer to a DMA_ChannelInitTypeDef structure which will
//  381   *         be initialized.
//  382   * @retval None
//  383   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  384 void DMA_StructInit(DMA_ChannelInitTypeDef* DMA_InitStruct)
//  385 {
//  386   DMA_InitStruct->DMA_PriCtrlData = 0;
DMA_StructInit:
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  387   DMA_InitStruct->DMA_AltCtrlData = 0;
        STR      R1,[R0, #+4]
//  388   DMA_InitStruct->DMA_ProtCtrl = 0;
        STR      R1,[R0, #+8]
//  389   DMA_InitStruct->DMA_Priority = 0;
        STRB     R1,[R0, #+12]
//  390   DMA_InitStruct->DMA_UseBurst = 0;
        STRB     R1,[R0, #+13]
//  391   DMA_InitStruct->DMA_SelectDataStructure = 0;
        STRB     R1,[R0, #+14]
//  392 }
        BX       LR               ;; return
//  393 
//  394 /**
//  395   * @brief  Fills each DMA_SG_InitStruct member with its default value.
//  396   * @param  DMA_SG_InitStruct : pointer to a DMA_Channel_SG_InitTypeDef structure which will
//  397   *         be initialized.
//  398   * @retval None
//  399   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  400 void DMA_SG_StructInit(DMA_Channel_SG_InitTypeDef* DMA_SG_InitStruct)
//  401 {
//  402   DMA_SG_InitStruct->DMA_SG_TaskArray = 0;
DMA_SG_StructInit:
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  403   DMA_SG_InitStruct->DMA_SG_TaskNumber = 0;
        STR      R1,[R0, #+4]
//  404   DMA_SG_InitStruct->DMA_SourceProtCtrl = 0;
        STR      R1,[R0, #+8]
//  405   DMA_SG_InitStruct->DMA_DestProtCtrl = 0;
        STR      R1,[R0, #+12]
//  406   DMA_SG_InitStruct->DMA_ProtCtrl = 0;
        STR      R1,[R0, #+16]
//  407   DMA_SG_InitStruct->DMA_Priority = 0;
        STRB     R1,[R0, #+20]
//  408   DMA_SG_InitStruct->DMA_UseBurst = 0;
        STRB     R1,[R0, #+21]
//  409 }
        BX       LR               ;; return
//  410 
//  411 /**
//  412   * @brief  Enables or disables the specified DMA Channel.
//  413   * @param  DMA_Channel: can be 0 to 31 or a value of @ref DMA_valid_channels to select the DMA Channel.
//  414   * @param  NewState: new state of the DMA Channel.
//  415   *         This parameter can be: ENABLE or DISABLE.
//  416   * @retval None
//  417   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  418 void DMA_Cmd(uint8_t DMA_Channel, FunctionalState NewState)
//  419 {
DMA_Cmd:
        MOVS     R2,#+1
        LSL      R0,R2,R0
        LDR.N    R2,??DataTable8_3  ;; 0x40028028
        CMP      R1,#+1
        BNE.N    ??DMA_Cmd_0
//  420   /* Check the parameters */
//  421   assert_param(IS_DMA_CHANNEL(DMA_Channel));
//  422 
//  423   /* Channel Enable/Disable */
//  424   if ( NewState == ENABLE)
//  425   {
//  426     MDR_DMA->CHNL_ENABLE_SET = (1 << DMA_Channel);
        STR      R0,[R2, #+0]
        BX       LR
//  427   }
//  428   else
//  429   {
//  430     MDR_DMA->CHNL_ENABLE_CLR = (1 << DMA_Channel);
??DMA_Cmd_0:
        STR      R0,[R2, #+4]
//  431   }
//  432 }
        BX       LR               ;; return
//  433 
//  434 /**
//  435   * @brief  Generates the specified DMA Channel software request.
//  436   * @param  DMA_Channel: can be 0 to 31 or a value of @ref DMA_valid_channels to select the DMA Channel.
//  437   * @retval None
//  438   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  439 void DMA_Request(uint8_t DMA_Channel)
//  440 {
//  441   /* Check the parameters */
//  442   assert_param(IS_DMA_CHANNEL(DMA_Channel));
//  443 
//  444   /* Set SW Request */
//  445   MDR_DMA->CHNL_SW_REQUEST = (1 << DMA_Channel);
DMA_Request:
        MOVS     R1,#+1
        LSL      R0,R1,R0
        LDR.N    R1,??DataTable8_4  ;; 0x40028014
        STR      R0,[R1, #+0]
//  446 }
        BX       LR               ;; return
//  447 
//  448 /**
//  449   * @brief  Clears the DMA Error bit.
//  450   * @param  None
//  451   * @retval None
//  452   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  453 void DMA_ClearError(void)
//  454 {
//  455   MDR_DMA->ERR_CLR = 0x01;            /* Clear dma_err status */
DMA_ClearError:
        LDR.N    R0,??DataTable8_5  ;; 0x4002804c
        MOVS     R1,#+1
        STR      R1,[R0, #+0]
//  456 }
        BX       LR               ;; return
//  457 
//  458 /**
//  459   * @brief  Returns the number of remaining transfers in the current
//  460   *         DMA Channel cycle.
//  461   * @param  DMA_Channel: can be 0 to 31 or a value of @ref DMA_valid_channels to select the DMA Channel.
//  462   * @param  DMA_CtrlData: specifies the primary / alternate control data table to check.
//  463   *         This parameter can be a value of @ref DMA_data_structure_selection.
//  464   * @retval The number of remaining transfers in the current DMA Channel
//  465   *         cycle and the current control data structure.
//  466   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  467 uint32_t DMA_GetCurrTransferCounter(uint8_t DMA_Channel, uint8_t DMA_CtrlData)
//  468 {
//  469   DMA_CtrlDataTypeDef *ptr;     /* Pointer to the channels's data structure */
//  470 
//  471   /* Check the parameters */
//  472   assert_param(IS_DMA_CHANNEL(DMA_Channel));
//  473   assert_param(IS_DMA_SELECT_DATA_STRUCTURE(DMA_CtrlData));
//  474 
//  475   /* Init the control data pointer */
//  476   if (DMA_CtrlData == DMA_CTRL_DATA_PRIMARY)
DMA_GetCurrTransferCounter:
        LDR.N    R2,??DataTable8_6  ;; 0x40028008
        CMP      R1,#+0
        ITE      EQ 
        LDREQ    R1,[R2, #+0]
        LDRNE    R1,[R2, #+4]
//  477   {
//  478     ptr = (DMA_CtrlDataTypeDef *)(MDR_DMA->CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
//  479   }
//  480   else
//  481   {
//  482     ptr = (DMA_CtrlDataTypeDef *)(MDR_DMA->ALT_CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
        ADDS     R0,R1,R0, LSL #+4
//  483   }
//  484 
//  485   /* Read the number of remaining transfers */
//  486   return (((ptr->DMA_Control & DMA_CONTROL_MINUS_1)>>4) + 1);
        LDR      R0,[R0, #+8]
        UBFX     R0,R0,#+4,#+10
        ADDS     R0,R0,#+1
        BX       LR               ;; return
//  487 }
//  488 
//  489 /**
//  490   * @brief  Checks whether the specified DMA Channel flag is set or not.
//  491   * @param  DMA_Channel: can be 0 to 31 or a value of @ref DMA_valid_channels to select the DMA Channel.
//  492   * @param  DMA_Flag: specifies the flag to check.
//  493   *         This parameter can be one of the following values:
//  494   *           @arg DMA_FLAG_DMA_ENA:       DMA unit global enable status.
//  495   *           @arg DMA_FLAG_DMA_ERR:       DMA unit bus error status.
//  496   *           @arg DMA_FLAG_CHNL_ENA:      DMA channel enable status.
//  497   *           @arg DMA_FLAG_CHNL_MASK:     DMA channel request mask status.
//  498   *           @arg DMA_FLAG_CHNL_WAIT:     DMA channel wait on request status.
//  499   *           @arg DMA_FLAG_CHNL_BURST:    DMA channel burst mode status.
//  500   *           @arg DMA_FLAG_CHNL_ALT:      DMA channel alternate control data status.
//  501   *           @arg DMA_FLAG_CHNL_PRIORITY: DMA channel priority status.
//  502   * @retval The new state of DMA_FLAG (SET or RESET).
//  503   */

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
//  504 FlagStatus DMA_GetFlagStatus(uint8_t DMA_Channel, uint8_t DMA_Flag)
//  505 {
DMA_GetFlagStatus:
        MOVS     R2,#+1
        LSL      R0,R2,R0
        LDR.N    R2,??DataTable8_7  ;; 0x40028000
        SUBS     R1,R1,#+1
        CMP      R1,#+7
        BHI.N    ??DMA_GetFlagStatus_1
        TBB      [PC, R1]
        DATA
??DMA_GetFlagStatus_0:
        DC8      0x4,0x6,0xA,0xC
        DC8      0xE,0x10,0x12,0x14
        THUMB
//  506   /* Check the parameters */
//  507   assert_param(IS_DMA_CHANNEL(DMA_Channel));
//  508   assert_param(IS_DMA_FLAG(DMA_Flag));
//  509 
//  510   /* Read Flag Status */
//  511   switch(DMA_Flag)
//  512   {
//  513     case DMA_FLAG_DMA_ENA:
//  514       return (FlagStatus)(MDR_DMA->STATUS & DMA_STATUS_MASTER_ENABLE);
??DMA_GetFlagStatus_2:
        LDR      R0,[R2, #+0]
        B.N      ??DMA_GetFlagStatus_3
//  515     case DMA_FLAG_DMA_ERR:
//  516       return (FlagStatus)(MDR_DMA->ERR_CLR & 0x01);
??DMA_GetFlagStatus_4:
        LDR      R0,[R2, #+76]
??DMA_GetFlagStatus_3:
        AND      R0,R0,#0x1
        BX       LR
//  517     case DMA_FLAG_CHNL_ENA:
//  518       return (FlagStatus)(MDR_DMA->CHNL_ENABLE_SET & (1 << DMA_Channel));
??DMA_GetFlagStatus_5:
        LDR      R1,[R2, #+40]
        B.N      ??DMA_GetFlagStatus_6
//  519     case DMA_FLAG_CHNL_MASK:
//  520       return (FlagStatus)(MDR_DMA->CHNL_REQ_MASK_SET & (1 << DMA_Channel));
??DMA_GetFlagStatus_7:
        LDR      R1,[R2, #+32]
        B.N      ??DMA_GetFlagStatus_6
//  521     case DMA_FLAG_CHNL_WAIT:
//  522       return (FlagStatus)(MDR_DMA->WAITONREQ_STATUS & (1 << DMA_Channel));
??DMA_GetFlagStatus_8:
        LDR      R1,[R2, #+16]
        B.N      ??DMA_GetFlagStatus_6
//  523     case DMA_FLAG_CHNL_BURST:
//  524       return (FlagStatus)(MDR_DMA->CHNL_USEBURST_SET & (1 << DMA_Channel));
??DMA_GetFlagStatus_9:
        LDR      R1,[R2, #+24]
        B.N      ??DMA_GetFlagStatus_6
//  525     case DMA_FLAG_CHNL_ALT:
//  526       return (FlagStatus)(MDR_DMA->CHNL_PRI_ALT_SET & (1 << DMA_Channel));
??DMA_GetFlagStatus_10:
        LDR      R1,[R2, #+48]
        B.N      ??DMA_GetFlagStatus_6
//  527     case DMA_FLAG_CHNL_PRIORITY:
//  528       return (FlagStatus)(MDR_DMA->CHNL_PRIORITY_SET & (1 << DMA_Channel));
??DMA_GetFlagStatus_11:
        LDR      R1,[R2, #+56]
??DMA_GetFlagStatus_6:
        ANDS     R0,R0,R1
        UXTB     R0,R0
        BX       LR
//  529     default:
//  530       return (FlagStatus)0;
??DMA_GetFlagStatus_1:
        MOVS     R0,#+0
        BX       LR               ;; return
//  531   }
//  532 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8:
        DC32     0x40028004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_1:
        DC32     DMA_ControlTable

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_2:
        DC32     0x4002800c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_3:
        DC32     0x40028028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_4:
        DC32     0x40028014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_5:
        DC32     0x4002804c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_6:
        DC32     0x40028008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable8_7:
        DC32     0x40028000

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
//  533 
//  534 /** @} */ /* End of group DMA_Private_Functions */
//  535 
//  536 /** @} */ /* End of group DMA */
//  537 
//  538 /** @} */ /* End of group __MDR32F9Qx_StdPeriph_Driver */
//  539 
//  540 /******************* (C) COPYRIGHT 2010 Phyton *********************************
//  541 *
//  542 * END OF FILE MDR32F9Qx_dma.c */
//  543 
// 
// 1 024 bytes in section .bss
//   556 bytes in section .text
// 
//   556 bytes of CODE memory
// 1 024 bytes of DATA memory
//
//Errors: none
//Warnings: none
