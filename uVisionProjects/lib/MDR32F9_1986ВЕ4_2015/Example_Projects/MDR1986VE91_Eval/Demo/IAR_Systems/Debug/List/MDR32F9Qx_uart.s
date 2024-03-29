///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.40.2.53884/W32 for ARM    22/Sep/2015  17:25:58 /
// Copyright 1999-2012 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  C:\WORK\Milandr.MDR1986BExx.1.4.0\Libraries\MDR32F9Qx_S /
//                    tdPeriph_Driver\src\MDR32F9Qx_uart.c                    /
//    Command line =  C:\WORK\Milandr.MDR1986BExx.1.4.0\Libraries\MDR32F9Qx_S /
//                    tdPeriph_Driver\src\MDR32F9Qx_uart.c --preprocess       /
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
//                    986VE91_Eval\Demo\IAR_Systems\Debug\List\MDR32F9Qx_uart /
//                    .s                                                      /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME MDR32F9Qx_uart

        #define SHT_PROGBITS 0x1

        EXTERN RST_CLK_GetClocksFreq

        PUBLIC UART_BRGInit
        PUBLIC UART_BreakLine
        PUBLIC UART_ClearITPendingBit
        PUBLIC UART_Cmd
        PUBLIC UART_DMACmd
        PUBLIC UART_DMAConfig
        PUBLIC UART_DeInit
        PUBLIC UART_GetFlagStatus
        PUBLIC UART_GetITStatus
        PUBLIC UART_GetITStatusMasked
        PUBLIC UART_ITConfig
        PUBLIC UART_Init
        PUBLIC UART_IrDACmd
        PUBLIC UART_IrDAConfig
        PUBLIC UART_ReceiveData
        PUBLIC UART_SendData
        PUBLIC UART_StructInit

// C:\WORK\Milandr.MDR1986BExx.1.4.0\Libraries\MDR32F9Qx_StdPeriph_Driver\src\MDR32F9Qx_uart.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    MDR32F9Qx_uart.c
//    4   * @author  Phyton Application Team
//    5   * @version V1.4.0
//    6   * @date    22/06/2010
//    7   * @brief   This file contains all the UART firmware functions.
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
//   20   * FILE MDR32F9Qx_uart.c
//   21   */
//   22 
//   23 /* Includes ------------------------------------------------------------------*/
//   24 #include "MDR32F9Qx_config.h"
//   25 #include "MDR32F9Qx_uart.h"
//   26 #include "MDR32F9Qx_rst_clk.h"
//   27 
//   28 
//   29 #define ASSERT_INFO_FILE_ID FILEID__MDR32F9X_UART_C
//   30 
//   31 /** @addtogroup __MDR32F9Qx_StdPeriph_Driver MDR32F9Qx Standard Peripherial Driver
//   32   * @{
//   33   */
//   34 
//   35 /** @defgroup UART UART
//   36   * @{
//   37   */
//   38 
//   39 /** @defgroup UART_Private_Defines UART Private Defines
//   40   * @{
//   41   */
//   42 
//   43 #define CR_EN_Set                 ((uint16_t)0x0001)  /*!< UART Enable Mask */
//   44 #define CR_EN_Reset               ((uint16_t)0xFFFE)  /*!< UART Disable Mask */
//   45 
//   46 #define CR_SIREN_Set              ((uint16_t)0x0002)  /*!< UART IrDA mode Enable Mask */
//   47 #define CR_SIREN_Reset            ((uint16_t)0xFFFD)  /*!< UART IrDA mode Disable Mask */
//   48 
//   49 #define CR_FC_Mask                ((uint16_t)0xFF80)  /*!< UART CR Flow control Bits Mask */
//   50 
//   51 #define LCR_H_BRK_Set             ((uint16_t)0x0001)  /*!< UART Break Line Set Mask */
//   52 #define LCR_H_BRK_Reset           ((uint16_t)0xFFFE)  /*!< UART Break Line Reset Mask */
//   53 #define LCR_H_Clear_Mask          ((uint16_t)0x00FF)  /*!< UART LCR_H Mask */
//   54 
//   55 #define FBRD_Fract_Mask           ((uint16_t)0x003F)  /*!< Fractional divider Mask */
//   56 
//   57 #define IrLPBaud16                ((uint32_t)1843200) /*!< F_IrLPBaud16 nominal frequency Hz */
//   58 
//   59 #define UART1_BRG_Mask            ((uint32_t)0x0007)  /*!< UART1 clock divider Mask */
//   60 #define UART2_BRG_Mask            ((uint32_t)0x0700)  /*!< UART2 clock divider Mask */
//   61 #define UART2_BRG_Offs            ((uint32_t)0x0008)  /*!< UART2 clock divider Offset */
//   62 
//   63 
//   64 #if defined (USE_MDR1986VE3)
//   65 #define UART3_BRG_Mask				((uint32_t)0x0007) /*!< UART3  clock divider Mask */
//   66 #define UART4_BRG_Mask				((uint32_t)0x0700) /*!< UART4 clock divider Mask */
//   67 #define UART4_BRG_Offs              ((uint32_t)0x0008) /*!< UART4 clock divider Offset */
//   68 
//   69 #endif // #if defined (USE_MDR1986VE3)
//   70 
//   71 /** @} */ /* End of group UART_Private_Defines */
//   72 
//   73 /** @defgroup UART_Private_Functions UART Private Functions
//   74   * @{
//   75   */
//   76 
//   77 /**
//   78   * @brief  Resets the UARTx peripheral registers to their default reset values.
//   79   * @param  UARTx: Select the UART peripheral.
//   80   *         This parameter can be one of the following values: UART1, UART2.
//   81   * @retval None
//   82   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   83 void UART_DeInit(MDR_UART_TypeDef* UARTx)
//   84 {
//   85   /* Check the parameters */
//   86   assert_param(IS_UART_ALL_PERIPH(UARTx));
//   87 
//   88   /* Clear UART CR */
//   89   UARTx->CR = 0;
UART_DeInit:
        MOVS     R1,#+0
        STR      R1,[R0, #+48]
//   90   UARTx->LCR_H = 0;
        STR      R1,[R0, #+44]
//   91   UARTx->RSR_ECR = 0;
        STR      R1,[R0, #+4]
//   92   UARTx->FR = UART_FLAG_TXFE | UART_FLAG_RXFE;
        MOVS     R1,#+144
        STR      R1,[R0, #+24]
//   93   UARTx->ILPR = 0;
        MOVS     R1,#+0
        STR      R1,[R0, #+32]
//   94   UARTx->IBRD = 0;
        STR      R1,[R0, #+36]
//   95   UARTx->FBRD = 0;
        STR      R1,[R0, #+40]
//   96   UARTx->IFLS = UART_IT_FIFO_LVL_8words;
        MOVS     R1,#+2
        STR      R1,[R0, #+52]
//   97   UARTx->IMSC = 0;
        MOVS     R1,#+0
        STR      R1,[R0, #+56]
//   98   UARTx->DMACR = 0;
        STR      R1,[R0, #+72]
//   99   /* Set UART CR[RXE] and UART CR[TXE] bits */
//  100   UARTx->CR = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
        MOV      R1,#+768
        STR      R1,[R0, #+48]
//  101 }
        BX       LR               ;; return
//  102 
//  103 /**
//  104   * @brief  Initializes the UARTx peripheral according to the specified
//  105   *         parameters in the UART_InitStruct.
//  106   * @param  UARTx: Select the UART peripheral.
//  107   *         This parameter can be one of the following values:
//  108   *         UART1, UART2.
//  109   * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure
//  110   *         that contains the configuration information for the specified UART peripheral.
//  111   * @retval The Baud Rate status (BaudRateValid or BaudRateInvalid).
//  112   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  113 BaudRateStatus UART_Init ( MDR_UART_TypeDef* UARTx,
//  114 						   UART_InitTypeDef* UART_InitStruct )
//  115 {
UART_Init:
        PUSH     {R4-R6,LR}
        SUB      SP,SP,#+24
        MOV      R4,R0
        MOV      R5,R1
//  116 	uint32_t tmpreg, cpuclock;
//  117 	uint32_t realspeed, speederror;
//  118 	uint32_t divider;
//  119 	uint32_t integerdivider;
//  120 	uint32_t fractionaldivider;
//  121 	RST_CLK_FreqTypeDef RST_CLK_Clocks;
//  122 
//  123 	/* Check the parameters */
//  124 	assert_param(IS_UART_ALL_PERIPH(UARTx));
//  125 	assert_param(IS_UART_BAUDRATE(UART_InitStruct->UART_BaudRate));
//  126 	assert_param(IS_UART_WORD_LENGTH(UART_InitStruct->UART_WordLength));
//  127 	assert_param(IS_UART_STOPBITS(UART_InitStruct->UART_StopBits));
//  128 	assert_param(IS_UART_PARITY(UART_InitStruct->UART_Parity));
//  129 	assert_param(IS_UART_FIFOMODE(UART_InitStruct->UART_FIFOMode));
//  130 	assert_param(IS_UART_HARDWARE_FLOW_CONTROL(UART_InitStruct->UART_HardwareFlowControl));
//  131 
//  132 	/* Configure the UART Baud Rate */
//  133 	RST_CLK_GetClocksFreq(&RST_CLK_Clocks);
        ADD      R0,SP,#+0
        BL       RST_CLK_GetClocksFreq
//  134 	cpuclock = RST_CLK_Clocks.CPU_CLK_Frequency;
        LDR      R0,[SP, #+0]
//  135 #if defined (USE_MDR1986VE3) /* For Cortex M1 */
//  136 	if(( UARTx == MDR_UART3 ) || (UARTx == MDR_UART4))
//  137 		tmpreg = MDR_RST_CLK->UART_SSP_CLOCK;
//  138 	else
//  139 		if(( UARTx == MDR_UART1 ) || (UARTx == MDR_UART2))
//  140 #endif // #if defined (USE_MDR1986VE3) /* For Cortex M1 */
//  141 			tmpreg = MDR_RST_CLK->UART_CLOCK;
        LDR.N    R1,??DataTable1  ;; 0x40020028
        LDR      R1,[R1, #+0]
//  142 
//  143 	if (UARTx == MDR_UART1) {
        MOVS     R2,#+1
        LDR.N    R3,??DataTable1_1  ;; 0x40030000
        CMP      R4,R3
        BEQ.N    ??UART_Init_0
//  144 		cpuclock /= (1 << (tmpreg & UART1_BRG_Mask));
//  145 	}
//  146 	else
//  147 		if (UARTx == MDR_UART2) {
        LDR.N    R3,??DataTable1_2  ;; 0x40038000
        CMP      R4,R3
        BNE.N    ??UART_Init_1
//  148 			cpuclock /= (1 << ((tmpreg & UART2_BRG_Mask) >> UART2_BRG_Offs));
        LSRS     R1,R1,#+8
??UART_Init_0:
        AND      R1,R1,#0x7
        LSL      R1,R2,R1
        UDIV     R0,R0,R1
//  149 		}
//  150 #if defined (USE_MDR1986VE3) /* For Cortex M1 */
//  151 		else
//  152 			if(UARTx == MDR_UART3) {
//  153 				cpuclock /= (1 << (tmpreg & UART3_BRG_Mask ));
//  154 			}
//  155 			else
//  156 				if(UARTx == MDR_UART4) {
//  157 					cpuclock /= (1 << ((tmpreg & UART4_BRG_Mask) >> UART4_BRG_Offs));
//  158 				}
//  159 #endif // #if defined (USE_MDR1986VE3) /* For Cortex M1 */
//  160 
//  161 	/* Determine the integer part */
//  162 	divider = cpuclock / (UART_InitStruct->UART_BaudRate >> 2);
??UART_Init_1:
        LDR      R1,[R5, #+0]
        LSRS     R2,R1,#+2
        UDIV     R2,R0,R2
//  163 	integerdivider = divider >> 6;
        LSRS     R3,R2,#+6
//  164 	/* Determine the fractional part */
//  165 	fractionaldivider = (divider & FBRD_Fract_Mask);
        AND      R2,R2,#0x3F
//  166 	/* Determine the speed error */
//  167 	realspeed = (cpuclock * 4) / ((integerdivider * 64) + fractionaldivider);
//  168 	speederror = ((realspeed - UART_InitStruct->UART_BaudRate) * 128)
//  169 			/ UART_InitStruct->UART_BaudRate;
//  170 	if (speederror > 2) {
        LSLS     R0,R0,#+2
        ADD      R6,R2,R3, LSL #+6
        UDIV     R0,R0,R6
        SUBS     R0,R0,R1
        LSLS     R0,R0,#+7
        UDIV     R0,R0,R1
        CMP      R0,#+3
        IT       CS 
        MOVCS    R0,#+0
//  171 		return BaudRateInvalid;
        BCS.N    ??UART_Init_2
//  172 	}
//  173 	/* Write UART Baud Rate */
//  174 	UARTx->IBRD = integerdivider;
        STR      R3,[R4, #+36]
//  175 	UARTx->FBRD = fractionaldivider;
        STR      R2,[R4, #+40]
//  176 
//  177 	/* UART LCR_H configuration */
//  178 	/* Set the WLEN bits according to UART_WordLength value */
//  179 	/* Set STP2 bit according to UART_StopBits value */
//  180 	/* Set PEN, EPS and SPS bits according to UART_Parity value */
//  181 	/* Set FEN bit according to UART_FIFOMode value */
//  182 	tmpreg = UARTx->LCR_H;
//  183 	tmpreg |= UART_InitStruct->UART_WordLength | UART_InitStruct->UART_StopBits
//  184 			| UART_InitStruct->UART_Parity | UART_InitStruct->UART_FIFOMode;
//  185 	UARTx->LCR_H = tmpreg;
        LDR      R0,[R4, #+44]
        LDRH     R1,[R5, #+4]
        LDRH     R2,[R5, #+6]
        ORRS     R1,R2,R1
        LDRH     R2,[R5, #+8]
        ORRS     R1,R2,R1
        LDRH     R2,[R5, #+10]
        ORRS     R1,R2,R1
        ORRS     R0,R1,R0
        STR      R0,[R4, #+44]
//  186 
//  187 	/* UART CR configuration */
//  188 	tmpreg = UARTx->CR;
//  189 	/* Clear UART CR Flow control bits */
//  190 	tmpreg &= ~CR_FC_Mask;
//  191 	/* Set UART CR Flow control bits */
//  192 	tmpreg |= UART_InitStruct->UART_HardwareFlowControl;
//  193 	/* Write to UART CR */
//  194 	UARTx->CR = tmpreg;
        LDR      R0,[R4, #+48]
        BFC      R0,#+7,#+9
        LDRH     R1,[R5, #+12]
        ORRS     R0,R1,R0
        STR      R0,[R4, #+48]
//  195 
//  196 	return BaudRateValid;
        MOVS     R0,#+1
??UART_Init_2:
        ADD      SP,SP,#+24
        POP      {R4-R6,PC}       ;; return
//  197 }
//  198 
//  199 /**
//  200   * @brief  Fills each UART_InitStruct member with its default value.
//  201   * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure
//  202   *         that is to be initialized.
//  203   * @retval None
//  204   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  205 void UART_StructInit(UART_InitTypeDef* UART_InitStruct)
//  206 {
//  207   /* UART_InitStruct members default value */
//  208   UART_InitStruct->UART_BaudRate = 9600;
UART_StructInit:
        MOV      R1,#+9600
        STR      R1,[R0, #+0]
//  209   UART_InitStruct->UART_WordLength = UART_WordLength5b;
        MOVS     R1,#+0
        STRH     R1,[R0, #+4]
//  210   UART_InitStruct->UART_StopBits = UART_StopBits1;
        STRH     R1,[R0, #+6]
//  211   UART_InitStruct->UART_Parity = UART_Parity_No;
        STRH     R1,[R0, #+8]
//  212   UART_InitStruct->UART_FIFOMode = UART_FIFO_OFF;
        STRH     R1,[R0, #+10]
//  213   UART_InitStruct->UART_HardwareFlowControl = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
        MOV      R1,#+768
        STRH     R1,[R0, #+12]
//  214 }
        BX       LR               ;; return
//  215 
//  216 /**
//  217   * @brief  Enables or disables the specified UART peripheral.
//  218   * @param  UARTx: Select the UART peripheral.
//  219   *         This parameter can be one of the following values:
//  220   *         UART1, UART2.
//  221   * @param  NewState: new state of the UARTx peripheral.
//  222   *         This parameter can be: ENABLE or DISABLE.
//  223   * @retval None
//  224   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  225 void UART_Cmd(MDR_UART_TypeDef* UARTx, FunctionalState NewState)
//  226 {
//  227   /* Check the parameters */
//  228   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  229   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  230 
//  231   if (NewState != DISABLE)
UART_Cmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+48]
        ITEE     NE 
        ORRNE    R1,R1,#0x1
        MOVWEQ   R2,#+65534
        ANDEQ    R1,R2,R1
//  232   {
//  233     /* Enable the selected UART by setting the UARTEN bit in the CR register */
//  234     UARTx->CR |= CR_EN_Set;
//  235   }
//  236   else
//  237   {
//  238     /* Disable the selected UART by clearing the UARTEN bit in the CR register */
//  239     UARTx->CR &= CR_EN_Reset;
        STR      R1,[R0, #+48]
//  240   }
//  241 }
        BX       LR               ;; return
//  242 
//  243 /**
//  244   * @brief  Enables or disables the specified UART interrupts.
//  245   * @param  UARTx: Select the UART peripheral.
//  246   *         This parameter can be one of the following values:
//  247   *         UART1, UART2.
//  248   * @param  UART_IT: specifies the UART interrupt sources to be enabled or disabled.
//  249   *         This parameter can be one of the following values:
//  250   *           @arg UART_IT_OE:  Buffer overflow interrupt (UARTOEINTR).
//  251   *           @arg UART_IT_BE:  Line break interrupt (UARTBEINTR).
//  252   *           @arg UART_IT_PE:  Parity error interrupt (UARTPEINTR).
//  253   *           @arg UART_IT_FE:  Frame structure error interrupt (UARTFEINTR).
//  254   *           @arg UART_IT_RT:  Data input timeout interrupt (UARTRTINTR).
//  255   *           @arg UART_IT_TX:  Transmitter interrupt (UARTTXINTR).
//  256   *           @arg UART_IT_RX:  Receiver interrupt (UARTRXINTR).
//  257   *           @arg UART_IT_DSR: Line nUARTDSR change interrupt (UARTDSRINTR).
//  258   *           @arg UART_IT_DCD: Line nUARTDCD change interrupt (UARTDCDINTR).
//  259   *           @arg UART_IT_CTS: Line nUARTCTS change interrupt (UARTCTSINTR).
//  260   *           @arg UART_IT_RI:  Line nUARTRI change interrupt (UARTRIINTR).
//  261   * @param  NewState: new state of the specified UARTx interrupts.
//  262   *         This parameter can be: ENABLE or DISABLE.
//  263   * @retval None
//  264   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  265 void UART_ITConfig(MDR_UART_TypeDef* UARTx, uint32_t UART_IT, FunctionalState NewState)
//  266 {
//  267   /* Check the parameters */
//  268   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  269   assert_param(IS_UART_CONFIG_IT(UART_IT));
//  270   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  271 
//  272   if (NewState != DISABLE)
UART_ITConfig:
        CMP      R2,#+0
        LDR      R2,[R0, #+56]
        ITE      NE 
        ORRNE    R1,R1,R2
        BICEQ    R1,R2,R1
//  273   {
//  274     UARTx->IMSC |= UART_IT;
//  275   }
//  276   else
//  277   {
//  278     UARTx->IMSC &= ~UART_IT;
        STR      R1,[R0, #+56]
//  279   }
//  280 }
        BX       LR               ;; return
//  281 
//  282 /**
//  283   * @brief  Checks whether the specified UART interrupt has occurred or not.
//  284   * @param  UARTx: Select the UART peripheral.
//  285   *         This parameter can be one of the following values:
//  286   *         UART1, UART2.
//  287   * @param  UART_IT: specifies the UART interrupt source to check.
//  288   *         This parameter can be one of the following values:
//  289   *           @arg UART_IT_OE:  Buffer overflow interrupt (UARTOEINTR).
//  290   *           @arg UART_IT_BE:  Line break interrupt (UARTBEINTR).
//  291   *           @arg UART_IT_PE:  Parity error interrupt (UARTPEINTR).
//  292   *           @arg UART_IT_FE:  Frame structure error interrupt (UARTFEINTR).
//  293   *           @arg UART_IT_RT:  Data input timeout interrupt (UARTRTINTR).
//  294   *           @arg UART_IT_TX:  Transmitter interrupt (UARTTXINTR).
//  295   *           @arg UART_IT_RX:  Receiver interrupt (UARTRXINTR).
//  296   *           @arg UART_IT_DSR: Line nUARTDSR change interrupt (UARTDSRINTR).
//  297   *           @arg UART_IT_DCD: Line nUARTDCD change interrupt (UARTDCDINTR).
//  298   *           @arg UART_IT_CTS: Line nUARTCTS change interrupt (UARTCTSINTR).
//  299   *           @arg UART_IT_RI:  Line nUARTRI change interrupt (UARTRIINTR).
//  300   * @retval The new state of UART_IT (SET or RESET).
//  301   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  302 ITStatus UART_GetITStatus(MDR_UART_TypeDef* UARTx, uint32_t UART_IT)
//  303 {
//  304   ITStatus bitstatus;
//  305 
//  306   /* Check the parameters */
//  307   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  308   assert_param(IS_UART_CONFIG_IT(UART_IT));
//  309 
//  310   if ((UARTx->RIS & UART_IT) == UART_IT)
UART_GetITStatus:
        LDR      R0,[R0, #+60]
        ANDS     R0,R1,R0
        CMP      R0,R1
        BNE.N    ??UART_GetITStatus_0
//  311   {
//  312     bitstatus = SET;
        MOVS     R0,#+1
        BX       LR
//  313   }
//  314   else
//  315   {
//  316     bitstatus = RESET;
??UART_GetITStatus_0:
        MOVS     R0,#+0
//  317   }
//  318 
//  319   return bitstatus;
        BX       LR               ;; return
//  320 }
//  321 
//  322 /**
//  323   * @brief  Checks whether the specified UART interrupt (masked) has occurred or not.
//  324   * @param  UARTx: Select the UART peripheral.
//  325   *         This parameter can be one of the following values:
//  326   *         UART1, UART2.
//  327   * @param  UART_IT: specifies the UART interrupt source to check.
//  328   *         This parameter can be one of the following values:
//  329   *           @arg UART_IT_OE:  Buffer overflow interrupt (UARTOEINTR).
//  330   *           @arg UART_IT_BE:  Line break interrupt (UARTBEINTR).
//  331   *           @arg UART_IT_PE:  Parity error interrupt (UARTPEINTR).
//  332   *           @arg UART_IT_FE:  Frame structure error interrupt (UARTFEINTR).
//  333   *           @arg UART_IT_RT:  Data input timeout interrupt (UARTRTINTR).
//  334   *           @arg UART_IT_TX:  Transmitter interrupt (UARTTXINTR).
//  335   *           @arg UART_IT_RX:  Receiver interrupt (UARTRXINTR).
//  336   *           @arg UART_IT_DSR: Line nUARTDSR change interrupt (UARTDSRINTR).
//  337   *           @arg UART_IT_DCD: Line nUARTDCD change interrupt (UARTDCDINTR).
//  338   *           @arg UART_IT_CTS: Line nUARTCTS change interrupt (UARTCTSINTR).
//  339   *           @arg UART_IT_RI:  Line nUARTRI change interrupt (UARTRIINTR).
//  340   * @retval The new state of UART_IT (SET or RESET).
//  341   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  342 ITStatus UART_GetITStatusMasked(MDR_UART_TypeDef* UARTx, uint32_t UART_IT)
//  343 {
//  344   ITStatus bitstatus;
//  345 
//  346   /* Check the parameters */
//  347   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  348   assert_param(IS_UART_CONFIG_IT(UART_IT));
//  349 
//  350   if (UARTx->MIS & UART_IT)
UART_GetITStatusMasked:
        LDR      R0,[R0, #+64]
        ANDS     R0,R1,R0
        IT       NE 
        MOVNE    R0,#+1
//  351   {
//  352     bitstatus = SET;
//  353   }
//  354   else
//  355   {
//  356     bitstatus = RESET;
//  357   }
//  358 
//  359   return bitstatus;
        UXTB     R0,R0
        BX       LR               ;; return
//  360 }
//  361 
//  362 /**
//  363   * @brief  Clears the UARTx�s interrupt pending bits.
//  364   * @param  UARTx: Select the UART or the UART peripheral.
//  365   *         This parameter can be one of the following values:
//  366   *         UART1, UART2.
//  367   * @param  UART_IT: specifies the interrupt pending bit to clear.
//  368   *         This parameter can be one of the following values:
//  369   *           @arg UART_IT_OE:  Buffer overflow interrupt (UARTOEINTR).
//  370   *           @arg UART_IT_BE:  Line break interrupt (UARTBEINTR).
//  371   *           @arg UART_IT_PE:  Parity error interrupt (UARTPEINTR).
//  372   *           @arg UART_IT_FE:  Frame structure error interrupt (UARTFEINTR).
//  373   *           @arg UART_IT_RT:  Data input timeout interrupt (UARTRTINTR).
//  374   *           @arg UART_IT_TX:  Transmitter interrupt (UARTTXINTR).
//  375   *           @arg UART_IT_RX:  Receiver interrupt (UARTRXINTR).
//  376   *           @arg UART_IT_DSR: Line nUARTDSR change interrupt (UARTDSRINTR).
//  377   *           @arg UART_IT_DCD: Line nUARTDCD change interrupt (UARTDCDINTR).
//  378   *           @arg UART_IT_CTS: Line nUARTCTS change interrupt (UARTCTSINTR).
//  379   *           @arg UART_IT_RI:  Line nUARTRI change interrupt (UARTRIINTR).
//  380   * @retval None
//  381   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  382 void UART_ClearITPendingBit(MDR_UART_TypeDef* UARTx, uint32_t UART_IT)
//  383 {
//  384   /* Check the parameters */
//  385   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  386   assert_param(IS_UART_CONFIG_IT(UART_IT));
//  387 
//  388   UARTx->ICR |= UART_IT;
UART_ClearITPendingBit:
        LDR      R2,[R0, #+68]
        ORRS     R1,R1,R2
        STR      R1,[R0, #+68]
//  389 }
        BX       LR               ;; return
//  390 
//  391 /**
//  392   * @brief  Specified the UART DMA buffer interrupt level.
//  393   * @param  UARTx: Select the UART peripheral.
//  394   *         This parameter can be one of the following values:
//  395   *         UART1, UART2.
//  396   * @param  UART_IT_RB_LVL: specifies the receiver buffer.
//  397   *         This parameter can be one of the following values:
//  398   *           @arg UART_IT_FIFO_LVL_2words:  Interrupt on 1/8 buffer filled
//  399   *           @arg UART_IT_FIFO_LVL_4words:  Interrupt on 1/4 buffer filled
//  400   *           @arg UART_IT_FIFO_LVL_8words:  Interrupt on 1/2 buffer filled
//  401   *           @arg UART_IT_FIFO_LVL_12words: Interrupt on 3/4 buffer filled
//  402   *           @arg UART_IT_FIFO_LVL_14words: Interrupt on 7/8 buffer filled
//  403   * @param  UART_IT_TB_LVL: specifies the transmitter buffer.
//  404   *         This parameter can be one of the following values:
//  405   *           @arg UART_IT_FIFO_LVL_2words:  Interrupt on 1/8 buffer filled
//  406   *           @arg UART_IT_FIFO_LVL_4words:  Interrupt on 1/4 buffer filled
//  407   *           @arg UART_IT_FIFO_LVL_8words:  Interrupt on 1/2 buffer filled
//  408   *           @arg UART_IT_FIFO_LVL_12words: Interrupt on 3/4 buffer filled
//  409   *           @arg UART_IT_FIFO_LVL_14words: Interrupt on 7/8 buffer filled
//  410   * @retval None
//  411   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  412 void UART_DMAConfig(MDR_UART_TypeDef* UARTx, uint32_t UART_IT_RB_LVL, uint32_t UART_IT_TB_LVL)
//  413 {
//  414   /* Check the parameters */
//  415   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  416   assert_param(IS_UART_IT_FIFO_LVL(UART_IT_RB_LVL));
//  417   assert_param(IS_UART_IT_FIFO_LVL(UART_IT_TB_LVL));
//  418 
//  419   UARTx->IFLS = (UART_IT_RB_LVL << 3) | UART_IT_TB_LVL;
UART_DMAConfig:
        ORR      R1,R2,R1, LSL #+3
        STR      R1,[R0, #+52]
//  420 }
        BX       LR               ;; return
//  421 
//  422 /**
//  423   * @brief  Enables or disables the UART�s DMA interface.
//  424   * @param  UARTx: Select the UART peripheral.
//  425   *         This parameter can be one of the following values:
//  426   *         UART1, UART2.
//  427   * @param  UART_DMAReq: specifies the DMA request.
//  428   *         This parameter can be any combination of the following values:
//  429   *           @arg UART_DMA_RXE: UART DMA receive request
//  430   *           @arg UART_DMA_TXE: UART DMA transmit request
//  431   *           @arg UART_DMA_ONERR: UART DMA blocking transmit with error
//  432   * @param  NewState: new state of the DMA Request sources.
//  433   *         This parameter can be: ENABLE or DISABLE.
//  434   * @note The DMA mode is not available for UART5.
//  435   * @retval None
//  436   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  437 void UART_DMACmd(MDR_UART_TypeDef* UARTx, uint32_t UART_DMAReq, FunctionalState NewState)
//  438 {
//  439   /* Check the parameters */
//  440   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  441   assert_param(IS_UART_DMAREQ(UART_DMAReq));
//  442   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  443 
//  444   if (NewState != DISABLE)
UART_DMACmd:
        CMP      R2,#+0
        LDR      R2,[R0, #+72]
        ITEE     NE 
        ORRNE    R1,R1,R2
        MVNEQ    R1,R1
        ANDEQ    R1,R1,R2
//  445   {
//  446     /* Enable the DMA transfer for selected requests and DMAONERR bit
//  447        in the UART DMACR register */
//  448     UARTx->DMACR |= UART_DMAReq;
//  449   }
//  450   else
//  451   {
//  452     /* Disable the DMA transfer for selected requests and DMAONERR bit
//  453        in the UART DMACR register */
//  454     UARTx->DMACR &= (uint16_t)~UART_DMAReq;
        STR      R1,[R0, #+72]
//  455   }
//  456 }
        BX       LR               ;; return
//  457 
//  458 /**
//  459   * @brief  Transmits single data through the UARTx peripheral.
//  460   * @param  UARTx: Select the UART peripheral.
//  461   *         This parameter can be one of the following values:
//  462   *         UART1, UART2.
//  463   * @param  Data: the data to transmit.
//  464   * @retval None
//  465   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  466 void UART_SendData(MDR_UART_TypeDef* UARTx, uint16_t Data)
//  467 {
//  468   /* Check the parameters */
//  469   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  470   assert_param(IS_UART_DATA(Data));
//  471 
//  472   /* Transmit Data */
//  473   UARTx->DR = (Data & (uint16_t)0x0FF);
UART_SendData:
        UXTB     R1,R1
        STR      R1,[R0, #+0]
//  474 }
        BX       LR               ;; return
//  475 
//  476 /**
//  477   * @brief  Returns the most recent received data by the UARTx peripheral.
//  478   * @param  UARTx: Select the UART peripheral.
//  479   *         This parameter can be one of the following values:
//  480   *         UART1, UART2.
//  481   * @retval The received data (7:0) and error flags (15:8).
//  482   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  483 uint16_t UART_ReceiveData(MDR_UART_TypeDef* UARTx)
//  484 {
//  485   /* Check the parameters */
//  486   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  487 
//  488   /* Receive Data */
//  489   return (uint16_t)(UARTx->DR);
UART_ReceiveData:
        LDR      R0,[R0, #+0]
        UXTH     R0,R0
        BX       LR               ;; return
//  490 }
//  491 
//  492 /**
//  493   * @brief  Break transmit.
//  494   * @param  UARTx: Select the UART peripheral.
//  495   *         This parameter can be one of the following values:
//  496   *         UART1, UART2.
//  497   * @param  NewState: new state of the Line.
//  498   *         This parameter can be: ENABLE or DISABLE.
//  499   * @retval None
//  500   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  501 void UART_BreakLine(MDR_UART_TypeDef* UARTx, FunctionalState NewState)
//  502 {
//  503   /* Check the parameters */
//  504   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  505   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  506 
//  507   if (NewState != DISABLE)
UART_BreakLine:
        CMP      R1,#+0
        LDR      R1,[R0, #+44]
        ITEE     NE 
        ORRNE    R1,R1,#0x1
        ORREQ    R1,R1,#0xFF00
        ORREQ    R1,R1,#0xFE
//  508   {
//  509     /* Set BRK bit in the UART LCR_H register */
//  510     UARTx->LCR_H |= LCR_H_BRK_Set;
//  511   }
//  512   else
//  513   {
//  514     /* Reset BRK bit in the UART LCR_H register */
//  515     UARTx->LCR_H |= LCR_H_BRK_Reset;
        STR      R1,[R0, #+44]
//  516   }
//  517 }
        BX       LR               ;; return
//  518 
//  519 /**
//  520   * @brief  Configures the UART�s IrDA interface.
//  521   * @param  UARTx: Select the UART peripheral.
//  522   *         This parameter can be one of the following values:
//  523   *         UART1, UART2.
//  524   * @param  UART_IrDAMode: specifies the IrDA mode.
//  525   *         This parameter can be one of the following values:
//  526   *           @arg UART_IrDAMode_LowPower
//  527   *           @arg UART_IrDAMode_Normal
//  528   * @retval None
//  529   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  530 void UART_IrDAConfig(MDR_UART_TypeDef* UARTx, uint32_t UART_IrDAMode)
//  531 {
UART_IrDAConfig:
        PUSH     {R4,R5,LR}
        SUB      SP,SP,#+20
        MOV      R4,R0
        MOV      R5,R1
//  532   uint32_t cpuclock;
//  533   RST_CLK_FreqTypeDef RST_CLK_Clocks;
//  534 
//  535   /* Check the parameters */
//  536   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  537   assert_param(IS_UART_IRDA_MODE(UART_IrDAMode));
//  538 
//  539   if (UART_IrDAMode == UART_IrDAMode_LowPower)
        CMP      R5,#+4
        BNE.N    ??UART_IrDAConfig_0
//  540   {
//  541     /* Configure the UART ILPR */
//  542     RST_CLK_GetClocksFreq(&RST_CLK_Clocks);
        ADD      R0,SP,#+0
        BL       RST_CLK_GetClocksFreq
//  543     cpuclock = RST_CLK_Clocks.CPU_CLK_Frequency;
//  544     UARTx->ILPR = cpuclock / IrLPBaud16;
        LDR      R0,[SP, #+0]
        MOV      R1,#+1843200
        UDIV     R0,R0,R1
        STR      R0,[R4, #+32]
//  545   }
//  546   UARTx->CR |= UART_IrDAMode;
??UART_IrDAConfig_0:
        LDR      R0,[R4, #+48]
        ORRS     R0,R5,R0
        STR      R0,[R4, #+48]
//  547 }
        ADD      SP,SP,#+20
        POP      {R4,R5,PC}       ;; return
//  548 
//  549 /**
//  550   * @brief  Enables or disables the UART�s IrDA interface.
//  551   * @param  UARTx: Select the UART peripheral.
//  552   *         This parameter can be one of the following values:
//  553   *         UART1, UART2.
//  554   * @param  NewState: new state of the IrDA mode.
//  555   *         This parameter can be: ENABLE or DISABLE.
//  556   * @retval None
//  557   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  558 void UART_IrDACmd(MDR_UART_TypeDef* UARTx, FunctionalState NewState)
//  559 {
//  560   /* Check the parameters */
//  561   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  562   assert_param(IS_FUNCTIONAL_STATE(NewState));
//  563 
//  564   if (NewState != DISABLE)
UART_IrDACmd:
        CMP      R1,#+0
        LDR      R1,[R0, #+48]
        ITEE     NE 
        ORRNE    R1,R1,#0x2
        ORREQ    R1,R1,#0xFF00
        ORREQ    R1,R1,#0xFD
//  565   {
//  566     /* Set SIREN bit in the UART CR register */
//  567     UARTx->CR |= CR_SIREN_Set;
//  568   }
//  569   else
//  570   {
//  571     /* Reset SIREN bit in the UART CR register */
//  572     UARTx->CR |= CR_SIREN_Reset;
        STR      R1,[R0, #+48]
//  573   }
//  574 }
        BX       LR               ;; return
//  575 
//  576 /**
//  577   * @brief  Checks whether the specified UART flag is set or not.
//  578   * @param  UARTx: Select the UART or the UART peripheral.
//  579   *         This parameter can be one of the following values:
//  580   *         UART1, UART2, UART3, UART4 or UART5.
//  581   * @param  UART_FLAG: specifies the flag to check.
//  582   *         This parameter can be one of the following values:
//  583   *           @arg UART_FLAG_RI  : nUARTRI line inverted value
//  584   *           @arg UART_FLAG_TXFE: Transmit buffer is empty flag
//  585   *           @arg UART_FLAG_RXFF: Receive buffer is full flag
//  586   *           @arg UART_FLAG_TXFF: Transmit buffer is full flag
//  587   *           @arg UART_FLAG_RXFE: Receive buffer is empty flag
//  588   *           @arg UART_FLAG_BUSY: Transmit Busy flag
//  589   *           @arg UART_FLAG_DCD:  nUARTDCD line inverted value
//  590   *           @arg UART_FLAG_DSR:  nUARTDSR line inverted value
//  591   *           @arg UART_FLAG_CTS:  nUARTCTS line inverted value
//  592   * @retval The new state of UART_FLAG (SET or RESET).
//  593   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  594 FlagStatus UART_GetFlagStatus(MDR_UART_TypeDef* UARTx, uint32_t UART_FLAG)
//  595 {
//  596   FlagStatus bitstatus;
//  597 
//  598   /* Check the parameters */
//  599   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  600   assert_param(IS_UART_FLAG(UART_FLAG));
//  601 
//  602   if (UARTx->FR & UART_FLAG)
UART_GetFlagStatus:
        LDR      R0,[R0, #+24]
        ANDS     R0,R1,R0
        IT       NE 
        MOVNE    R0,#+1
//  603   {
//  604     bitstatus = SET;
//  605   }
//  606   else
//  607   {
//  608     bitstatus = RESET;
//  609   }
//  610   return bitstatus;
        UXTB     R0,R0
        BX       LR               ;; return
//  611 }
//  612 
//  613 /**
//  614   * @brief  Initializes the UARTx peripheral Clock according to the
//  615   *         specified parameters.
//  616   * @param  UARTx: Select the UART peripheral.
//  617   *         This parameter can be one of the following values:
//  618   *         UART1, UART2.
//  619   * @param  UART_BRG: specifies the HCLK division factor.
//  620   *         This parameter can be one of the following values:
//  621   *           @arg UART_HCLKdiv1
//  622   *           @arg UART_HCLKdiv2
//  623   *           @arg UART_HCLKdiv4
//  624   *           @arg UART_HCLKdiv8
//  625   *           @arg UART_HCLKdiv16
//  626   *           @arg UART_HCLKdiv32
//  627   *           @arg UART_HCLKdiv64
//  628   *           @arg UART_HCLKdiv128
//  629   * @retval None
//  630   */

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  631 void UART_BRGInit(MDR_UART_TypeDef* UARTx, uint32_t UART_BRG)
//  632 {
UART_BRGInit:
        PUSH     {R4,LR}
//  633   uint32_t tmpreg;
//  634 
//  635   /* Check the parameters */
//  636   assert_param(IS_UART_ALL_PERIPH(UARTx));
//  637   assert_param(IS_UART_CLOCK_BRG(UART_BRG));
//  638 
//  639   tmpreg = MDR_RST_CLK->UART_CLOCK;
        LDR.N    R2,??DataTable1  ;; 0x40020028
        LDR      R3,[R2, #+0]
//  640 
//  641   if (UARTx == MDR_UART1)
        LDR.N    R4,??DataTable1_1  ;; 0x40030000
        CMP      R0,R4
        BNE.N    ??UART_BRGInit_0
//  642   {
//  643     tmpreg |= RST_CLK_UART_CLOCK_UART1_CLK_EN;
//  644     tmpreg &= ~RST_CLK_UART_CLOCK_UART1_BRG_Msk;
//  645     tmpreg |= UART_BRG;
        LSRS     R0,R3,#+8
        ORRS     R0,R1,R0, LSL #+8
        ORR      R3,R0,#0x1000000
        B.N      ??UART_BRGInit_1
//  646   }
//  647   else if (UARTx == MDR_UART2)
??UART_BRGInit_0:
        LDR.N    R4,??DataTable1_2  ;; 0x40038000
        CMP      R0,R4
        BNE.N    ??UART_BRGInit_1
//  648   {
//  649     tmpreg |= RST_CLK_UART_CLOCK_UART2_CLK_EN;
//  650     tmpreg &= ~RST_CLK_UART_CLOCK_UART2_BRG_Msk;
//  651     tmpreg |= (UART_BRG << 8);
        BIC      R0,R3,#0xFF00
        ORR      R0,R0,R1, LSL #+8
        ORR      R3,R0,#0x2000000
//  652   }
//  653   MDR_RST_CLK->UART_CLOCK = tmpreg;
??UART_BRGInit_1:
        STR      R3,[R2, #+0]
//  654 }
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1:
        DC32     0x40020028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_1:
        DC32     0x40030000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_2:
        DC32     0x40038000

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
//  655 
//  656 /** @} */ /* End of group UART_Private_Functions */
//  657 
//  658 /** @} */ /* End of group UART */
//  659 
//  660 /** @} */ /* End of group __MDR32F9Qx_StdPeriph_Driver */
//  661 
//  662 /******************* (C) COPYRIGHT 2010 Phyton *********************************
//  663 *
//  664 * END OF FILE MDR32F9Qx_uart.c */
//  665 
// 
// 454 bytes in section .text
// 
// 454 bytes of CODE memory
//
//Errors: none
//Warnings: none
