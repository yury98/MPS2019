///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.40.2.53884/W32 for ARM    22/Sep/2015  17:26:03 /
// Copyright 1999-2012 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\src\leds.c                                 /
//    Command line =  C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1 /
//                    986VE91_Eval\src\leds.c --preprocess                    /
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
//                    986VE91_Eval\Demo\IAR_Systems\Debug\List\leds.s         /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME leds

        #define SHT_PROGBITS 0x1

        EXTERN PORT_ReadInputData
        EXTERN PORT_Write

        PUBLIC CurrentLights
        PUBLIC ShiftLights

// C:\WORK\Milandr.MDR1986BExx.1.4.0\Example_Projects\MDR1986VE91_Eval\src\leds.c
//    1 /**
//    2   ******************************************************************************
//    3   * @file    leds.c
//    4   * @author  Phyton Application Team
//    5   * @version V3.0.0
//    6   * @date    10.09.2011
//    7   * @brief   This file provides the LEDs utilities.
//    8   ******************************************************************************
//    9   * <br><br>
//   10   * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//   11   * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
//   12   * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY
//   13   * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
//   14   * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
//   15   * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//   16   *
//   17   * <h2><center>&copy; COPYRIGHT 2011 Phyton</center></h2>
//   18   */
//   19 
//   20 /* Includes ------------------------------------------------------------------*/
//   21 #include <MDR32F9Qx_port.h>
//   22 #include "leds.h"
//   23 
//   24 /** @addtogroup __MDR32F9Qx_Eval_Demo MDR32F9Qx Demonstration Example
//   25   * @{
//   26   */
//   27 
//   28 /** @addtogroup LEDs_Control LEDs Control
//   29   * @{
//   30   */
//   31 
//   32 /** @defgroup LEDs_Variables LEDs Variables
//   33   * @{
//   34   */
//   35 

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   36 uint32_t CurrentLights;         /*!< LEDs on mask */
CurrentLights:
        DS8 4
//   37 
//   38 /** @} */ /* End of group LEDs_Variables */
//   39 
//   40 /** @defgroup LEDs_Private_Functions LEDs Private Functions
//   41   * @{
//   42   */
//   43 
//   44 /*******************************************************************************
//   45 * Function Name  : ShiftLights
//   46 * Description    : Controls LEDs on/off.
//   47 * Input          : None
//   48 * Output         : None
//   49 * Return         : None
//   50 *******************************************************************************/

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
//   51 void ShiftLights(void)
//   52 {
ShiftLights:
        PUSH     {R3-R5,LR}
//   53   uint32_t ovf;
//   54   uint32_t portdata;
//   55 
//   56   portdata = PORT_ReadInputData(LEDs_PORT);
        LDR.N    R4,??ShiftLights_0  ;; 0x400c0000
        MOV      R0,R4
        BL       PORT_ReadInputData
//   57   PORT_Write(LEDs_PORT, (portdata & ~LEDs_PINs) | (CurrentLights & LEDs_PINs));
        LDR.N    R5,??ShiftLights_0+0x4
        BIC      R0,R0,#0x7C00
        LDR      R1,[R5, #+0]
        AND      R1,R1,#0x7C00
        ORRS     R1,R1,R0
        MOV      R0,R4
        BL       PORT_Write
//   58   ovf = (CurrentLights & (1UL << 31)) != 0;
//   59   CurrentLights <<= 1;
//   60   CurrentLights |= ovf;
        LDR      R0,[R5, #+0]
        ROR      R0,R0,#+31
        STR      R0,[R5, #+0]
//   61 }
        POP      {R0,R4,R5,PC}    ;; return
        DATA
??ShiftLights_0:
        DC32     0x400c0000
        DC32     CurrentLights

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
//   62 
//   63 /** @} */ /* End of group LEDs_Private_Functions */
//   64 
//   65 /** @} */ /* End of group LEDs_Control */
//   66 
//   67 /** @} */ /* End of group __MDR32F9Qx_Eval_Demo */
//   68 
//   69 /******************* (C) COPYRIGHT 2011 Phyton *********************************
//   70 *
//   71 * END OF FILE leds.c */
//   72 
// 
//  4 bytes in section .bss
// 48 bytes in section .text
// 
// 48 bytes of CODE memory
//  4 bytes of DATA memory
//
//Errors: none
//Warnings: none
