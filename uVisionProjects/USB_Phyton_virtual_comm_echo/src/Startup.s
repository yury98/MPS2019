;/*****************************************************************************/
;/* STM32F10x.s: Startup file for ARM Cortex-M1 Device Family                 */
;/*****************************************************************************/
;/* <<< Use Configuration Wizard in Context Menu >>>                          */
;/*****************************************************************************/
;/* This file is part of the uVision/ARM development tools.                   */
;/* Copyright (c) 2005-2008 Keil Software. All rights reserved.               */
;/* This software may only be used under the terms of a valid, current,       */
;/* end user licence from KEIL for a compatible version of KEIL software      */
;/* development tools. Nothing else gives you the right to use this software. */
;/*****************************************************************************/


;// <h> Stack Configuration
;//   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;// </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


;// <h> Heap Configuration
;//   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;// </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     MILSTD1553B1_IRQ_Handler	;IRQ0 MILSTD1553B1 Handler
				DCD     MILSTD1553B2_IRQ_Handler    ;IRQ1 MILSTD1553B2 Handler
				DCD     USB_IRQHandler		        ;IRQ2 USB     Handler
				DCD     CAN1_IRQ_Handler		  	;IRQ3 CAN1    Handler
				DCD     CAN2_IRQ_Handler		  	;IRQ4 CAN2    Handler
				DCD     DMA_IRQ_Handler		  	    ;IRQ5 DMA     Handler
				DCD     UART1_IRQ_Handler		  	;IRQ6 UART1   Handler
				DCD     UART2_IRQ_Handler		  	;IRQ7 UART2   Handler
				DCD     SPI1_IRQ_Handler		  	;IRQ8 SPI1    Handler
				DCD     NAND_IRQ_Handler		  	;IRQ9 NAND    Handler
				DCD     ARINCR_IRQ_Handler		  	;IRQ10 ARINCR  Handler
				DCD     PWR_IRQ_Handler		  		;IRQ11 PWR    Handler
				DCD     WWDG_IRQ_Handler		  	;IRQ12 WWDG   Handler
				DCD     TIM4_IRQ_Handler		  	;IRQ13 TIM4   Handler
				DCD     TIM1_IRQ_Handler		  	;IRQ14 TIM1   Handler
				DCD     TIM2_IRQ_Handler		  	;IRQ15 TIM2   Handler
				DCD     TIM3_IRQ_Handler		  	;IRQ16 TIM3   Handler
				DCD     ADC_IRQ_Handler		  	    ;IRQ17 ADCIU3 Handler
				DCD     ETH_IRQ_Handler		  	    ;IRQ18 Ethernet Handler
				DCD     SPI3_IRQ_Handler		  	;IRQ19 SPI3    Handler
				DCD     SPI2_IRQ_Handler		  	;IRQ20 SPI2    Handler
				DCD     ARINCT1_IRQ_Handler		  	;IRQ21 ARINCT1 Handler
				DCD     ARINCT2_IRQ_Handler		  	;IRQ22 ARINCT2 Handler
				DCD     ARINCT3_IRQ_Handler		  	;IRQ23 ARINCT3 Handler
				DCD     ARINCT4_IRQ_Handler		  	;IRQ24 ARINCT4 Handler
				DCD     Reserved_IRQ_Handler		;IRQ25 Reserved Handler
				DCD     Reserved_IRQ_Handler		;IRQ26 Reserved Handler
				DCD     BKP_IRQ_Handler		  		;IRQ27 BKP     Handler
				DCD     EXT1_IRQ_Handler		  	;IRQ28 EXT1 Handler
				DCD     EXT2_IRQ_Handler		  	;IRQ29 EXT2 Handler
				DCD     EXT3_IRQ_Handler		  	;IRQ30 EXT3 Handler
				DCD     EXT4_IRQ_Handler		  	;IRQ31 EXT4 Handler

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  __main
                LDR     R0,=__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)                

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
				B       .
                ENDP

DMA_IRQ_Handler     PROC
                EXPORT  DMA_IRQ_Handler               [WEAK]
				B       .
                ENDP

UART1_IRQ_Handler     PROC
                EXPORT  UART1_IRQ_Handler               [WEAK]
				B       .
               ENDP

UART2_IRQ_Handler     PROC
                EXPORT  UART2_IRQ_Handler               [WEAK]
				B       .
                ENDP

SPI1_IRQ_Handler     PROC
                EXPORT  SPI1_IRQ_Handler               [WEAK]
				B       .
                ENDP

SPI2_IRQ_Handler     PROC
                EXPORT  SPI2_IRQ_Handler               [WEAK]
				B       .
                ENDP

SPI3_IRQ_Handler     PROC
                EXPORT  SPI3_IRQ_Handler               [WEAK]
				B       .
                ENDP


PWR_IRQ_Handler     PROC
               EXPORT  PWR_IRQ_Handler               [WEAK]
				B       .
                ENDP
				
WWDG_IRQ_Handler     PROC
               EXPORT  WWDG_IRQ_Handler               [WEAK]
				B       .
                ENDP	 

TIM1_IRQ_Handler     PROC
               EXPORT  TIM1_IRQ_Handler               [WEAK]
				B       .
                ENDP

TIM2_IRQ_Handler     PROC
               EXPORT  TIM2_IRQ_Handler               [WEAK]
				B       .
                ENDP

TIM3_IRQ_Handler     PROC
               EXPORT  TIM3_IRQ_Handler               [WEAK]
				B       .
                ENDP

TIM4_IRQ_Handler     PROC
               EXPORT  TIM4_IRQ_Handler               [WEAK]
				B       .
                ENDP

ADC_IRQ_Handler     PROC
               EXPORT  ADC_IRQ_Handler               [WEAK]
				B       .
                ENDP

USB_IRQHandler     PROC
               EXPORT  USB_IRQHandler               [WEAK]
				B       .
                ENDP

BKP_IRQ_Handler     PROC
               EXPORT  BKP_IRQ_Handler               [WEAK]
				B       .
                ENDP

EXT1_IRQ_Handler     PROC
               EXPORT  EXT1_IRQ_Handler               [WEAK]
				B       .
                ENDP

EXT2_IRQ_Handler     PROC
               EXPORT  EXT1_IRQ_Handler               [WEAK]
				B       .
                ENDP

EXT3_IRQ_Handler     PROC
               EXPORT  EXT1_IRQ_Handler               [WEAK]
				B       .
                ENDP

EXT4_IRQ_Handler     PROC
               EXPORT  EXT1_IRQ_Handler               [WEAK]
				B       .
                ENDP

MILSTD1553B1_IRQ_Handler     PROC
               EXPORT  MILSTD1553B1_IRQ_Handler               [WEAK]
				B       .
                ENDP

MILSTD1553B2_IRQ_Handler     PROC
               EXPORT  MILSTD1553B2_IRQ_Handler              [WEAK]
				B       .
                ENDP

CAN1_IRQ_Handler     PROC
               EXPORT  CAN1_IRQ_Handler              [WEAK]
				B       .
                ENDP

CAN2_IRQ_Handler     PROC
               EXPORT  CAN2_IRQ_Handler              [WEAK]
				B       .
                ENDP

NAND_IRQ_Handler     PROC
               EXPORT  NAND_IRQ_Handler              [WEAK]
				B       .
                ENDP

ARINCR_IRQ_Handler     PROC
               EXPORT  ARINCR_IRQ_Handler              [WEAK]
				B       .
                ENDP

ARINCT1_IRQ_Handler     PROC
               EXPORT  ARINCT1_IRQ_Handler              [WEAK]
				B       .
                ENDP

ARINCT2_IRQ_Handler     PROC
               EXPORT  ARINCT2_IRQ_Handler              [WEAK]
				B       .
                ENDP

ARINCT3_IRQ_Handler     PROC
               EXPORT  ARINCT1_IRQ_Handler              [WEAK]
				B       .
                ENDP

ARINCT4_IRQ_Handler     PROC
               EXPORT  ARINCT4_IRQ_Handler              [WEAK]
				B       .
                ENDP

ETH_IRQ_Handler     PROC
               EXPORT  ETH_IRQ_Handler              [WEAK]
				B       .
                ENDP

Default_Handler PROC
                EXPORT   Reserved_IRQ_Handler               [WEAK]
Reserved_IRQ_Handler
                ENDP


                ALIGN



; User Initial Stack & Heap

                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
