/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_can.h"              // Keil::Drivers:CAN
#include "1986BE9x_it.h"
#include "mlt_lcd.h"
#include "font.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
int extern count_title;
int extern MenuMainItem;
int extern CursorPosItem;
char extern UartFlag;
char extern TestCanDone;
char extern CanFlag;
char extern TestUartDone;
char extern USBFlag;
char extern SSPFlag;
char extern TestSSPDone;
char extern FullTestFlag;
char extern FullTestCANFlag;
char extern FullTestCANGetFlag;
char extern FullTestUARTFlag;
char extern FullTestSSPFlag;
char extern FullTestSSPGetFlag;
uint8_t extern USBInfo;
int count_main = 0;

__IO uint32_t extern rx_buf;

uint8_t* main_string[]  = 
{
	 cyr_K,  cyr_u,  cyr_r,  cyr_s,  cyr_o,  cyr_v, cyr_a,  cyr_ya, sym_sp,
	 cyr_r,  cyr_a,  cyr_b,  cyr_o,  cyr_t,  cyr_a,  sym_sp, cyr_p, cyr_o,  sym_sp,
	 cyr_k,  cyr_u,  cyr_r,  cyr_s,  cyr_u,  sym_sp,  cyr_M,  cyr_i,  cyr_k,  cyr_r,  cyr_o,
	 cyr_p,  cyr_r,  cyr_o,  cyr_c,  cyr_e,  cyr_s,  cyr_s,  cyr_o,  cyr_r,  cyr_n,
	 cyr_y,  cyr_e,  sym_sp,  cyr_s,  cyr_i,  cyr_s,  cyr_t,  cyr_e,  cyr_m,  cyr_y
};

uint8_t* menu_string[] =
{
	sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,lat_C,lat_A,lat_N,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,
	sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,lat_S,lat_S,lat_P,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,
	sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,lat_U,lat_S,lat_B,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,
	sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,lat_U,lat_A,lat_R,lat_T,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,
	sym_sp,sym_sp,sym_sp,sym_sp,lat_F,lat_U,lat_L,lat_L,sym_sp,lat_T,lat_E,lat_S,lat_T,sym_sp,sym_sp,sym_sp
};

uint8_t* numbers_string[] =
{
	dig_0,dig_0,dig_0,dig_1,dig_0,dig_2,dig_0,dig_3,dig_0,dig_4,dig_0,dig_5,dig_0,dig_6,dig_0,dig_7,dig_0,dig_8,dig_0,dig_9,dig_0,lat_A,dig_0,lat_B,dig_0,lat_C,dig_0,lat_D,dig_0,lat_E,dig_0,lat_F,
	dig_1,dig_0,dig_1,dig_1,dig_1,dig_2,dig_1,dig_3,dig_1,dig_4,dig_1,dig_5,dig_1,dig_6,dig_1,dig_7,dig_1,dig_8,dig_1,dig_9,dig_1,lat_A,dig_1,lat_B,dig_1,lat_C,dig_1,lat_D,dig_1,lat_E,dig_1,lat_F,
	dig_2,dig_0,dig_2,dig_1,dig_2,dig_2,dig_2,dig_3,dig_2,dig_4,dig_2,dig_5,dig_2,dig_6,dig_2,dig_7,dig_2,dig_8,dig_2,dig_9,dig_2,lat_A,dig_2,lat_B,dig_2,lat_C,dig_2,lat_D,dig_2,lat_E,dig_2,lat_F,
	dig_3,dig_0,dig_3,dig_1,dig_3,dig_2,dig_3,dig_3,dig_3,dig_4,dig_3,dig_5,dig_3,dig_6,dig_3,dig_7,dig_3,dig_8,dig_3,dig_9,dig_3,lat_A,dig_3,lat_B,dig_3,lat_C,dig_3,lat_D,dig_3,lat_E,dig_3,lat_F,
	dig_4,dig_0,dig_4,dig_1,dig_4,dig_2,dig_4,dig_3,dig_4,dig_4,dig_4,dig_5,dig_4,dig_6,dig_4,dig_7,dig_4,dig_8,dig_4,dig_9,dig_4,lat_A,dig_4,lat_B,dig_4,lat_C,dig_4,lat_D,dig_4,lat_E,dig_4,lat_F,
	dig_5,dig_0,dig_5,dig_1,dig_5,dig_2,dig_5,dig_3,dig_5,dig_4,dig_5,dig_5,dig_5,dig_6,dig_5,dig_7,dig_5,dig_8,dig_5,dig_9,dig_5,lat_A,dig_5,lat_B,dig_5,lat_C,dig_5,lat_D,dig_5,lat_E,dig_5,lat_F,
	dig_6,dig_0,dig_6,dig_1,dig_6,dig_2,dig_6,dig_3,dig_6,dig_4,dig_6,dig_5,dig_6,dig_6,dig_6,dig_7,dig_6,dig_8,dig_6,dig_9,dig_6,lat_A,dig_6,lat_B,dig_6,lat_C,dig_6,lat_D,dig_6,lat_E,dig_6,lat_F,
	dig_7,dig_0,dig_7,dig_1,dig_7,dig_2,dig_7,dig_3,dig_7,dig_4,dig_7,dig_5,dig_7,dig_6,dig_7,dig_7,dig_7,dig_8,dig_7,dig_9,dig_7,lat_A,dig_7,lat_B,dig_7,lat_C,dig_7,lat_D,dig_7,lat_E,dig_7,lat_F,
	dig_8,dig_0,dig_8,dig_1,dig_8,dig_2,dig_8,dig_3,dig_8,dig_4,dig_8,dig_5,dig_8,dig_6,dig_8,dig_7,dig_8,dig_8,dig_8,dig_9,dig_8,lat_A,dig_8,lat_B,dig_8,lat_C,dig_8,lat_D,dig_8,lat_E,dig_8,lat_F,
	dig_9,dig_0,dig_9,dig_1,dig_9,dig_2,dig_9,dig_3,dig_9,dig_4,dig_9,dig_5,dig_9,dig_6,dig_9,dig_7,dig_9,dig_8,dig_9,dig_9,dig_9,lat_A,dig_9,lat_B,dig_9,lat_C,dig_9,lat_D,dig_9,lat_E,dig_9,lat_F,
	lat_A,dig_0,lat_A,dig_1,lat_A,dig_2,lat_A,dig_3,lat_A,dig_4,lat_A,dig_5,lat_A,dig_6,lat_A,dig_7,lat_A,dig_8,lat_A,dig_9,lat_A,lat_A,lat_A,lat_B,lat_A,lat_C,lat_A,lat_D,lat_A,lat_E,lat_A,lat_F,
	lat_B,dig_0,lat_B,dig_1,lat_B,dig_2,lat_B,dig_3,lat_B,dig_4,lat_B,dig_5,lat_B,dig_6,lat_B,dig_7,lat_B,dig_8,lat_B,dig_9,lat_B,lat_A,lat_B,lat_B,lat_B,lat_C,lat_B,lat_D,lat_B,lat_E,lat_B,lat_F,
	lat_C,dig_0,lat_C,dig_1,lat_C,dig_2,lat_C,dig_3,lat_C,dig_4,lat_C,dig_5,lat_C,dig_6,lat_C,dig_7,lat_C,dig_8,lat_C,dig_9,lat_C,lat_A,lat_C,lat_B,lat_C,lat_C,lat_C,lat_D,lat_C,lat_E,lat_C,lat_F,
	lat_D,dig_0,lat_D,dig_1,lat_D,dig_2,lat_D,dig_3,lat_D,dig_4,lat_D,dig_5,lat_D,dig_6,lat_D,dig_7,lat_D,dig_8,lat_D,dig_9,lat_D,lat_A,lat_D,lat_B,lat_D,lat_C,lat_D,lat_D,lat_D,lat_E,lat_D,lat_F,
	lat_E,dig_0,lat_E,dig_1,lat_E,dig_2,lat_E,dig_3,lat_E,dig_4,lat_E,dig_5,lat_E,dig_6,lat_E,dig_7,lat_E,dig_8,lat_E,dig_9,lat_E,lat_A,lat_E,lat_B,lat_E,lat_C,lat_E,lat_D,lat_E,lat_E,lat_E,lat_F,
	lat_F,dig_0,lat_F,dig_1,lat_F,dig_2,lat_F,dig_3,lat_F,dig_4,lat_F,dig_5,lat_F,dig_6,lat_F,dig_7,lat_F,dig_8,lat_F,dig_9,lat_F,lat_A,lat_F,lat_B,lat_F,lat_C,lat_F,lat_D,lat_F,lat_E,lat_F,lat_F
};

uint8_t* usb_string1[]  =
{
	sym_sp,sym_sp,sym_sp,sym_sp,cyr_T,cyr_e,cyr_s,cyr_t,sym_sp,lat_U,lat_S,lat_B,sym_sp,sym_sp,sym_sp,sym_sp
};
uint8_t* usb_string2[]  =
{
	sym_sp,cyr_p,cyr_r,cyr_o,cyr_v,cyr_o,cyr_d,cyr_i,cyr_t,cyr_s,cyr_ya,sym_sp,cyr_n,cyr_a,sym_sp,sym_sp
};
uint8_t* usb_string3[]  =
{
	sym_sp,sym_sp,cyr_p,cyr_l,cyr_a,cyr_t,cyr_e,sym_sp,lat_m,lat_a,lat_s,lat_t,lat_e,lat_r,sym_sp,sym_sp
};

uint8_t* uart_string[]  =
{
	sym_sp,sym_sp,sym_sp,cyr_T,cyr_e,cyr_s,cyr_t,sym_sp,lat_U,lat_A,lat_R,lat_T,sym_sp,sym_sp,sym_sp,sym_sp
};
uint8_t* ssp_string[]  =
{
	sym_sp,sym_sp,sym_sp,sym_sp,cyr_T,cyr_e,cyr_s,cyr_t,sym_sp,lat_S,lat_S,lat_P,sym_sp,sym_sp,sym_sp,sym_sp
};
uint8_t* pass_string[]  =
{
	cyr_D,cyr_a,cyr_n,cyr_n,cyr_y,cyr_e,sym_sp,cyr_p,cyr_e,cyr_r,cyr_e,cyr_d,cyr_a,cyr_n,cyr_y,sym_sp
};
uint8_t* get_string[]  =
{
	cyr_D,cyr_a,cyr_n,cyr_n,cyr_y,cyr_e,sym_sp,cyr_p,cyr_o,cyr_l,cyr_u,cyr_ch,cyr_e,cyr_n,cyr_y,sym_sp
};
uint8_t* wait_string[]  =
{
	cyr_O,cyr_zh,cyr_i,cyr_d,cyr_a,cyr_n,cyr_i,cyr_e,sym_sp,cyr_d,cyr_a,cyr_n,cyr_n,cyr_y,cyr_kh,sym_sp
};
uint8_t* fail_string[]  =
{
	cyr_D,cyr_a,cyr_n,cyr_n,cyr_y,cyr_e,sym_sp,cyr_o,cyr_sh,cyr_i,cyr_b,cyr_o,cyr_ch,cyr_n,cyr_y,sym_sp
};




uint8_t* can_string[]  =
{
	sym_sp,sym_sp,sym_sp,sym_sp,cyr_T,cyr_e,cyr_s,cyr_t,sym_sp,lat_C,lat_A,lat_N,sym_sp,sym_sp,sym_sp,sym_sp
};


uint8_t* empty_string[]  =
{
	sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp
};
uint8_t* info_string[]  =
{
	sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp,sym_sp
};

uint8_t* InfoString(uint8_t* RecievedData)
{
	int value = RecievedData;
	info_string[7] = numbers_string[value * 2];
	info_string[8] = numbers_string[value * 2 + 1];
	return info_string;
}


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{
}
/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}
/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
/*******************************************************************************
* Function Name  : SVC_Handler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVC_Handler(void)
{
}
/*******************************************************************************
* Function Name  : DebugMon_Handler
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMon_Handler(void)
{
}
/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles Debug PendSV exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSV_Handler(void)
{
}
/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
}
/*******************************************************************************
* Function Name  : CAN1_IRQHandler
* Description    : This function handles CAN1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN1_IRQHandler(void)
{
	CAN_RxMsgTypeDef RxMessage;

  CAN_GetRawReceivedData(MDR_CAN1, rx_buf, &RxMessage);
	if (FullTestFlag == 0)
	{
		if((RxMessage.Rx_Header.ID==0x15555555) && (RxMessage.Rx_Header.IDE==CAN_ID_EXT)
			 && (RxMessage.Rx_Header.DLC==4) && (RxMessage.Data[0]==0x12345678))
		{
			TestCanDone = 1;
		}
		else
		{
			TestCanDone = 2;
		}
	}
	else 
	{
		USBInfo = RxMessage.Data[0];
		FullTestCANGetFlag = 1;
		
	}
  CAN_ITClearRxTxPendingBit(MDR_CAN1, rx_buf, CAN_STATUS_RX_READY);
}
/*******************************************************************************
* Function Name  : CAN2_IRQHandler
* Description    : This function handles CAN2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN2_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void USB_IRQHandler(void)
{
}*/
/*******************************************************************************
* Function Name  : DMA_IRQHandler
* Description    : This function handles DMA global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : UART1_IRQHandler
* Description    : This function handles UART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART1_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : UART2_IRQHandler
* Description    : This function handles UART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART2_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : SSP1_IRQHandler
* Description    : This function handles SSP1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SSP1_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : I2C_IRQHandler
* Description    : This function handles I2C global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : POWER_IRQHandler
* Description    : This function handles POWER global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void POWER_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : WWDG_IRQHandler
* Description    : This function handles WWDG global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_IRQHandler(void)
{
}

/*******************************************************************************
* Function Name  : Timer1_IRQHandler
* Description    : This function handles Timer1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Timer1_IRQHandler(void)
{
int i;
	MDR_TIMER1->STATUS = 0;

	if (count_title == 0)
	{
		count_title = 1;

		LcdPutChar (cyr_SH, 0, 0);  LcdPutChar (cyr_a, 1, 0);  LcdPutChar (cyr_sh, 2, 0);
		LcdPutChar (cyr_k, 3, 0); LcdPutChar (cyr_i, 4, 0); LcdPutChar (cyr_n, 5, 0);
		
		LcdPutChar (cyr_I, 10, 0); LcdPutChar (cyr_U, 11, 0); LcdPutChar (dig_6, 12, 0);
		LcdPutChar (sym_def, 13, 0); LcdPutChar (dig_7, 14, 0); LcdPutChar (dig_3, 15, 0);
		//LcdPutChar (cyr_r, 14, 0);
  }
	
	LcdPutChar (lat_s, 11, 1); LcdPutChar (lat_l, 12, 1);
	LcdPutChar (lat_a, 13, 1); LcdPutChar (lat_v, 14, 1); LcdPutChar (lat_e, 15, 1);
		
	//scroll main string
	while (!LcdScrollString (main_string,7,50,count_main));
	count_main++;
	if (count_main == 55) count_main = 0;
	//
	if (CanFlag == 1)
	{
		LcdPutString (can_string, 3);
		LcdPutString (empty_string, 4);
		if (TestCanDone == 1) 
		{
			LcdPutString (get_string, 5);
		}
		else if (TestCanDone == 0) 
		{
			LcdPutString (wait_string, 5);
		} 
		else if (TestCanDone == 2) 
		{
			LcdPutString (fail_string, 5);
		} 
	}
	else if (SSPFlag == 1)
	{
		LcdPutString (ssp_string, 3);
		LcdPutString (empty_string, 4);
		if (TestSSPDone == 1) 
		{
			LcdPutString (get_string, 5);
		}
		else if (TestSSPDone == 0) 
		{
			LcdPutString (wait_string, 5);
		} 
		else if (TestSSPDone == 2) 
		{
			LcdPutString (fail_string, 5);
		} 
	}
	else if (USBFlag == 1)
	{
		LcdPutString (usb_string1, 3);
		LcdPutString (usb_string2, 4);
		LcdPutString (usb_string3, 5);
	}
	else if (UartFlag == 1)
	{
		LcdPutString (uart_string, 3);
		LcdPutString (empty_string, 4);
		if (TestUartDone == 1) 
		{
			LcdPutString (get_string, 5);
		}
		else 
		{
			LcdPutString (wait_string, 5);
		} 
	}
	else if (FullTestFlag == 1)
	{
		if (FullTestCANFlag == 1)
		{
			if (FullTestCANGetFlag == 0)
			{
				LcdPutString (can_string, 3);
				LcdPutString (wait_string, 4);
				LcdPutString (empty_string, 5);
			}
			else
			{
				LcdPutString (can_string, 3);
				LcdPutString (get_string, 4);
				LcdPutString (InfoString(USBInfo), 5);
			}
		}
		else if (FullTestSSPFlag == 1)
		{
			if (FullTestSSPGetFlag == 0)
			{
				LcdPutString (ssp_string, 3);
				LcdPutString (wait_string, 4);
				LcdPutString (empty_string, 5);
			}
			else
			{
				LcdPutString (ssp_string, 3);
				LcdPutString (get_string, 4);
				LcdPutString (InfoString(USBInfo), 5);
			}
		}
		else if (FullTestUARTFlag == 1)
		{
			LcdPutString (uart_string, 3);
			LcdPutString (pass_string, 4);
			LcdPutString (InfoString(USBInfo), 5);
		}
	}
	else
	{
		if (CursorPosItem == 0)
		{
			for (i=0;i<16;i++) LcdPutChar(menu_string[16*(MenuMainItem+0)+i],i,3);
			for (i=0;i<16;i++) LcdPutChar(menu_string[16*(MenuMainItem+1)+i],i,4);
			for (i=0;i<16;i++) LcdPutChar(menu_string[16*(MenuMainItem+2)+i],i,5);
			LcdPutChar (cursor, 15, 3);
		}
		if (CursorPosItem == 1)
		{
			for (i=0;i<16;i++) LcdPutChar(menu_string[16*(MenuMainItem-1)+i],i,3);
			for (i=0;i<16;i++) LcdPutChar(menu_string[16*(MenuMainItem+0)+i],i,4);
			for (i=0;i<16;i++) LcdPutChar(menu_string[16*(MenuMainItem+1)+i],i,5);
			LcdPutChar (cursor, 15, 4);
		}
		if (CursorPosItem == 2)
		{
			for (i=0;i<16;i++) LcdPutChar(menu_string[16*(MenuMainItem-2)+i],i,3);
			for (i=0;i<16;i++) LcdPutChar(menu_string[16*(MenuMainItem-1)+i],i,4);
			for (i=0;i<16;i++) LcdPutChar(menu_string[16*(MenuMainItem+0)+i],i,5);
			LcdPutChar (cursor, 15, 5);
		}
	}
}
/*******************************************************************************
* Function Name  : Timer2_IRQHandler
* Description    : This function handles Timer2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Timer2_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : Timer3_IRQHandler
* Description    : This function handles Timer3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Timer3_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : ADC_IRQHandler
* Description    : This function handles ADC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : COMPARATOR_IRQHandler
* Description    : This function handles COMPARATOR global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void COMPARATOR_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : SSP2_IRQHandler
* Description    : This function handles SSP2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SSP2_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : BACKUP_IRQHandler
* Description    : This function handles BACKUP global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BACKUP_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : EXT_INT1_IRQHandler
* Description    : This function handles EXT_INT1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXT_INT1_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : EXT_INT2_IRQHandler
* Description    : This function handles EXT_INT2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXT_INT2_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : EXT_INT3_IRQHandler
* Description    : This function handles EXT_INT3 global interrupt request. 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXT_INT3_IRQHandler(void)
{
}
/*******************************************************************************
* Function Name  : EXT_INT4_IRQHandler
* Description    : This function handles EXT_INT4 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXT_INT4_IRQHandler(void)
{
}
