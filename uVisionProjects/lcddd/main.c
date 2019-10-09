#include "MDR32F9Qx_port.h"             // Keil::Drivers:PORT
#include "MDR32F9Qx_rst_clk.h"          // Keil::Drivers:RST_CLK
#include "mlt_lcd.h"
#include "font.h"

PORT_InitTypeDef PORTAInit;
PORT_InitTypeDef PORTCInit;
PORT_InitTypeDef PORTEInit;
uint8_t* uart_string[]  =
{
	sym_sp,cyr_T,cyr_e,cyr_s,cyr_t,sym_sp,lat_C,lat_O,lat_M,sym_sp,cyr_p,cyr_o,cyr_r,lat_t,cyr_a,sym_sp
};
void MltPinCfg (void)
{
	/* Configure PORTA pins 0..7 for mlt inout data  */
	PORTAInit.PORT_Pin = PORT_Pin_0 |
												PORT_Pin_1 |
												PORT_Pin_2 |
												PORT_Pin_3 |
												PORT_Pin_4 |
												PORT_Pin_5 |
												PORT_Pin_6 |
												PORT_Pin_7;
	PORTAInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	PORTAInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
	PORTAInit.PORT_PD = PORT_PD_DRIVER;
	PORTAInit.PORT_GFEN = PORT_GFEN_OFF;
	PORTAInit.PORT_OE   = PORT_OE_IN;
	PORTAInit.PORT_FUNC = PORT_FUNC_PORT;
	PORTAInit.PORT_MODE  = PORT_MODE_DIGITAL;
	PORTAInit.PORT_SPEED = PORT_SPEED_SLOW;
	PORT_Init(MDR_PORTA, &PORTAInit);
	/* Configure PORTC pins 2,7 for mlt output  */
	PORTCInit.PORT_Pin = PORT_Pin_2 |
												PORT_Pin_7;
	PORTCInit.PORT_OE   = PORT_OE_IN;
	PORT_Init(MDR_PORTC, &PORTCInit);

	/* Configure PORTE pins 4,5,10,11 for mlt output */
		PORTEInit.PORT_Pin = PORT_Pin_4 |
												PORT_Pin_5 |
												PORT_Pin_10 |
												PORT_Pin_11;
		PORT_Init(MDR_PORTE, &PORTEInit);
}

int main(void) {
 // takt on ports A, C, E
	/* Enables the clock on PORTA */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA, ENABLE);
	/* Enables the clock on PORTB */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE);	
    /* Enables the clock on PORTC */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC, ENABLE);
	/* Enables the clock on PORTD */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD, ENABLE);
	/* Enables the clock on PORTE */
	RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE, ENABLE);
	
	MltPinCfg();
	LcdInit();
	DispOn(1);
	DispOn(2);
		LcdClearChip(1);
		LcdClearChip(2);
	while(1) {
LcdPutString (uart_string, 3);
	}
}






