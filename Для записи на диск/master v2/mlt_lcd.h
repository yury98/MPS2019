#include "MDR32Fx.h"
#include "MDR32F9Qx_port.h"

#define	Set_Res_Pin		PORT_SetBits  (MDR_PORTB,PORT_Pin_9);
#define	Clr_Res_Pin		PORT_ResetBits(MDR_PORTB,PORT_Pin_9);

#define	Set_Stb_Pin		PORT_SetBits  (MDR_PORTC,PORT_Pin_1);
#define	Clr_Stb_Pin		PORT_ResetBits(MDR_PORTC,PORT_Pin_1);

#define	Set_RW_Pin		PORT_SetBits  (MDR_PORTB,PORT_Pin_10);
#define	Clr_RW_Pin		PORT_ResetBits(MDR_PORTB,PORT_Pin_10);

#define	Set_A0_Pin		PORT_SetBits  (MDR_PORTC,PORT_Pin_0);
#define	Clr_A0_Pin		PORT_ResetBits(MDR_PORTC,PORT_Pin_0);

#define	Set_E1_Pin		PORT_SetBits  (MDR_PORTB,PORT_Pin_7);
#define	Clr_E1_Pin		PORT_ResetBits(MDR_PORTB,PORT_Pin_7);

#define	Set_E2_Pin		PORT_SetBits  (MDR_PORTB,PORT_Pin_8);
#define	Clr_E2_Pin		PORT_ResetBits(MDR_PORTB,PORT_Pin_8);

#define Data_Dir_InputPA  MDR_PORTA->OE &= 0xFFC0;
#define Data_Dir_OutputPA MDR_PORTA->OE |= 0x003F;
#define Data_Dir_InputPF  MDR_PORTF->OE &= 0xFFF3;
#define Data_Dir_OutputPF MDR_PORTF->OE |= 0x000C;

#define OutputData		(uint8_t)(MDR_PORTA->RXTX & 0x3F) | (uint8_t)((MDR_PORTF->RXTX<<4) & 0xC0);


void SetDataBits (uint8_t value);

void Delay (uint32_t value);
uint8_t ReadStatus (uint8_t Chip);

void DispOn (uint8_t Chip);
void DispOff (uint8_t Chip);
void LcdInit(void);


void SetPage (uint8_t Chip,uint8_t page);
void SetAdress (uint8_t Chip,uint8_t adress);
void WriteData (uint8_t Chip,uint8_t data);
uint8_t ReadData (uint8_t Chip);

void LcdClearPage (uint8_t Chip,uint8_t Page);
void LcdClearChip (uint8_t Chip);

void LcdPutChar (uint8_t* array, int Xpos, int Ypos);
void LcdPutString (uint8_t** array, int Ypos);
int LcdScrollString (uint8_t** array, int Ypos, int size,int cnt);
void LcdPutImage (uint8_t* array, int Ypos1, int Xpos1,int Ypos2, int Xpos2);

