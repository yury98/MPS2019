#include "opora.h"
#include "itmr.h"

/*
	������ ������������� ������� - "�����������" ������� ������ � ����������� (while(1){...})
*/

static volatile unsigned int TickCnt; // ������� �����

//	������������� ������� ��� ������������� � �������� �������������
//	1 ��� - 1 ��
void itmr_Init(void)
{
	TIMER1->CNT=0;				 			
	TIMER1->ARR=TICK(1);				
	TIMER1->IE=0x2;						
	TIMER1->CNTRL=0x0001;				
	
	NVIC_EnableIRQ(TIMER1_IRQn); 			// ���������� ���������� 	
	TickCnt = 0;	
}

// ���������� ����������
void TIMER1_Handler(void)
{	
	TIMER1->STATUS &= 0xFFFD;	// ����� ���������� �� ���������� �������� � ��������� ARR
	TickCnt++;					// ��������� ��������
} 

// ������ �������� �������� �������� �����
unsigned int itmr_GetTick(void)
{
	return TickCnt;
}

// �������� ��������� ��������� �������
bool itmr_IsTime(unsigned int stime, unsigned int ival)
{
	return((itmr_GetTick() - stime) >= (ival + 1));
}

// �������� �� ������������ �������
void itmr_Delay(unsigned int delay)
{
	unsigned int start = itmr_GetTick();

	while(!itmr_IsTime(start, delay));
}
