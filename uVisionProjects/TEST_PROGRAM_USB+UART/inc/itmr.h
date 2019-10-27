// ������������ ������ �� ������� TIMER_NUM
#ifndef __ITMR_H__
#define __ITMR_H__

#include "misc.h"

#define TIMER_NUM	1
#define TICK(t)		(CPU_CLK * (t) * 1000)

//	������������� ������� ��� ������������� � �������� �������������
//	1 ��� - 1 ��
void itmr_Init(void);
//	������� ������� �������� �������� �����
unsigned int itmr_GetTick(void);
// �������� ��������� ��������� �������
//  true - �������� �����
//	false - ���	 
bool itmr_IsTime(unsigned int stime, unsigned int ival);
// �������� 
void itmr_Delay(unsigned int delay);

#endif
