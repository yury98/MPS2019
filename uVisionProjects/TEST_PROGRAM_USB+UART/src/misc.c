#include "misc.h"

unsigned int PLL_Mul = 0x2;	// ���������� ��������� �������

// ������� ��������, �� �� ����� ������
void delay(unsigned int delay)
{
	while(delay--);
}
