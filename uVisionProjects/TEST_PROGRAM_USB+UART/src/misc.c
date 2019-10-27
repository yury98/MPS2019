#include "misc.h"

unsigned int PLL_Mul = 0x2;	// коэффициен умножения частоты

// простая задержка, всё же нужна иногда
void delay(unsigned int delay)
{
	while(delay--);
}
