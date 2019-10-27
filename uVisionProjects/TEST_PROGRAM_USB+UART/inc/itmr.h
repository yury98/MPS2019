// интервальный таймер на таймере TIMER_NUM
#ifndef __ITMR_H__
#define __ITMR_H__

#include "misc.h"

#define TIMER_NUM	1
#define TICK(t)		(CPU_CLK * (t) * 1000)

//	инициализация таймера для использования в качестве интервального
//	1 тик - 1 мс
void itmr_Init(void);
//	считать текущее значение счетчика тиков
unsigned int itmr_GetTick(void);
// проверка истечения интервала времени
//  true - интервал истек
//	false - нет	 
bool itmr_IsTime(unsigned int stime, unsigned int ival);
// задержка 
void itmr_Delay(unsigned int delay);

#endif
