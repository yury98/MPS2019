#include "opora.h"
#include "itmr.h"

/*
	Модуль интервального таймера - "незаменимый" элемент систем с суперциклом (while(1){...})
*/

static volatile unsigned int TickCnt; // счетчик тиков

//	инициализация таймера для использования в качестве интервального
//	1 тик - 1 мс
void itmr_Init(void)
{
	TIMER1->CNT=0;				 			
	TIMER1->ARR=TICK(1);				
	TIMER1->IE=0x2;						
	TIMER1->CNTRL=0x0001;				
	
	NVIC_EnableIRQ(TIMER1_IRQn); 			// разрешение прерываний 	
	TickCnt = 0;	
}

// обработчик прерываний
void TIMER1_Handler(void)
{	
	TIMER1->STATUS &= 0xFFFD;	// сброс прерывания по совпадению счетчика с значением ARR
	TickCnt++;					// инкремент счетчика
} 

// выдача текущего значения счетчика тиков
unsigned int itmr_GetTick(void)
{
	return TickCnt;
}

// проверка истечения интервала времени
bool itmr_IsTime(unsigned int stime, unsigned int ival)
{
	return((itmr_GetTick() - stime) >= (ival + 1));
}

// задержка на интервальном таймере
void itmr_Delay(unsigned int delay)
{
	unsigned int start = itmr_GetTick();

	while(!itmr_IsTime(start, delay));
}
