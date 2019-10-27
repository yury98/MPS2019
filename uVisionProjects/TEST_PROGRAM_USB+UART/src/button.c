#include "opora.h"
#include "button.h"
#include "itmr.h"
#include "misc.h"

/*
	Драйвер опроса кнопок, с подавлением дребезга и отсутствием задержек(интервальный таймер в деле)
*/

static unsigned char StateButton; // состояние автомата контроля кнопок
static unsigned int PrevState = NOPRESS, // предыдущее состояние кнопок
					StartInt; // интервал отслеживания дребезга контакта
#define INT_DELAY	100 // интевал подавления дребезга в миллисекундах
enum{
	BUTTON_NO_PRESS, // ожидания изменения состояния кнопок
	BUTTON_DELAY	 // "задержка" на подавление дребезга
};

//------------------------------------------------------------
// определение нажатой кнопки
// на выходе состояние кнопок
unsigned int button_Press(void)
{	
	unsigned int cur_state = PORTD->RXTX & NOPRESS;			

	switch(StateButton){
		case BUTTON_NO_PRESS:			
			if(cur_state != PrevState){
				StateButton = BUTTON_DELAY;
				StartInt = itmr_GetTick();
				return NOPRESS;
			}
			else
				return NOPRESS;			
		
		case BUTTON_DELAY:			
			if(itmr_IsTime(StartInt, INT_DELAY)){
				if(cur_state != PrevState){
					PrevState = cur_state;					
					StateButton = BUTTON_NO_PRESS;
					return PrevState;
				}
				else{
					StateButton = BUTTON_NO_PRESS;
					return NOPRESS;
				}
			}				
			else{
				StateButton = BUTTON_DELAY;
				return NOPRESS;
			}
		
		default:
			return NOPRESS;						
	}		
}
