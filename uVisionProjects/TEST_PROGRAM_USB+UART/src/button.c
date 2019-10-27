#include "opora.h"
#include "button.h"
#include "itmr.h"
#include "misc.h"

/*
	������� ������ ������, � ����������� �������� � ����������� ��������(������������ ������ � ����)
*/

static unsigned char StateButton; // ��������� �������� �������� ������
static unsigned int PrevState = NOPRESS, // ���������� ��������� ������
					StartInt; // �������� ������������ �������� ��������
#define INT_DELAY	100 // ������� ���������� �������� � �������������
enum{
	BUTTON_NO_PRESS, // �������� ��������� ��������� ������
	BUTTON_DELAY	 // "��������" �� ���������� ��������
};

//------------------------------------------------------------
// ����������� ������� ������
// �� ������ ��������� ������
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
