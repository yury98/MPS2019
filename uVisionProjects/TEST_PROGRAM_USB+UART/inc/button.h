#ifndef __BUTTON_H__
#define __BUTTON_H__

// определение кнопок
enum {
	UP 		= 0x74, 
	DOWN 	= 0x6C, 
	RIGHT	= 0x5C, 
	LEFT 	= 0x3C, 
	SELECT 	= 0x78, 
	NOPRESS = 0x7C
};

// определение нажатой кнопки
unsigned int button_Press(void);	

	
#endif
