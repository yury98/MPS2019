#ifndef __MISC_H__
#define __MISC_H__

//---------------------------------------------------------------------------------------------------
#define FCLK		16							// частота генератора МГц
extern unsigned int PLL_Mul;					// коэффициент умножения PLL
#define CPU_CLK		(FCLK * (PLL_Mul + 1))
#define	DELAY(d)	(FCLK * (PLL_Mul + 1) * (d) / 1000 + 1)	// FCLK * (PLL_Mul + 1) * d / 1000 + 1

//---------------------------------------------------------------------------------------------------
// полезные макро
#define BM(n)		(1ul << (n))

#define LOBYTE(w)         (unsigned char)(w)
#define HIBYTE(w)         (unsigned char)((w) >> 8)
#define MAKEWORD(lo, hi)  ((unsigned short)(unsigned char)(lo) | ((unsigned short)(unsigned char)(hi) << 8))

#define HINIBBLE(b)			(((unsigned char)(b) & 0xF0) >> 4)
#define LONIBBLE(b)			((unsigned int)(b) & 0x0F)

#define EXCHANGE(a) (((unsigned short)(a) >> 8) | (((unsigned short)(a) & 0xFF) << 8))							 

#define NO	((unsigned short)(-1))
//---------------------------------------------------------------------------------------------------										
typedef unsigned char bool;
#define true	1
#define false	0

//---------------------------------------------------------------------------------------------------
// задержка
void delay(unsigned int delay);

#define 	DELAY_160NS 	DELAY(160)
#define 	DELAY_226NS 	DELAY(226)
#define 	DELAY_480NS 	DELAY(480)
#define 	DELAY_1040NS 	DELAY(1040)
#define 	DELAY_8220NS 	DELAY(8220)
#define 	DELAY_10280NS 	DELAY(10280)

#endif
