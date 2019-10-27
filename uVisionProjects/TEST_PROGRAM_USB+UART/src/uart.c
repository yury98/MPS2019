#include <string.h>
#include <opora.h>

#define MAX_BUF_SIZE	128				// максимальный размер буфера приема
static int RxBufByteCnt;				// количество принятых байт
static char RxBuf[MAX_BUF_SIZE];		// буфер приема
static char TxFlag = 0;	// признак передача байта завершена
static char IDN[]="Dmitry_Lipanov\r\n";
// настройка модуля UART 2
char comm_get(void);
void putstr(uint8_t *str);
unsigned char getstr(void);
void UART2Config(void)
{
	// настройка пинов UART'а
	PORTD->FUNC |= (0x1 << (13 * 2)) | (0x1 << (14 * 2));	// ноги на функцию UARTа
	PORTD->ANALOG |= (1 << 13) | (1 << 14);  // цифровые
	PORTD->PWR = (0x3 << (13 * 2)) | (0x3 << (14 * 2)); // максимальные фронты
	PORTD->OE|= (1<<13)|(1<<14); // как выходы
	// настройка скорости обмена 115200
	UART2->IBRD = 0x0034; // 52.7125=k
	UART2->FBRD = 0x1bd5;
	
	UART2->LCR_H = 0x0060;                  //FIFO отключен, длина слова 8 бит, два 2 стоп бита
	UART2->CR = 0x0301;
	UART2->IMSC = (1<<4)|(1<<5);            // разрешаем прерывания от приемника и передатчика
	UART2->ICR = (1 << 4) | (1 << 5);	// снимаем флаги прерывания по приему и передаче (для порядку)

	NVIC_EnableIRQ(UART2_IRQn); // разрешаем прерывания от модуля
	
	RxBufByteCnt = 0;	// еще ничего не принято из порта
}

// передача массива 
/*void uart_Send(char* buf, unsigned int buf_size)
{	
	unsigned int index;		
	for(index = 0; index < buf_size; index++){	
		TxFlag = false; // байт не передан
		UART2->DR = buf[index];			
		while(!TxFlag); // ждем завершения передачи
	}
}
*/void putchar(uint8_t ch)
{
  /* Loop until the end of transmission */
  while((UART2->FR&0x00000020)!=0)		   /*TXFF flak*/
  {
  }
  UART2->DR=(uint16_t)ch&(uint16_t)0x01FF;

}
// обработка принятых пакетов
void uart_Rcv(void)
{
	
	NVIC_DisableIRQ(UART2_IRQn);	// блокируем прерывание
	// если принята последовательность байт длинной более 1 и завершающаяся символом " " - пробел
	// начинаем обработку данных
	if(RxBufByteCnt > 1 && RxBuf[RxBufByteCnt - 1] =='?')
		{
		
									// если принят IDN то отвечает Dmitry_Lipanov
								if(!strncmp(RxBuf, "IDN?", 4))
												{
													if (TxFlag==0)
															{
																putstr("Dmitry_Lipanov");
																TxFlag=1;
																RxBufByteCnt=0; // РАЗОБРАТЬСЯ КУДА ВСТАВИТЬ ДАННУЮ СТРОКУ ЧТОБЫ МОЖНО БЫЛО Принимать любые слова и отвечать регулярно на "IDN?"!!!!!!!!
															}
												}	
								RxBufByteCnt=0;
		}
	NVIC_EnableIRQ(UART2_IRQn);		// разрешаем прерывание
}

// простой обработцик прерываний

void UART2_Handler(void)
{

	// прерывание по приему байта
	if(UART2->MIS & (1 << 4)){ //если возникло прерывание по приема
		UART2->ICR |= (1 << 4);		// снимаем флаг прерывания		
		if(RxBufByteCnt < (MAX_BUF_SIZE - 1))		// если буфер не переполнен то
			RxBuf[RxBufByteCnt++] = UART2->DR & 0xFF; // переносим байт в буфер приема
		TxFlag=0;
	}
	// прерывание по передачи байта
	if(UART2->MIS & (1 << 5)){  // если возникло прерывание по передачи
		UART2->ICR |= 1 << 5;		// снимаем флаг перывания		
		
	}
}



void putstr(uint8_t *str)
{
 uint8_t i;
 i=0;
  while(str[i]!=0)
  {
	while((UART2->FR&0x80)==0) // пока буфер передатчика не пуст (флаг TXFE) регистра FR
    {
    } 
	putchar(str[i]);
//  	MDR_UART1->DR=(uint16_t)str[i]&(uint16_t)0x01FF;	  
    i++;
  }
  putchar(0x0D);
  putchar(0x0A);
}
/*unsigned char getstr(void)
{
 uint8_t i;
 i=0;
unsigned char str[128];
	while(str[i]!=0)
	{
		while((UART2->FR&0x40)==1);
	str[i]=comm_get();
	i++;
	}
  return str[];
}
*/
char comm_get(void)
{
  uint8_t data;
  while((UART2->FR&UART_FR_RXFE)!=0)//ожидание прихода данных
  {
  }
  data=(uint8_t)UART2->DR;
  return data;
}

void comm_put(char d)
{
	  /* Loop until the end of transmission */
  while((UART2->FR&0x00000020)!=0)		   /*TXFF flak*/
  {
  }
  UART2->DR=(uint16_t)d&(uint16_t)0x01FF;
}

void comm_puts(const char* s)
{
  char c;
	while ((c=*s++) != '\0' ) {
		comm_put(c);
	}
}