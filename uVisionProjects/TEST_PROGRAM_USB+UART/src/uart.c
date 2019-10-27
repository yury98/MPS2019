#include <string.h>
#include <opora.h>

#define MAX_BUF_SIZE	128				// ������������ ������ ������ ������
static int RxBufByteCnt;				// ���������� �������� ����
static char RxBuf[MAX_BUF_SIZE];		// ����� ������
static char TxFlag = 0;	// ������� �������� ����� ���������
static char IDN[]="Dmitry_Lipanov\r\n";
// ��������� ������ UART 2
char comm_get(void);
void putstr(uint8_t *str);
unsigned char getstr(void);
void UART2Config(void)
{
	// ��������� ����� UART'�
	PORTD->FUNC |= (0x1 << (13 * 2)) | (0x1 << (14 * 2));	// ���� �� ������� UART�
	PORTD->ANALOG |= (1 << 13) | (1 << 14);  // ��������
	PORTD->PWR = (0x3 << (13 * 2)) | (0x3 << (14 * 2)); // ������������ ������
	PORTD->OE|= (1<<13)|(1<<14); // ��� ������
	// ��������� �������� ������ 115200
	UART2->IBRD = 0x0034; // 52.7125=k
	UART2->FBRD = 0x1bd5;
	
	UART2->LCR_H = 0x0060;                  //FIFO ��������, ����� ����� 8 ���, ��� 2 ���� ����
	UART2->CR = 0x0301;
	UART2->IMSC = (1<<4)|(1<<5);            // ��������� ���������� �� ��������� � �����������
	UART2->ICR = (1 << 4) | (1 << 5);	// ������� ����� ���������� �� ������ � �������� (��� �������)

	NVIC_EnableIRQ(UART2_IRQn); // ��������� ���������� �� ������
	
	RxBufByteCnt = 0;	// ��� ������ �� ������� �� �����
}

// �������� ������� 
/*void uart_Send(char* buf, unsigned int buf_size)
{	
	unsigned int index;		
	for(index = 0; index < buf_size; index++){	
		TxFlag = false; // ���� �� �������
		UART2->DR = buf[index];			
		while(!TxFlag); // ���� ���������� ��������
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
// ��������� �������� �������
void uart_Rcv(void)
{
	
	NVIC_DisableIRQ(UART2_IRQn);	// ��������� ����������
	// ���� ������� ������������������ ���� ������� ����� 1 � ������������� �������� " " - ������
	// �������� ��������� ������
	if(RxBufByteCnt > 1 && RxBuf[RxBufByteCnt - 1] =='?')
		{
		
									// ���� ������ IDN �� �������� Dmitry_Lipanov
								if(!strncmp(RxBuf, "IDN?", 4))
												{
													if (TxFlag==0)
															{
																putstr("Dmitry_Lipanov");
																TxFlag=1;
																RxBufByteCnt=0; // ����������� ���� �������� ������ ������ ����� ����� ���� ��������� ����� ����� � �������� ��������� �� "IDN?"!!!!!!!!
															}
												}	
								RxBufByteCnt=0;
		}
	NVIC_EnableIRQ(UART2_IRQn);		// ��������� ����������
}

// ������� ���������� ����������

void UART2_Handler(void)
{

	// ���������� �� ������ �����
	if(UART2->MIS & (1 << 4)){ //���� �������� ���������� �� ������
		UART2->ICR |= (1 << 4);		// ������� ���� ����������		
		if(RxBufByteCnt < (MAX_BUF_SIZE - 1))		// ���� ����� �� ���������� ��
			RxBuf[RxBufByteCnt++] = UART2->DR & 0xFF; // ��������� ���� � ����� ������
		TxFlag=0;
	}
	// ���������� �� �������� �����
	if(UART2->MIS & (1 << 5)){  // ���� �������� ���������� �� ��������
		UART2->ICR |= 1 << 5;		// ������� ���� ���������		
		
	}
}



void putstr(uint8_t *str)
{
 uint8_t i;
 i=0;
  while(str[i]!=0)
  {
	while((UART2->FR&0x80)==0) // ���� ����� ����������� �� ���� (���� TXFE) �������� FR
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
  while((UART2->FR&UART_FR_RXFE)!=0)//�������� ������� ������
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