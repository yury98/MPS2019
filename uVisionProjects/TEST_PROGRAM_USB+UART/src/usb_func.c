#include "usb_func.h"
#include "term_io.h"

void USBsendData(unsigned char *data, int count) // функция передачи данных по USB
{
    int i;

    EP_TX_FIFO_FORCE_EMPTY(EPin); // сброс указателя FIFO
    EP_ENABLE(EPin); // оконечная точка разрешена
//     EP_ANS_WITH_DATA0(EPin); // отвечать на IN-запрос с DATA0
    for (i=0;i<count;i++)
    {
		EP_FIFO_LOAD_BYTE(EPin,data[i]); // заносим данные из буфера data в FIFO
        if ( (i+1)%MAXPACKET64 == 0 )
        {
            EP_SET_RDY(EPin); // готовность ок. точки
            while ( (USB->SIS&0x01)==0/*!GET_TDONE()*/ ) // ждем успешного приема
                ;
						
            while ( (USB->SEP1_STS&0x40)==0/*!EP_GET_ACKRXED(EPn)*/ )  // waiting for ACK from host
                ;
						putstr("ACKrxed"); // получили ответ - отправим в терминал
            CLEAR_TDONE();
            EP_INVERS_DATASEQ(EPin); //инвертирование настройки ответа IN на запрос хоста Data1/Data0
				}
		EP_SET_RDY(EPin);					//ответит на IN пакет данными
		while ( (USB->SIS&0x01)==0 );// ждем успешного выполнения передачи
		while ( (USB->SEP1_STS&0x40)==0  );  // waiting for ACK from host
		CLEAR_TDONE();
		EP_INVERS_DATASEQ(EPin); //инвертирование настройки ответа IN на запрос хоста Data1/Data0
		}
}

void EP_STALL(uint8_t EPn)
{
//  EP_TX_FIFO_FORCE_EMPTY(EPn);
//    EP_ANS_WITH_DATA1(EPn);
    EP_ENABLE(EPn);
	  EP_SET_STALL(EPn);
    EP_SET_RDY(EPn);

    while (!(MDR_USB->USB_SEP[EPn].STS|0x20)); /*stallsent*/

	  EP_CLR_STALL(EPn);
//    EP_TX_FIFO_FORCE_EMPTY(EPn);
    CLEAR_TDONE();
//    EP_INVERS_DATASEQ(EPn);
}

void EP_TX_FIFO_FORCE_EMPTY(char EP)
{
	if (EP==0)
		EP_TX_FIFO_FORCE_EMPTY0;
	
	if (EP==1)
		EP_TX_FIFO_FORCE_EMPTY1;
	
	if (EP==2)
		EP_TX_FIFO_FORCE_EMPTY2;
}
void EP_ENABLE(char EP)
{
	if (EP==0)
		EP_ENABLE0;
	
	if (EP==1)
		EP_ENABLE1;
	
	if (EP==2)
		EP_ENABLE2;
}

void EP_SET_RDY(char EP)
{
		if (EP==0)
		EP_SET_RDY;
	
	if (EP==1)
		EP_SET_RDY1;
	
	if (EP==2)
		EP_SET_RDY2;
}

void EP_RX_FIFO_FORCE_EMPTY(char EP)
{	
		if (EP==0)
		EP_RX_FIFO_FORCE_EMPTY0;
	
	if (EP==1)
		EP_RX_FIFO_FORCE_EMPTY1;
	
	if (EP==2)
		EP_RX_FIFO_FORCE_EMPTY2;
}

void EP_FIFO_READ_BYTE(char EP)
{
	if (EP==0)
		EP_FIFO_READ_BYTE0;
	
	if (EP==1)
		EP_FIFO_READ_BYTE1;
	
	if (EP==2)
		EP_FIFO_READ_BYTE2;
}
void EP_FIFO_LOAD_BYTE(char data, char EP)
{
		if (EP==0)
		EP_FIFO_LOAD_BYTE0(data);
	
	if (EP==1)
		EP_FIFO_LOAD_BYTE1(data);
	
	if (EP==2)
		EP_FIFO_LOAD_BYTE2(data);
}
void EP_ANS_WITH_DATA1(char EP)
{
	if (EP==0)
		EP_ANS_WITH_DATA10;
	
	if (EP==1)
		EP_ANS_WITH_DATA11;
	
	if (EP==2)
		EP_ANS_WITH_DATA12;
}
void EP_ANS_WITH_DATA0(char EP)
{
	if (EP==0)
	EP_ANS_WITH_DATA00;
	
	if (EP==1)
		EP_ANS_WITH_DATA01;
	
	if (EP==2)
		EP_ANS_WITH_DATA02;
}
void EP_INVERS_DATASEQ(char EP)
{
	if (EP==0)
	EP_INVERS_DATASEQ0;
	
	if (EP==1)
		EP_INVERS_DATASEQ1;
	
	if (EP==2)
		EP_INVERS_DATASEQ2;
}
void EP_GET_ACKRXED0(char EP)
{
	if (EP==0)
	EP_GET_ACKRXED00;
	
	if (EP==1)
		EP_GET_ACKRXED01;
	
	if (EP==2)
		EP_GET_ACKRXED02;
}
void EP_SET_STALL(char EP)
{
	if (EP==0)
	EP_SET_STALL0;
	
	if (EP==1)
		EP_SET_STALL1;
	
	if (EP==2)
		EP_SET_STALL2;
}
void EP_CLR_STALL(char EP)
{
	if (EP==0)
	EP_CLR_STALL0;
	
	if (EP==1)
		EP_CLR_STALL1;
	
	if (EP==2)
		EP_CLR_STALL2;
}
void EP_Get_count(char EP)
{
	if (EP==0)
	EP_Get_count0;
	
	if (EP==1)
		EP_Get_count1;
	
	if (EP==2)
		EP_Get_count2;
}
void EP_Get_data(char EPbuff[], char EP)
{
		if (EP==0)
	EP_Get_data0(*EPbuff[]);
	
	if (EP==1)
		EP_Get_data1(*EPbuff[]);
	
	if (EP==2)
		EP_Get_data2(*EPbuff[]);
}