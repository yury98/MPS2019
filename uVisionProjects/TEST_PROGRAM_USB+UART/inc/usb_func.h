#include <opora.h>

#define CLEAR_NAK_SENTED()      USB->SIS|= 0x10 // проверка ответа NAK
#define GET_TDONE()             USB->SIS&0x01   //  проверка бита на успешное выполнение передачи
#define CLEAR_TDONE()           USB->SIS|=0x01 //Очищается записью 1
#define EN_IRQ()                USB->SIM|=1;     //прерывание tdone,reset
#define DSBL_IRQ()              USB->SIM&=~0x01;
//int i;
/*****************************новые макросы*************************************/
#define EP0					   0
#define EP1					   1
#define EP2					   2
#define EP3						3
#define EPin          EP1
#define EPout         EP2

#define EP_ENABLE0               		USB->SEP0_CTRL|= 0x01     // разрешение оконечной точки
#define EP_ENABLE1               		USB->SEP1_CTRL|= 0x01     // разрешение оконечной точки
#define EP_ENABLE2               		USB->SEP2_CTRL|= 0x01     // разрешение оконечной точки

#define EP_SET_RDY0              USB->SEP0_CTRL|=0x02     // готовность оконечной точки для приема данных
#define EP_SET_RDY1              USB->SEP1_CTRL|=0x02     // готовность оконечной точки для приема данных
#define EP_SET_RDY2              USB->SEP2_CTRL|=0x02     // готовность оконечной точки для приема данных

#define EP_TX_FIFO_FORCE_EMPTY0 		 USB->SEP0_TXFDC|= 0x01 /// очистить указатель буфера FIFO передатчика
#define EP_TX_FIFO_FORCE_EMPTY1  		USB->SEP1_TXFDC|= 0x01 /// очистить указатель буфера FIFO передатчика
#define EP_TX_FIFO_FORCE_EMPTY2  		USB->SEP2_TXFDC|= 0x01 /// очистить указатель буфера FIFO передатчика

#define EP_RX_FIFO_FORCE_EMPTY0 		 USB->SEP0_RXFC|= 0x01  // очистить указатель(сбросить) буфера FIFO приемника
#define EP_RX_FIFO_FORCE_EMPTY1  	 	USB->SEP1_RXFC|= 0x01  // очистить указатель(сбросить) буфера FIFO приемника
#define EP_RX_FIFO_FORCE_EMPTY2 		 USB->SEP2_RXFC|= 0x01  // очистить указатель(сбросить) буфера FIFO приемника

#define	EP_FIFO_READ_BYTE0	     USB->SEP0_RXFD // чтение принятых данных из FIFO
#define	EP_FIFO_READ_BYTE1	     USB->SEP1_RXFD // чтение принятых данных из FIFO
#define	EP_FIFO_READ_BYTE2	     USB->SEP2_RXFD // чтение принятых данных из FIFO

#define	EP_FIFO_LOAD_BYTE0(data)	 USB->SEP0_TXFD=(uint8_t)data  // загрузка данных на передачу в FIFO
#define	EP_FIFO_LOAD_BYTE1(data)	 USB->SEP1_TXFD=(uint8_t)data  // загрузка данных на передачу в FIFO
#define	EP_FIFO_LOAD_BYTE2(data)	 USB->SEP2_TXFD=(uint8_t)data  // загрузка данных на передачу в FIFO

#define EP_ANS_WITH_DATA10       USB->SEP0_CTRL|= 0x04 // отвечать IN на запрос хоста c DATA1
#define EP_ANS_WITH_DATA11       USB->SEP1_CTRL|= 0x04 // отвечать IN на запрос хоста c DATA1
#define EP_ANS_WITH_DATA12       USB->SEP2_CTRL|= 0x04 // отвечать IN на запрос хоста c DATA1


#define EP_ANS_WITH_DATA00       USB->SEP0_CTRL&=~0x04 // не отвечать IN на запрос хоста c DATA0
#define EP_ANS_WITH_DATA01       USB->SEP1_CTRL&=~0x04 // не отвечать IN на запрос хоста c DATA0
#define EP_ANS_WITH_DATA02       USB->SEP2_CTRL&=~0x04 // не отвечать IN на запрос хоста c DATA0
 
#define EP_INVERS_DATASEQ0 	{if((USB->SEP0_CTRL&0x04)==0) USB->SEP0_CTRL|=0x04; else USB->SEP0_CTRL&=~0x04;}                                     
#define EP_INVERS_DATASEQ1 	{if((USB->SEP1_CTRL&0x04)==0) USB->SEP1_CTRL|=0x04; else USB->SEP1_CTRL&=~0x04;}                                
#define EP_INVERS_DATASEQ2	{if((USB->SEP2_CTRL&0x04)==0) USB->SEP2_CTRL|=0x04;  else  USB->SEP2_CTRL&=~0x04;}
                                    

#define EP_GET_ACKRXED0	USB->SEP0_STS&0x40  // получен ACK от хоста на переданные данные
#define EP_GET_ACKRXED1	USB->SEP1_STS&0x40  // получен ACK от хоста на переданные данные
#define EP_GET_ACKRXED2	USB->SEP2_STS&0x40  // получен ACK от хоста на переданные данные
																		 
																		 
#define EP_SET_STALL0	USB->SEP0_CTRL|= 0x08 // если точка разрешена и готова, и не работает в изохронном режиме, то на запрос хоста ответит STALL
#define EP_SET_STALL1	USB->SEP1_CTRL|= 0x08 // если точка разрешена и готова, и не работает в изохронном режиме, то на запрос хоста ответит STALL
#define EP_SET_STALL2	USB->SEP2_CTRL|= 0x08 // если точка разрешена и готова, и не работает в изохронном режиме, то на запрос хоста ответит STALL
																		 
#define EP_CLR_STALL0	{USB->SEP0_CTRL&=0x08; USB->SEP0_STS&=0x20;}

#define EP_CLR_STALL1	{USB->SEP1_CTRL&=0x08; USB->SEP1_STS&=0x20;}

#define EP_CLR_STALL2	{USB->SEP2_CTRL&=0x08; USB->SEP2_STS&=0x20;}

#define EP_Get_count0	USB->SEP0_RXFDC_H// хранит число байт записанных в буфер
#define EP_Get_count1	USB->SEP1_RXFDC_H// хранит число байт записанных в буфер
#define EP_Get_count2	USB->SEP2_RXFDC_H// хранит число байт записанных в буфер
/*																		 
#define EP_Get_data0(EPbuff)    {for(int i=0;i<EP_Get_count0;i++) EPbuff[i]=USB->SEP0_RXFD;}

#define EP_Get_data1(EPbuff)    {for(int i=0;i<EP_Get_count1;i++) EPbuff[i]=USB->SEP1_RXFD;}
                                     
#define EP_Get_data2(EPbuff)    {for(int i=0;i<EP_Get_count2;i++) EPbuff[i]=USB->SEP2_RXFD;}
*/
									                                                                
#define MAXPACKET32		32 
#define MAXPACKET64		64 
/************************Exported functions**************************/
void USBsendData(unsigned char *data, int count);
void USB_SEND_ZERO_LENGTH_DATA(void);
void EP_STALL(uint8_t EPn);
void SEND_ZERO_LENGTH_DATA_EP0(void);
void sendDataEP0(unsigned char *data, int count);
void EP_TX_FIFO_FORCE_EMPTY(char EP);
unsigned char GetTS(char EP);
unsigned char GetCTRL(char EP);
unsigned char GetSTS(char EP);

void EP_ENABLE(char EP);

void EP_SET_RDY(char EP);

void EP_RX_FIFO_FORCE_EMPTY(char EP);

char EP_FIFO_READ_BYTE(char EP);

void EP_FIFO_LOAD_BYTE(char data, char EP);

void EP_ANS_WITH_DATA1(char EP);

void EP_ANS_WITH_DATA0(char EP);

void EP_INVERS_DATASEQ(char EP);

char EP_GET_ACKRXED(char EP);

void EP_SET_STALL(char EP);

void EP_CLR_STALL(char EP);

char EP_Get_count(char EP);

//char* EP_Get_data(char *EPbuff, char EP);

#define D11_SET_HUB_ADDRESS 		0xD0
#define D11_SET_ADDRESS_ENABLE		0xD1
#define D11_SET_ENDPOINT_ENABLE  	0xD8
#define D11_SET_MODE			0xF3
#define D11_READ_INTERRUPT_REGISTER 	0xF4
#define D11_READ_LAST_TRANSACTION	0x40
#define D11_SET_ENDPOINT_STATUS		0x40
#define D11_READ_ENDPOINT_STATUS	0x80
#define D11_READ_BUFFER			0xF0
#define D11_WRITE_BUFFER		0xF0
#define D11_CLEAR_BUFFER		0xF2
#define D11_VALIDATE_BUFFER		0xFA
#define D11_ACK_SETUP			0xF1

#define D11_ENDPOINT_EP0_OUT 		0x02
#define D11_ENDPOINT_EP0_IN 		0x03
#define D11_ENDPOINT_EP1_OUT 		0x05
#define D11_ENDPOINT_EP1_IN 		0x04
#define D11_ENDPOINT_EP2_OUT 		0x06
#define D11_ENDPOINT_EP2_IN 		0x07
#define D11_ENDPOINT_EP3_OUT 		0x08
#define D11_ENDPOINT_EP3_IN 		0x09

#define D11_CMD_ADDR	  	   	0x36
#define D11_DATA_ADDR_WRITE		0x34
#define D11_DATA_ADDR_READ		0x35

#define D11_INT_BUS_RESET		0x4000
#define D11_INT_EP0_OUT			0x0004
#define D11_INT_EP0_IN			0x0008
#define D11_INT_EP1_OUT			0x0020
#define D11_INT_EP1_IN			0x0010
#define D11_INT_EP2_OUT			0x0040
#define D11_INT_EP2_IN			0x0080
#define D11_INT_EP3_OUT			0x0100
#define D11_INT_EP3_IN			0x0200

#define D11_LAST_TRAN_SETUP		0x20

#define STANDARD_DEVICE_REQUEST		0x00
#define STANDARD_INTERFACE_REQUEST	0x01
#define STANDARD_ENDPOINT_REQUEST	0x02
#define VENDOR_DEVICE_REQUEST		0x40
#define VENDOR_ENDPOINT_REQUEST		0x42

#define GET_STATUS  			0
#define CLEAR_FEATURE     		1
#define SET_FEATURE                 	3
#define SET_ADDRESS                 	5
#define GET_DESCRIPTOR              	6
#define SET_DESCRIPTOR              	7
#define GET_CONFIGURATION           	8
#define SET_CONFIGURATION           	9
#define GET_INTERFACE               	10
#define SET_INTERFACE               	11
#define SYNCH_FRAME                 	12
#define VENDOR_GET_ANALOG_VALUE		1
#define VENDOR_SET_RB_HIGH_NIBBLE	2

#define	ENDPOINT_HALT			0

#define TYPE_DEVICE_DESCRIPTOR          1
#define TYPE_CONFIGURATION_DESCRIPTOR   2
#define TYPE_STRING_DESCRIPTOR          3
#define TYPE_INTERFACE_DESCRIPTOR       4
#define TYPE_ENDPOINT_DESCRIPTOR        5
#define TYPE_HID_DESCRIPTOR		0x21

#define USB_ENDPOINT_TYPE_CONTROL	0x00
#define USB_ENDPOINT_TYPE_ISOCHRONOUS	0x01
#define USB_ENDPOINT_TYPE_BULK		0x02
#define USB_ENDPOINT_TYPE_INTERRUPT	0x03
