#include <opora.h>


#define CLEAR_NAK_SENTED()      USB->SIS|= 0x10 // проверка ответа NAK
#define GET_TDONE()             USB->SIS&0x01   //  проверка бита на успешное выполнение передачи
#define CLEAR_TDONE()           USB->SIS|=0x01 //Очищается записью 1
#define EN_IRQ()                USB->SIM|=1;     //прерывание tdone,reset
#define DSBL_IRQ()              USB->SIM&=~0x01;

/*****************************новые макросы*************************************/
#define EP0					   0
#define EP1					   1
#define EP2					   2
#define EPin          EP1
#define EPout         EP2

#define EP_ENABLE0               		USB->SEP0_CTRL|= 0x01     // разрешение оконечной точки
#define EP_ENABLE1               		USB->SEP1_CTRL|= 0x01     // разрешение оконечной точки
#define EP_ENABLE2               		USB->SEP2_CTRL|= 0x01     // разрешение оконечной точки

#define EP_SET_RDY0              USB->SEP0_CTRL|= 0x02     // готовность оконечной точки для приема данных
#define EP_SET_RDY1              USB->USB_SEP1_CTRL|= 0x02     // готовность оконечной точки для приема данных
#define EP_SET_RDY2              USB->USB_SEP2_CTRL|= 0x02     // готовность оконечной точки для приема данных

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

#define EP_INVERS_DATASEQ0       {
                                     if ((USB->SEP0_CTRL&0x04)==0)
                                     {
									                    USB->SEP0_CTRL|=0x04;} // инвертирование настройки ответа IN на запрос хоста Data1/Data0
									                   else {
									                    USB->SEP0_CTRL&=~0x04;}
									                   } 
#define EP_INVERS_DATASEQ1       {
                                     if ((USB->SEP1_CTRL&0x04)==0)
                                     {
									                    USB->SEP1_CTRL|=0x04;} // инвертирование настройки ответа IN на запрос хоста Data1/Data0
									                   else {
									                    USB->SEP1_CTRL&=~0x04;}
									                   } 
#define EP_INVERS_DATASEQ2       {
                                     if ((USB->SEP2_CTRL&0x04)==0)
                                     {
									                    USB->SEP2_CTRL|=0x04;} // инвертирование настройки ответа IN на запрос хоста Data1/Data0
									                   else {
									                    USB->SEP2_CTRL&=~0x04;}
									                   } 

#define EP_GET_ACKRXED0       	  USB->SEP0_STS&0x40  // получен ACK от хоста на переданные данные
#define EP_GET_ACKRXED1        	  USB->SEP1_STS&0x40  // получен ACK от хоста на переданные данные
#define EP_GET_ACKRXED2        	  USB->SEP2_STS&0x40  // получен ACK от хоста на переданные данные
																		 
																		 
#define EP_SET_STALL0            USB->SEP0_CTRL|= 0x08 // если точка разрешена и готова, и не работает в изохронном режиме, то на запрос хоста ответит STALL
#define EP_SET_STALL1            USB->SEP1_CTRL|= 0x08 // если точка разрешена и готова, и не работает в изохронном режиме, то на запрос хоста ответит STALL
#define EP_SET_STALL2            USB->SEP2_CTRL|= 0x08 // если точка разрешена и готова, и не работает в изохронном режиме, то на запрос хоста ответит STALL
																		 
#define EP_CLR_STALL0            (
	                                   USB->SEP0_CTRL&= 0x08; // отмена ответа STALL
                                     USB->SEP0_STS&= 0x20;
                                   )
#define EP_CLR_STALL1           (
	                                   USB->SEP1_CTRL&= 0x08; // отмена ответа STALL
                                     USB->SEP1_STS&= 0x20;
                                     )
#define EP_CLR_STALL2           (
	                                   USB->SEP2_CTRL&= 0x08; // отмена ответа STALL
                                     USB->SEP2_STS&= 0x20;
																		 
                                     )

#define EP_Get_count0 	       USB->SEP0_RXFDC_H   // хранит число байт записанных в буфер
#define EP_Get_count1 	       USB->SEP1_RXFDC_H   // хранит число байт записанных в буфер
#define EP_Get_count2 	       USB->SEP2_RXFDC_H   // хранит число байт записанных в буфер
																		 
#define EP_Get_data0(EPbuff)    {
                                     for(i=0;i<EP_Get_count0;i++)
                                     {
									                    EPbuff[i]=USB->SEP0_RXFD; // занесение в буфер данных из FIFO
									                   }
                                     }
#define EP_Get_data1(EPbuff)    {
                                     for(i=0;i<EP_Get_count1;i++)
                                     {
									                    EPbuff[i]=USB->SEP1_RXFD; // занесение в буфер данных из FIFO
									                   }
                                     }
#define EP_Get_data2(EPbuff)    {
                                     for(i=0;i<EP_Get_count2;i++)
                                     {
									                    EPbuff[i]=USB->SEP2_RXFD; // занесение в буфер данных из FIFO
									                   }
                                     }
									                                                                
#define MAXPACKET32		32 
#define MAXPACKET64		64 
																		 
#define STANDARD_DEVICE_REQUEST		0x00
#define STANDARD_INTERFACE_REQUEST	0x01
#define STANDARD_ENDPOINT_REQUEST	0x02
#define VENDOR_DEVICE_REQUEST		0x40
#define VENDOR_ENDPOINT_REQUEST		0x42	

#define GET_STATUS  									0
#define CLEAR_FEATURE     						1
#define SET_FEATURE                 	3
#define SET_ADDRESS                 	5
#define GET_DESCRIPTOR              	6
#define SET_DESCRIPTOR              	7
#define GET_CONFIGURATION           	8
#define SET_CONFIGURATION           	9
#define GET_INTERFACE               	10
#define SET_INTERFACE               	11
#define SYNCH_FRAME                 	12
#define VENDOR_GET_ANALOG_VALUE				1
#define VENDOR_SET_RB_HIGH_NIBBLE			2		


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
																		 
typedef struct {
	unsigned char bmRequestType;
	unsigned char bRequest;
	unsigned short wValue;
	unsigned short wIndex;
	unsigned short wLength;
} USB_SETUP_REQUEST, *PUSB_SETUP_REQUEST;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned short bcdUSB;
		unsigned char bDeviceClass;
		unsigned char bDeviceSubClass;
		unsigned char bDeviceProtocol;
		unsigned char bMaxPacketSize0;
		unsigned short idVendor;
		unsigned short idProduct;
		unsigned short bcdDevice;
		unsigned char iManufacturer;
		unsigned char iProduct;
		unsigned char iSerialNumber;
		unsigned char bNumConfigurations;
} USB_DEVICE_DESCRIPTOR, *PUSB_DEVICE_DESCRIPTOR;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned char bEndpointAddress;
		unsigned char bmAttributes;
		unsigned short wMaxPacketSize;
		unsigned char bInterval;
} USB_ENDPOINT_DESCRIPTOR, *PUSB_ENDPOINT_DESCRIPTOR;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned short wTotalLength;
		unsigned char bNumInterfaces;
		unsigned char bConfigurationValue;
		unsigned char iConfiguration;
		unsigned char bmAttributes;
		unsigned char MaxPower;
} USB_CONFIGURATION_DESCRIPTOR, *PUSB_CONFIGURATION_DESCRIPTOR;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned char bInterfaceNumber;
		unsigned char bAlternateSetting;
		unsigned char bNumEndpoints;
		unsigned char bInterfaceClass;
		unsigned char bInterfaceSubClass;
		unsigned char bInterfaceProtocol;
		unsigned char iInterface;
} USB_INTERFACE_DESCRIPTOR, *PUSB_INTERFACE_DESCRIPTOR;

typedef struct {
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned short wHIDClassSpecComp;
		unsigned char bCountry;
		unsigned char bNumDescriptors;
		unsigned char b1stDescType;
		unsigned short w1stDescLength;
} USB_HID_DESCRIPTOR, *PUSB_HID_DESCRIPTOR;

typedef struct {
	USB_CONFIGURATION_DESCRIPTOR ConfigDescriptor;
	USB_INTERFACE_DESCRIPTOR InterfaceDescriptor;
  	USB_ENDPOINT_DESCRIPTOR EndpointDescriptor0;
	USB_ENDPOINT_DESCRIPTOR EndpointDescriptor1;
} USB_CONFIG_DATA, *PUSB_CONFIG_DATA;

typedef struct {
	unsigned char bLenght;
      	unsigned char bDescriptorType;
      	char     bString[24];
} MANUFACTURER_DESCRIPTOR, *PMANUFACTURER_DESCRIPTOR;

typedef struct {
				unsigned char bLenght;
      	unsigned char bDescriptorType;
      	unsigned short wLANGID0;
} LANGID_DESCRIPTOR, *PLANGID_DESCRIPTOR;

/************************Exported functions**************************/
void USBsendData(unsigned char *data, int count);
void USB_SEND_ZERO_LENGTH_DATA(void);
void EP_STALL(uint8_t EPn);
void SEND_ZERO_LENGTH_DATA_EP0(void);
void sendDataEP0(unsigned char *data, int count);
void EP_TX_FIFO_FORCE_EMPTY(char EP)

void EP_ENABLE(char EP);

void EP_SET_RDY(char EP);

void EP_RX_FIFO_FORCE_EMPTY(char EP);

void EP_FIFO_READ_BYTE(char EP);

void EP_FIFO_LOAD_BYTE(char data, char EP);

void EP_ANS_WITH_DATA1(char EP);

void EP_ANS_WITH_DATA0(char EP);

void EP_INVERS_DATASEQ(char EP);

void EP_GET_ACKRXED(char EP);

void EP_SET_STALL(char EP);

void EP_CLR_STALL(char EP);

void EP_Get_count(char EP);

void EP_Get_data(char EPbuff[], char EP);

