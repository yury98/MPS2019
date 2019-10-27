#include <string.h>
#include <opora.h>
#include <usb_func.h>
#include <uart.h>

typedef struct {
	unsigned char bmRequestType;
	unsigned char bRequest;
	unsigned short wValue;
	unsigned short wIndex;
	unsigned short wLength;
} USB_SETUP_REQUEST, *PUSB_SETUP_REQUEST;

typedef struct {     // ДЕСКРИПТОР УСТРОЙСТВА
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

typedef struct {  // ДЕСКРИПТОР КОНЕЧНОЙ ТОЧКИ
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned char bEndpointAddress;
		unsigned char bmAttributes;
		unsigned short wMaxPacketSize;
		unsigned char bInterval;
} USB_ENDPOINT_DESCRIPTOR, *PUSB_ENDPOINT_DESCRIPTOR;

typedef struct {   // ДЕСКРИПТОР КОНФИГУРАЦИИ
		unsigned char bLength;
		unsigned char bDescriptorType;
		unsigned short wTotalLength;
		unsigned char bNumInterfaces;
		unsigned char bConfigurationValue;
		unsigned char iConfiguration;
		unsigned char bmAttributes;
		unsigned char MaxPower;
} USB_CONFIGURATION_DESCRIPTOR, *PUSB_CONFIGURATION_DESCRIPTOR;

typedef struct {     // ДЕСКРИПТОР ИНТЕРФЕЙСА
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

typedef struct {          // КОНФИГУРАЦИОННЫЕ ДАННЫЕ - структура структур, включающая в себя дескрипторы устройства, конфигурации, интерфейса и конечной точки
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

const USB_DEVICE_DESCRIPTOR DeviceDescriptor = {
    sizeof(USB_DEVICE_DESCRIPTOR), /* bLength */
    TYPE_DEVICE_DESCRIPTOR,        /* bDescriptorType */
    0x0200,                        /* bcdUSB USB Version 1.1 */
    0,                             /* bDeviceClass */
    0,                             /* bDeviceSubclass */
    0,                             /* bDeviceProtocol */
    8,                             /* bMaxPacketSize 8 Bytes */
    0x1234, //0x04B4,                        /* idVendor (Cypress Semi) */
    0x5678, //0x0002,                        /* idProduct (USB Thermometer Example) */
    //0x045E,                        /* Intel 82930 USB Bulk IO Test Board */
   // 0x930A,
    0x0000,                        /* bcdDevice */
    1,                             /* iManufacturer String Index */
    0,                             /* iProduct String Index */
    0,                             /* iSerialNumber String Index */
    1                              /* bNumberConfigurations */
};

/* ----------------================СТРУКТУРА СТРУКТУ, СОДЕРЖАЩАЯ В СЕБЕ ВСЕ ДЕСКРИПТОРЫ ==========------------------- */
const USB_CONFIG_DATA ConfigurationDescriptor = {
    {                              /* configuration descriptor */
    sizeof(USB_CONFIGURATION_DESCRIPTOR), /* bLength */
    TYPE_CONFIGURATION_DESCRIPTOR, /* bDescriptorType */
    sizeof(USB_CONFIG_DATA),       /* wTotalLength */
    1,                             /* bNumInterfaces */
    1,                             /* bConfigurationValue */
    0,                             /* iConfiguration String Index */
    0x80,                          /* bmAttributes Bus Powered, No Remote Wakeup */
    0x50                           /* bMaxPower, 100mA */ 
    },
    {                              /* interface descriptor */
    sizeof(USB_INTERFACE_DESCRIPTOR), /* bLength */
    TYPE_INTERFACE_DESCRIPTOR,     /* bDescriptorType */
    0,                             /* bInterface Number */
    0,                             /* bAlternateSetting */
    2,                             /* bNumEndpoints 0 - если используем только 0-вую конечную точку, если еще 1 - то 1, если еще 2 - то 2*/ 
    0xFF,                          /* bInterfaceClass (Vendor specific) */
    0xFF,                          /* bInterfaceSubClass */
    0xFF,                          /* bInterfaceProtocol */
    0                              /* iInterface String Index */
    },
    {                              /* endpoint descriptor */
    sizeof(USB_ENDPOINT_DESCRIPTOR),  /* bLength */
    TYPE_ENDPOINT_DESCRIPTOR,      /* bDescriptorType */
    0x01,                          /* bEndpoint Address EP1 OUT */
    0x02,                          /* bmAttributes - канал передачи данных */
    0x0008,                        /* wMaxPacketSize */
    0x00                           /* bInterval */
    },
    {                              /* endpoint descriptor */
    sizeof(USB_ENDPOINT_DESCRIPTOR),  /* bLength */
    TYPE_ENDPOINT_DESCRIPTOR,      /* bDescriptorType */
    0x81,                          /* bEndpoint Address EP1 IN */
    0x02,                          /* bmAttributes - канал передачи данных */
    0x0008,                        /* wMaxPacketSize */
    0x00                           /* bInterval */
    }
};
const unsigned char * pSendBuffer;
unsigned char BytesToSend;
unsigned char CtlTransferInProgress;
unsigned char DeviceAddress;
unsigned char DeviceConfigured;
void GetDescriptor(PUSB_SETUP_REQUEST SetupPacket);
unsigned char TS,SIS,CTRL,STS,RxCH,SA;
void Clocks(void);
void PerClkConfig(void);
void UART2Config(void);
void Setup_USB(void);
void Process_EP0_OUT_Interrupt(void);
void Send_DATA_EP(char EP, const unsigned char *PSendBuffer, int count);
// тестовые посылки
unsigned char IDN[]="Dmitry_Lipanov\r\n";
void putstr(uint8_t *str);
unsigned char _data[64];
int main(void)
{

	Clocks();
	PerClkConfig();
	// настройка UART2, скорости обмена и режимы работы порта
	UART2Config();
	Setup_USB();
  putstr("work UART");
	putstr("USB_InitPass");
	while(1)
	{
	}
}									
int getPacketEPn(char EPn, unsigned char *buf)
{
     int i,count;
     EP_SET_RDY(EPn); // разрешение  готовности
     EP_ENABLE(EPn); // разрешение оконечной точки 0
     while (!(USB->SIS|USB_SIS_SCTDONE)); // когда сделана успешная передача

     count = EP_Get_count(EPn); // читаем кол-во полученных байтов
     for (i=0;i<count;i++)
         buf[i]=EP_FIFO_READ_BYTE(EPn); // считываем данные в программный буфер
     EP_RX_FIFO_FORCE_EMPTY(EPn); // сброс указателя FIFO
     CLEAR_TDONE(); // очистка бита успешной передачи
     return count;
			 //  else /* Incorrect packet size */
//  {
//    RX_FIFO_FORCE_EMPTY(EPx);
//    result = USB_ERROR;
//    /* Switch into STALL state */
//    USB_EP_Stall(EPx, USB_STALL_PROTO);
//  }
 }
void USB_Handler(void) // заходим сюда либо по окончанию передачи либо по RESET
{
	 char EPnum=0;

  if((USB->SIS&0x01)!=0)           /*ClearTransfer */
  {
    CLEAR_TDONE();
    while((EP_Get_count(EPnum)==0)&&(EPnum<3))	  /*Смотрим буферы оконечных точек и определяем от какой из них произошло прерывание ( на какую пришли данные)*/
    {
  	  EPnum++;									  
    }
	EP_ENABLE(EPnum);	 // разрешаем эту оконечную точку
//	RXcount=EP_Get_count(EPnum); // читаем кол-во полученных данных
		
		/* ==================ДЛЯ ОТЛАДКИ ============== */
		TS=GetTS(EPnum); // считываем тип последней передачи 
		STS=GetSTS(EPnum); // состояние оконечной точки
		CTRL=GetCTRL(EPnum);  // настройки оконечной точки: тип передачи, разрешение и т.д
		RxCH=EP_Get_count(EPnum); // количество принятых данных 
    SIS=USB->SIS; // считываем флаги событий контроллера SLAVE
    SA=USB->SA; // функциональный адрес контроллера USB
		
		putstr("TS=");
		putchar(TS);
		putstr("STS=");
		putchar(STS);
		putstr("CTRL=");
		putchar(CTRL);
		putstr("RxCH=");
		putchar(RxCH);
		putstr("SIS=");
		putchar(SIS);
		putstr("SA=");
		putchar(SA);
			/* ==================ДЛЯ ОТЛАДКИ ============== */
			switch(EPnum)
    {
	  	case EP0:
								switch (USB->SEP0_TS)
								{
									case 0:
													putstr("USB EP0 Setup packet interrupt\n");
													/* Обработчик прерывания от EP0 для setup пакета*/ 
													getPacketEPn(EP0, _data);
													Process_EP0_OUT_Interrupt();
													break;
									case 1:
													putstr("USB EP0 IN trans interrupt\n");
													/* Обработчик прерывания от EP0 для IN транзакции*/
													break;
									case 2: 
													putstr("USB EP0 OUT trans interrupt\n");
													/* Обработчик прерывания от EP0 для OUT транзакции*/
													break;
								}
								break;
			case EP1:
								switch (USB->SEP1_TS)
								{
									case 0:
													putstr("USB EP1 Setup packet interrupt\n");
													/* Обработчик прерывания от EP1 для setup пакета*/ 
													break;
									case 1:
													putstr("USB EP1 IN trans interrupt\n");
													/* Обработчик прерывания от EP1 для IN транзакции*/
													break;
									case 2: 
													putstr("USB EP1 OUT trans interrupt\n");
													/* Обработчик прерывания от EP1 для OUT транзакции*/
													break;
								}
								break;
			case EP2:
								switch (USB->SEP2_TS)
								{
									case 0:
													putstr("USB EP2 Setup packet interrupt\n");
													/* Обработчик прерывания от EP1 для setup пакета*/ 
													break;
									case 1:
													putstr("USB EP2 IN trans interrupt\n");
													/* Обработчик прерывания от EP1 для IN транзакции*/
													break;
									case 2: 
													putstr("USB EP2 IN trans interrupt\n");
													/* Обработчик прерывания от EP1 для OUT транзакции*/
													break;
								}
								break;
		default:   break;         /* Обработчик любого другого значения EP*/ 		
		
		}
		 if((USB->SIS&0x10)==0x10)  /* Сброс NAK если он был отвечен */
    {
      USB->SIS|=0x10;
    }
      EP_SET_RDY(EP0);
	}	
	else 
		if((USB->SIS&0x04)!=0)	   /*произошло прерывание из-за USB_Reset */
  {
		putstr("USB_Reset\n");
		Setup_USB();
//		USB->SIS&=~0x04;
  }	
}	



void Process_EP0_OUT_Interrupt(void)
{
    unsigned long a;
    unsigned char Buffer[2];
    USB_SETUP_REQUEST SetupPacket;
	// Заполняем структуру SetupPacket из программного буфера
		SetupPacket.bmRequestType = _data[0]; //тип пакета          
    SetupPacket.bRequest = _data[1]; // сам пакет
    SetupPacket.wValue = *((unsigned short *)(&_data[2])); // величина
    SetupPacket.wIndex = *((unsigned short *)(&_data[4])); // индекс
    SetupPacket.wLength = *((unsigned short *)(&_data[6])); // длина

//	bRequest=(((uint16_t)SetupPacket.bmRequestType)<<8)|((uint16_t)SetupPacket.bRequest);

        /* Парсим/распиливаем bmRequestType */
        switch (SetupPacket.bmRequestType & 0x7F) {

            case STANDARD_DEVICE_REQUEST: 
                putstr("Standard Device Request ");
                switch (SetupPacket.bRequest) { 
                    case GET_STATUS:
                        /* Get Status Request to Device should return */
                        /* Remote Wakeup and Self Powered Status */
                        // Ответить на запрос - статусом 
											Buffer[0]=0x01;
											Buffer[1]=0x00;
											USBsendDataEP(EP0,&Buffer, 2); // отправка статуса ( то что наше устройство питается от собственного источника питания и wakeup не поддерживается)
                        break;

                    case CLEAR_FEATURE: break; // ничего не делаем
                    case SET_FEATURE:
                        /* We don't support DEVICE_REMOTE_WAKEUP or TEST_MODE */
                        EP_STALL(EP0); // Stall пакет отправить
                        break;

                    case SET_ADDRESS:
                        putstr("Set Address\n\r");
                        DeviceAddress = SetupPacket.wValue | 0x80; 
                        //отправляем нуль в оконечную точку D11WriteEndpoint(D11_ENDPOINT_EP0_IN, NULL, 0);
												SEND_ZERO_LENGTH_DATA_EPn(EP0);
                        //CtlTransferInProgress = PROGRESS_ADDRESS;
                        break;

                    case GET_DESCRIPTOR:
                        GetDescriptor(&SetupPacket); // отправление дескриптора устройства
                        break;
  
                    case GET_CONFIGURATION:
												USBsendDataEP(EP0, &DeviceConfigured, 1);
                       // отправка конфигурации D11WriteEndpoint(D11_ENDPOINT_EP0_IN, &DeviceConfigured, 1);
                        break;

                    case SET_CONFIGURATION:
                        DeviceConfigured = SetupPacket.wValue & 0xFF; 
                        SEND_ZERO_LENGTH_DATA_EPn(EP0);// отправка в конечную точку нуля D11WriteEndpoint(D11_ENDPOINT_EP0_IN, NULL, 0);
                        if (DeviceConfigured) {
                            //RB3 = 0;
                            putstr("\n\r *** Device Configured *** \n\r");
                            }
                        else {
                            //RB3 = 1; /* Device Not Configured */
                            putstr("\n\r ** Device Not Configured *** \n\r");
                            }
                        break;

                    case SET_DESCRIPTOR: // установка дескриптора не поддерживается
											
                    default:
                        /* Unsupported - Request Error - Stall */
                        EP_STALL(EP0);// отправка STALL ErrorStallControlEndPoint();
                        break;

                }
                break;

            case STANDARD_INTERFACE_REQUEST:           // ***********************Стандартный запрос интерфейса************************
                putstr("Standard Interface Request\n\r");
                switch (SetupPacket.bRequest) {
                        
                    case GET_STATUS:
                        /* Get Status Request to Interface should return */
                        /* Zero, Zero (Reserved for future use) */
                        Buffer[0]=0x01;
												Buffer[1]=0x00;
												USBsendDataEP(EP0,&Buffer, 2); // отправка статуса
                        break;

                    case SET_INTERFACE:
                        /* Device Only supports default setting, Stall may be */
                        /* returned in the status stage of the request */
                        if (SetupPacket.wIndex == 0 && SetupPacket.wValue == 0)  
                            /* Interface Zero, Alternative Setting = 0 */
                            SEND_ZERO_LENGTH_DATA_EPn(EP0);    
                        else EP_STALL(EP0);
                        break;

                    case GET_INTERFACE:
                        if (SetupPacket.wIndex == 0) { /* Interface Zero */
                            
                            SEND_ZERO_LENGTH_DATA_EPn(EP0); // отправка нуля D11WriteEndpoint(D11_ENDPOINT_EP0_IN, Buffer, 1);
                            break;
			} /* else fall through as RequestError */

                    //case CLEAR_FEATURE:
                    //case SET_FEATURE:
                        /* Interface has no defined features. Return RequestError */
                    default:
                       EP_STALL(EP0);
                        break;

                }
                break;
 
            case STANDARD_ENDPOINT_REQUEST:            // ***********************?????? ? ????????? ?????, ???????? ??????????? ????????************************
                putstr("Standard Endpoint Request\n\r");
                switch (SetupPacket.bRequest) {

                    case CLEAR_FEATURE:
                    case SET_FEATURE:
                        /* Halt(Stall) feature required to be implemented on all Interrupt and */
                        /* Bulk Endpoints. It is not required nor recommended on the Default Pipe */
                          
                        if (SetupPacket.wValue == ENDPOINT_HALT)
                        {
                            if (SetupPacket.bRequest == CLEAR_FEATURE) Buffer[0] = 0x00;
                            else                                       Buffer[0] = 0x01;
                            switch (SetupPacket.wIndex & 0xFF) {
                                case 0x01 : 
																						
																//D11CmdDataWrite(D11_SET_ENDPOINT_STATUS + \
                                              D11_ENDPOINT_EP1_OUT, Buffer, 1);
                                            break;
                                case 0x81 : 
																						USBsendDataEP(EP1, &Buffer, 1);
																//D11CmdDataWrite(D11_SET_ENDPOINT_STATUS + \
                                              D11_ENDPOINT_EP1_IN, Buffer, 1);
                                            break;
                                case 0x02 : 
																						
																//D11CmdDataWrite(D11_SET_ENDPOINT_STATUS + \
                                              D11_ENDPOINT_EP2_OUT, Buffer, 1);
                                            break;
                                case 0x82 : 
																						USBsendDataEP(EP2, &Buffer, 1);
																//D11CmdDataWrite(D11_SET_ENDPOINT_STATUS + \
                                              D11_ENDPOINT_EP2_IN, Buffer, 1);
                                            break;
                                case 0x03 : 
																				
																//D11CmdDataWrite(D11_SET_ENDPOINT_STATUS + \
                                              D11_ENDPOINT_EP3_OUT, Buffer, 1);
                                            break;
                                case 0x83 : 
																					USBsendDataEP(EP3, &Buffer, 1);	
																//D11CmdDataWrite(D11_SET_ENDPOINT_STATUS + \
                                              D11_ENDPOINT_EP3_IN, Buffer, 1);
                                            break;
                                default   : /* Invalid Endpoint - RequestError */
                                            EP_STALL(EP0);
                                            break;
                            }
                            SEND_ZERO_LENGTH_DATA_EPn(EP0);
                        } else {
                            /* No other Features for Endpoint - Request Error */
                            EP_STALL(EP0);
                        }
                        break;

                    case GET_STATUS:
                        /* Get Status Request to Endpoint should return */
                        /* Halt Status in D0 for Interrupt and Bulk */
                        switch (SetupPacket.wIndex & 0xFF) {
                            case 0x01 : 
																					getPacketEPn(EP1, Buffer);
																					//D11CmdDataRead(D11_READ_ENDPOINT_STATUS + \
                                          D11_ENDPOINT_EP1_OUT, Buffer, 1);
                                        break;
                            case 0x81 : 
															
																				//D11CmdDataRead(D11_READ_ENDPOINT_STATUS + \
                                          D11_ENDPOINT_EP1_IN, Buffer, 1);
                                        break;
                            case 0x02 : 
																				getPacketEPn(EP2, Buffer);
																				//D11CmdDataRead(D11_READ_ENDPOINT_STATUS + \
                                          D11_ENDPOINT_EP2_OUT, Buffer, 1);
                                        break;
                            case 0x82 : 
															
																					//D11CmdDataRead(D11_READ_ENDPOINT_STATUS + \
                                          D11_ENDPOINT_EP2_IN, Buffer, 1);
                                        break;
                            case 0x03 : 
																					getPacketEPn(EP3, Buffer);
																					//D11CmdDataRead(D11_READ_ENDPOINT_STATUS + \
                                          D11_ENDPOINT_EP3_OUT, Buffer, 1);
                                        break;
                            case 0x83 : 
																				
																				//D11CmdDataRead(D11_READ_ENDPOINT_STATUS + \
                                          D11_ENDPOINT_EP3_IN, Buffer, 1);
                                        break;
                            default   : /* Invalid Endpoint - RequestError */
                                        EP_STALL(EP0);
                                        break;
                        }
                        if (Buffer[0] & 0x08) Buffer[0] = 0x01;
                        else                  Buffer[0] = 0x00;
                        Buffer[1] = 0x00;
                        USBsendDataEP(EP0, Buffer, 2);
												//D11WriteEndpoint(D11_ENDPOINT_EP0_IN, Buffer, 2);
                        break;
                    
                    default:
                        /* Unsupported - Request Error - Stall */
												EP_STALL(EP0);
                        break;
                }
                break;



            case VENDOR_DEVICE_REQUEST:
            case VENDOR_ENDPOINT_REQUEST:                              // ***********************Запрос на определенные действия, в нашем случае это запрос на данные с Манчестера************************
               // putstr("Vendor Device bRequest = 0x%X, wValue = 0x%X, wIndex = 0x%X\n\r", \
                    SetupPacket.bRequest, SetupPacket.wValue, SetupPacket.wIndex);
                switch (SetupPacket.bRequest) {

                    case VENDOR_GET_ANALOG_VALUE:
											/* Заносим в буфер оконечной точки данные из манчестера и отсылаем их на хост */
										 // или
										/* Заносим данные в буфер и отсылаем на оконечную точку для передачи этих данных по манчестеру */
                      

										/* printf("Get Analog Value, Channel %x :",SetupPacket.wIndex & 0x07);
                        ADCON0 = 0xC1 | (SetupPacket.wIndex & 0x07) << 3;            
                        /* Wait Acquistion time of Sample and Hold */
                       /* for (a = 0; a <= 255; a++);
                        ADGO = 1;
                        while(ADGO);
                        Buffer[0] = ADRESL; // ?????? ??????? 8 ?????
                        Buffer[1] = ADRESH; // ?????? ?????????? 2 ??????? ?????
                        a = (Buffer[1] << 8) + Buffer[0]; // ????????? ?? ? 1 ??????????
                        a = (a * 500) / 1024; // ??????? ? ??????
                        printf(" Value = %d.%02d\n\r",(unsigned int)a/100,(unsigned int)a%100); // ????? ?? ?????
                        D11WriteEndpoint(D11_ENDPOINT_EP0_IN, Buffer, 2); // ???????? ?????? ? ????????? ????? 
                        break;
												*/ break;

                    case VENDOR_SET_RB_HIGH_NIBBLE:
                       // putstr("Write High Nibble of PORTB\n\r");
                        //PORTB = (PORTB & 0x0F) | (SetupPacket.wIndex & 0xF0);
                       // D11WriteEndpoint(D11_ENDPOINT_EP0_IN, NULL, 0);
                        break;

                    default:
                        EP_STALL(EP0);
                        break;    
                }
                break;
            default: 
           //    putstr("UnSupported Request Type 0x%X\n\r", SetupPacket.bmRequestType);
                EP_STALL(EP0);
                break;
        }
    } 
	//else 	
		//{
    //putstr("Data Packet?\n\r");
    /* This is a Data Packet */
    //}
//}

void GetDescriptor(PUSB_SETUP_REQUEST SetupPacket)
{
    switch((SetupPacket->wValue & 0xFF00) >> 8) {

        case TYPE_DEVICE_DESCRIPTOR:
           // putstr("\n\rDevice Descriptor: Bytes Asked For %d, Size of Descriptor %d\n\r", \
             //       SetupPacket->wLength,DeviceDescriptor.bLength);
            pSendBuffer = (const unsigned char *)&DeviceDescriptor;
            BytesToSend = DeviceDescriptor.bLength;
            if (BytesToSend > SetupPacket->wLength)
                BytesToSend = SetupPacket->wLength;
            Send_DATA_EP(EP0, pSendBuffer, BytesToSend);// запись в буфер оконечной точки всего дискриптора WriteBufferToEndPoint();
            break;

        case TYPE_CONFIGURATION_DESCRIPTOR:
         //   putstr("\n\rConfiguration Descriptor: Bytes Asked For %d, Size of Descriptor %d\n\r", \
           //         SetupPacket->wLength, sizeof(ConfigurationDescriptor));
            pSendBuffer = (const unsigned char *)&ConfigurationDescriptor;
            BytesToSend = sizeof(ConfigurationDescriptor);
            if (BytesToSend > SetupPacket->wLength)
                BytesToSend = SetupPacket->wLength;
           Send_DATA_EP(EP0, pSendBuffer, BytesToSend);// запись в буфер оконечной точки всего дискриптора WriteBufferToEndPoint();
            break;

        case TYPE_STRING_DESCRIPTOR:
          //  putstr("\n\rString Descriptor: LANGID = 0x%04x, Index %d\n\r", \
            //    SetupPacket->wIndex, SetupPacket->wValue & 0xFF);
            switch (SetupPacket->wValue & 0xFF){

                case 0 ://  pSendBuffer = (const unsigned char *)&LANGID_Descriptor;
                         // BytesToSend = sizeof(LANGID_Descriptor);
                          break;

                case 1 : // pSendBuffer = (const unsigned char *)&Manufacturer_Descriptor;
                         // BytesToSend = sizeof(Manufacturer_Descriptor);
                          break;

                default : pSendBuffer = NULL;
                          BytesToSend = 0;
            }
            if (BytesToSend > SetupPacket->wLength)
                BytesToSend = SetupPacket->wLength;
            Send_DATA_EP(EP0, pSendBuffer, BytesToSend);// запись в буфер оконечной точки всего дискриптора WriteBufferToEndPoint();
            break;
       default:
            // отправка STALL ErrorStallControlEndPoint();
						EP_STALL(EP0);
            break;
    }
}
void Send_DATA_EP(char EP, const unsigned char *pSendBuffer, int count)
{

    if (BytesToSend == 0) { 

         SEND_ZERO_LENGTH_DATA_EPn(EP0);
    } else if (BytesToSend >= 8) {
				USBsendDataEP(EP, pSendBuffer, 8);
        pSendBuffer += 8;
        BytesToSend -= 8;
    } else {
        USBsendDataEP(EP, pSendBuffer, BytesToSend);
        BytesToSend = 0;
    }
}
void Clocks(void)
{
        RST_CLK->HS_CONTROL=0x00000001;           //HSE - включен. Режим осциллятора.
        while((RST_CLK->CLOCK_STATUS&0x04)!=0x04);                  //Ожидаем выхода HSE в рабочий режим
        RST_CLK->CPU_CLOCK=0x00000102;                                    //CPU_C1 = HSE = 8 MHz
        RST_CLK->PLL_CONTROL=(11<<8)|(1<<2);                          //Включаем CPU_PLL, устанавливаем PLL_MULL=11, после выхода в рабочий режим, получим тактовую частоту 96 MHz
        while((RST_CLK->CLOCK_STATUS&0x02)!=0x02);                   //Ожидаем выхода в рабочий режим PLL
        RST_CLK->PER_CLOCK|=(1<<24)|(1<<7)|(1<<6)|(1<<4);                //Разрешаем тактирование периферийных блоков
       
        RST_CLK->UART_CLOCK|=(1<<24)|(1<<25); //0x3000000;               //Разрешение тактирования UART1, UART2, источники тактирования для  UART1_CLK=HCLK, UART2_CLK=HCLK
        RST_CLK->CPU_CLOCK |= 0x00000106;                                   // Выберем источник для CPU_C1 = HSE (CPU Clock = 12*8 MHz = 96 MHz) HCLK=PLL*CPUC_1
}
void PerClkConfig(void)
{
	RST_CLK->PER_CLOCK	|=(1 << 24) /*PORTD*/ |
												(1 << 7) /*UART2*/;
	
	RST_CLK->UART_CLOCK=(1 << 25); /*UART2*/	
}
void Setup_USB(void)
{

uint32_t i;

RST_CLK->PER_CLOCK |= 0x00000004; // разрешение тактирования USB
RST_CLK->USB_CLOCK|=0x00000100|0x00000003;	 /* тактовая частота - HSE/2, разрешение тактирования - USBCLKEN*/	 
RST_CLK->PLL_CONTROL|=(11<<4)|0x00000001; // коэффициент умножения частоты = 11+1=12, разрешение тактирования USB
while (!(RST_CLK->CLOCK_STATUS&0x00000001)); // проверка флага выхода USB PLL в стабильный режим работы
RST_CLK->USB_CLOCK|=0x00000004;	/*выбор PLLUSBo как источника тактирования для ЮСБ*/
	
USB->HSCR|=0x00000002;          /*бит сброса USB_CORE*/
for (i=0;i<1000;i++){}
USB->HSCR&=~0x00000002; // убираем бит сброса
	
USB->HSCR|=0x0000001C;			/*4c если DN PULLUP, 1C если DP pullup,EnRX,EnTX*/
USB->SC|=0x00000001; // задание параметров шины в режиме Device ( скорость) SCGEN - разрешение работы с оконечных точек ,скорость LOW, 1,5Мбит/с
	
while (!(USB->SIS&0x00000004));	 /*ожидание сброса на шине*/
USB->SIS = 0xFF; // сбрасываем все флаги установкой 1
	
USB->SEP2_CTRL|= 3;		   /*разрешение конечных точек(EPEN,EPRDY биты) 2-ая точка*/
USB->SEP2_RXFC=1; // очищаем указатель FIFO 2 точка

USB->SEP1_CTRL|= 3; // разрешение конечной точки (EPEN, EPRDY биты) 1 точка
USB->SEP1_RXFC|=1; // очищаем указатель FIFO 1 точка
	
USB->SEP0_CTRL|= 1;		  // разрешение конечной точки (EPEN, EPRDY биты) 0 точка
USB->SEP0_RXFC|= 1; // разрешение конечной точки (EPEN, EPRDY биты) 0 точка
	
USB->SIM|=0x05;				/*разрешение прерывания по окончанию передачи и по reset*/
NVIC_EnableIRQ(USB_IRQn); // глобальное разрешение прерываний
}
