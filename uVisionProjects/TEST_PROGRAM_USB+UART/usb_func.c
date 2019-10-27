#include <opora.h>
#include <usb_func.h>

void USBsendDataEP(char EPn, const unsigned char *data, int count) // ������� �������� ������ �� USB
{
    int i;

    EP_TX_FIFO_FORCE_EMPTY(EPn); // ����� ��������� FIFO
    EP_ENABLE(EPn); // ��������� ����� ���������
//     EP_ANS_WITH_DATA0(EPin); // �������� �� IN-������ � DATA0
    for (i=0;i<count;i++)
    {
		EP_FIFO_LOAD_BYTE(EPn, data[i]); // ������� ������ �� ������ data � FIFO
        if ((i+1)%MAXPACKET64 == 0 )
        {
            EP_SET_RDY(EPn); // ���������� ��. �����
            while (GET_TDONE()==0); // ���� ��������� ������
						
            while ( EP_GET_ACKRXED(EPn)==0);  // waiting for ACK from host
            
						putstr("ACKrxed"); // �������� ����� - �������� � ��������
            CLEAR_TDONE();
            EP_INVERS_DATASEQ(EPn); //�������������� ��������� ������ IN �� ������ ����� Data1/Data0
				}
		EP_SET_RDY(EPn);					//������� �� IN ����� �������
		while (GET_TDONE()==0);// ���� ��������� ���������� ��������
		while (EP_GET_ACKRXED(EPn)==0 );  // waiting for ACK from host
		CLEAR_TDONE();
		EP_INVERS_DATASEQ(EPn); //�������������� ��������� ������ IN �� ������ ����� Data1/Data0
		}
}
void SEND_ZERO_LENGTH_DATA_EPn(char EPn)   // ������� �������� ������� ������ ������
{
    EP_TX_FIFO_FORCE_EMPTY(EPn); // ����� ��������� ������ FIFO
    EP_ENABLE(EPn); // ��������� ����� ���������
    EP_ANS_WITH_DATA1(EPn); // �������� �� IN-������ � DATA1
    EP_SET_RDY(EPn); // ��������� ���������� ������
    while (GET_TDONE()==0);		  //�������� �������� �������� ������

    while (EP_GET_ACKRXED(EPn)==0);  // waiting for ACK from host

    CLEAR_TDONE();
    EP_INVERS_DATASEQ(EPn);
	  EP_ENABLE(EPn);
	
}
void EP_STALL(uint8_t EPn)
{
//  EP_TX_FIFO_FORCE_EMPTY(EPn); // ���� �����������
//    EP_ANS_WITH_DATA1(EPn); // ���� �����������
    EP_ENABLE(EPn);
	  EP_SET_STALL(EPn);
    EP_SET_RDY(EPn);
		if (EPn==EP0)
    while (!(USB->SEP0_STS|0x20)); /*stallsent*/
		
		if (EPn==EP1)
    while (!(USB->SEP1_STS|0x20)); /*stallsent*/

		if (EPn==EP2)
    while (!(USB->SEP2_STS|0x20)); /*stallsent*/
		
	  EP_CLR_STALL(EPn);
    EP_TX_FIFO_FORCE_EMPTY(EPn); // ���� �����������
    CLEAR_TDONE();
    EP_INVERS_DATASEQ(EPn); // ���� �����������
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
		EP_SET_RDY0;
	
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

char EP_FIFO_READ_BYTE(char EP)
{
	char out;
	if (EP==0)
		out=EP_FIFO_READ_BYTE0;
	
	if (EP==1)
		out=EP_FIFO_READ_BYTE1;
	
	if (EP==2)
		out=EP_FIFO_READ_BYTE2;
	return out;
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
char EP_GET_ACKRXED(char EP)
{	
	char out;
	if (EP==0)
	out=EP_GET_ACKRXED0;
	
	if (EP==1)
		out = EP_GET_ACKRXED1;
	
	if (EP==2)
	out =	EP_GET_ACKRXED2;
	return out;
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
char EP_Get_count(char EP)
{
	char out;
	if (EP==0)
	out=EP_Get_count0;
	
	if (EP==1)
		out=EP_Get_count1;
	
	if (EP==2)
		out=EP_Get_count2;
}
/*char* EP_Get_data(char *EPbuff, char EP)
{
	
		if (EP==0)
	EP_Get_data0(EPbuff);
	
	if (EP==1)
		EP_Get_data1(EPbuff);
	
	if (EP==2)
		EP_Get_data2(EPbuff);
	
	return EPbuff;
} */
unsigned char GetTS(char EP)
{	
	unsigned char TS;
	if (EP==0)
	TS=USB->SEP0_TS; // ��������� ��� ��������� ��������
	
	if (EP==1)
		TS=USB->SEP1_TS; // ��������� ��� ��������� ��������
	
	if (EP==2)
		TS=USB->SEP2_TS; // ��������� ��� ��������� ��������
	
	return TS;
}

unsigned char GetCTRL(char EP)
{	
	unsigned char CTRL;
	if (EP==0)
	CTRL=USB->SEP0_CTRL; // ��������� ��������� �����: ��� ��������, ���������� � �.�
	
	if (EP==1)
		CTRL=USB->SEP1_CTRL; // ��������� ��������� �����: ��� ��������, ���������� � �.�
	
	if (EP==2)
		CTRL=USB->SEP2_CTRL; // ��������� ��������� �����: ��� ��������, ���������� � �.�
	
	return CTRL;
}

unsigned char GetSTS(char EP)
{	
	unsigned char STS;
	if (EP==0)
	STS=USB->SEP0_STS; // ��������� ��������� �����
	
	if (EP==1)
		STS=USB->SEP1_STS; // ��������� ��������� �����
	
	if (EP==2)
		STS=USB->SEP2_STS; // ��������� ��������� �����
	
	return STS;
}





		