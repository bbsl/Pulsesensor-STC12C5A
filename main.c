//******************************����˵��*********************************//
//MCU:STC12C5A60S2
//ADC PIN:P1.0
//SYSTEM CLOCK:11.0592MHz
//Baudrate:115200
//UART:P3.0 P3.1
//**********************************************************************//
#include "stc12c5a.h"
#include "stdio.h"
#include "oled.h"
#include "stdlib.h"
#include "string.h"

#define false 0
#define true 1
#define FOSC 11059200L	 		//ϵͳʱ��
#define BAUD 115200				//������
#define T0MS (65536-FOSC/12/500)//��ʱ����ֵ
//ADC����
#define ADC_POWER 0x80			
#define ADC_FLAG 0x10		
#define ADC_START 0x08;			//ADCʹ�ܿ���
#define ADC_SPEEDLL 0x00		//540
#define ADC_SPEEDL 0x20			//360
#define ADC_SPEEDH 0x40			//180 
#define ADC_SPEEDHH 0x60		//90
#define ADC_MASK 0x01

void UART_init(void); 
void ADC_init(unsigned char channel);
void T0_init(void);
void sendDataToProcessing(char symbol, int dat);
void UART_send(char dat);


unsigned char PulsePin = 0;       
int fadeRate = 0;                 			  //led��


// ȫ�ֱ������ж���ʹ�ã������ĵ��ź����ݴ���
volatile unsigned int BPM;                   //��������ֵ
volatile unsigned int Signal;                //ԭʼ�ź�����
volatile unsigned int IBI = 600;             //��¼�������
volatile bit Pulse        = false;     		 //��ǲ��岨��
volatile bit QS           = false;       	 //���ʳ��ֱ�־
volatile int rate[10];                    	 //�����ʮ��IBIֵ
volatile unsigned long sampleCounter = 0;    //ȷ������
volatile unsigned long lastBeatTime = 0;     //���ڼ���IBI
volatile int Peak          = 512;            //����
volatile int Trough        = 512;            //�Ҳ���
volatile int thresh        = 512;            //������
volatile int amp           = 100;            //���
volatile bit firstBeat     = true;        	 //ǰһ��
volatile bit secondBeat    = false;      	 //��һ��
static unsigned char order = 0;
unsigned char DisBuff[4]   = {0};			 //�������ݣ�������ʾ���ϴ�
unsigned char DisIBI[5]    = {0};

void sys_init()
{
	OLED_Init();   			//��ʼ��Һ��
 	UART_init();			//���ڳ�ʼ��
	ADC_init(PulsePin);		//ADC��ʼ��
 	T0_init();				//��ʱ����ʼ����2msһ�� 
}

void main(void)
{
  	sys_init();
	OLED_P8x16Str(0,0,"BPM:");
	OLED_P8x16Str(80,0,"t/min");
	OLED_P8x16Str(0,4,"IBI:");
	OLED_P8x16Str(80,4,"Ms");
	while(1)
	{
		sendDataToProcessing('S', Signal);     // �������ݵ���λ��
		if (QS == true)
		{                 
			fadeRate = 255;                  //LED����
			sendDataToProcessing('B',BPM);   //�������ʣ�B��ͷ
			sendDataToProcessing('Q',IBI);   //�������ڣ�Q��ͷ
			QS = false;                      //ÿ�η�����ɺ����ñ�־
			OLED_P8x16Str(40,0,DisBuff);		 //��ʾ
			OLED_P8x16Str(40,4,DisIBI);		 //��ʾ
		}
		delay(138);                         
	}
}
/************����λ��������**************/
void sendDataToProcessing(char symbol, int dat )
{
    putchar(symbol);                	// ��֪��λ����������
	printf("%d\r\n",dat);			// �ϴ�����
}
/***********���ڳ�ʼ��******************/
void UART_init(void)
{
	PCON &= 0x7f;  //�����ʲ�����
	SCON = 0x50;   //8λ���ݣ��ɱ䲨����
	BRT = 0xFD;    //���������ʲ�������ֵ
	AUXR |= 0x04;  //ʱ������Ϊ1Tģʽ
 	AUXR |= 0x01;  //ѡ����������ʲ�����
	AUXR |= 0x10;  //���������ʲ���
}
/***********���ڷ������ݵĺ���***********/
char putchar(unsigned char dat)
{
	TI=0;
	SBUF=dat;
	while(!TI);
	TI=0;
	
	return SBUF;
}
/************��ʱ����ʼ��2ms�ж�һ��****************/
void T0_init(void){     
  
	TMOD |= 0x01;	
	TL0=T0MS;
	TH0=T0MS>>8;
	TR0=1;
	ET0=1;	
	EA=1;             
}
/**************ADC��ʼ��********************/
void ADC_init(unsigned char channel)
{
	P1ASF=ADC_MASK<<channel;	//ʹ��ADCͨ��
	ADC_RES=0;					//�����־
	ADC_RESL=0;	
	AUXR1 |= 0x04;				//���
	ADC_CONTR=channel|ADC_POWER|ADC_SPEEDLL|ADC_START;	//��ʼ
}
/************ģ���źų�ʼ��************/
unsigned int analogRead(unsigned char channel)
{
	unsigned int result;

	ADC_CONTR &=!ADC_FLAG;	//���ADC��־
	result=ADC_RES;
	result=result<<8;
	result+=ADC_RESL;
	ADC_CONTR|=channel|ADC_POWER|ADC_SPEEDLL|ADC_START;
	return result;
}
/*****************************��Ҫ����**********************************/
/*********Timer 0�ж��ӳ���ÿ2MS�ж�һ�Σ���ȡADֵ����������ֵ********/
void Timer0_rountine(void) interrupt 1
{                       
	int N;
	unsigned char i;
	unsigned int runningTotal = 0;  		   //ʮ��������϶              

	EA=0;                                      //���жϣ���ֹ�ٴν����ж�
	TL0=T0MS;
	TH0=T0MS>>8;							   //��װ����ֵ
	Signal = analogRead(PulsePin);             //��ȡ�������ź� 
	sampleCounter += 2;                        //����
	N = sampleCounter - lastBeatTime;          //��Ϊ���һ���������ʱ��ɱ�������


    //�����ĵ��źŵĲ���Ͳ���
	if(Signal < thresh && N > (IBI/5)*3)
	{       									
		if (Signal < Trough)
		{                        				//����
      		Trough = Signal;                    //��¼����ֵ
    	}
  	}

  	if(Signal > thresh && Signal > Peak)
	{          									
    	Peak = Signal;                          //����
  	}                                        	//��¼����ֵ


  	//��������
  	if (N > 250)
	{                                   			//���ò��岨��������Ƶ����
    	if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) )
		{        
      		Pulse = true;                           //�ҵ����壬��־λ��λ
      		IBI = sampleCounter - lastBeatTime;     //�����ʱ��
      		lastBeatTime = sampleCounter;           //��һ�����嵽��ʱ��

      		if(secondBeat)
			{//����
        		secondBeat = false;                  	//��־λ��λ
        		for(i=0; i<=9; i++)
				{            							//��¼IBI
         	 		rate[i] = IBI;                      
        		}
      		}

      		if(firstBeat)
			{//���ĵ���
       		 	firstBeat = false;                   //�����־λ
       		 	secondBeat = true;                   //��һ�����ı�־λ
        		EA=1;                                //���жϣ����������һ�������ź�
        		return;                              //��ʱIBI��Ч
      		}

			for(i=0; i<=8; i++)
			{
        		rate[i] = rate[i+1];                   
        		runningTotal += rate[i];             
      		}

      		rate[9] = IBI;                          //����µ���ЧIBIֵ

			DisIBI[3] = IBI%10+48;					//ȡ��λ��,��ת�����ַ�
			DisIBI[2] = IBI%100/10+48;				//ȡʮλ��
			DisIBI[1] = IBI%1000/100+48;			//ȡ��λ��
			DisIBI[0] = IBI/1000+48;				//ȡǧλ��

      		runningTotal += rate[9];                
     	 	runningTotal /= 10;                     //����IBIƽ��ֵ
     	 	BPM = 60000/runningTotal;               //��������
			if(BPM>200) BPM=200;					//����BPM�����ʾֵ
			if(BPM<30)  BPM=30;						//����BPM�����ʾֵ

			DisBuff[2] = BPM%10+48;					//ȡ��λ��,��ת�����ַ�
			DisBuff[1] = BPM%100/10+48;				//ȡʮλ��
			DisBuff[0] = BPM/100+48;	   			//��λ��

			if(DisBuff[0]==48)
				DisBuff[0]=32;
    		QS = true;                              //���ʵó��ı�־λ��λ
		}                       
	}

  	if (Signal < thresh && Pulse == true)
  	{//�����źŽ���
    	Pulse = false;                    //��λ
    	amp = Peak - Trough;              //�������
    	thresh = amp/2 + Trough;          //��threah����Ϊ�����һ��
    	Peak = thresh;           
    	Trough = thresh;
  	}

	if (N > 2500)
	{//�ȴ��źŵ�ʱ�䵽�����źŵ���
    	thresh = 512;                     //Ĭ��
    	Peak = 512;                       //Ĭ��
    	Trough = 512;                     //Ĭ��
    	lastBeatTime = sampleCounter;     //Ĭ��       
    	firstBeat = true;                 //Ĭ��
    	secondBeat = false;               //Ĭ��
	}

  EA=1;                               //�жϴ�����ɺ󣬴��ж�
}