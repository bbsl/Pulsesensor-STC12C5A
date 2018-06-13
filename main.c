//******************************参数说明*********************************//
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
#define FOSC 11059200L	 		//系统时钟
#define BAUD 115200				//波特率
#define T0MS (65536-FOSC/12/500)//定时器初值
//ADC参数
#define ADC_POWER 0x80			
#define ADC_FLAG 0x10		
#define ADC_START 0x08;			//ADC使能控制
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
int fadeRate = 0;                 			  //led灯


// 全局变量，中断中使用，进行心电信号数据处理
volatile unsigned int BPM;                   //保存心率值
volatile unsigned int Signal;                //原始信号数据
volatile unsigned int IBI = 600;             //记录心跳间隔
volatile bit Pulse        = false;     		 //标记波峰波谷
volatile bit QS           = false;       	 //心率出现标志
volatile int rate[10];                    	 //保存后十个IBI值
volatile unsigned long sampleCounter = 0;    //确定脉冲
volatile unsigned long lastBeatTime = 0;     //用于计算IBI
volatile int Peak          = 512;            //波峰
volatile int Trough        = 512;            //找波谷
volatile int thresh        = 512;            //找心跳
volatile int amp           = 100;            //振幅
volatile bit firstBeat     = true;        	 //前一次
volatile bit secondBeat    = false;      	 //后一次
static unsigned char order = 0;
unsigned char DisBuff[4]   = {0};			 //数字数据，用于显示和上传
unsigned char DisIBI[5]    = {0};

void sys_init()
{
	OLED_Init();   			//初始化液晶
 	UART_init();			//串口初始化
	ADC_init(PulsePin);		//ADC初始化
 	T0_init();				//定时器初始化，2ms一次 
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
		sendDataToProcessing('S', Signal);     // 发生数据到上位机
		if (QS == true)
		{                 
			fadeRate = 255;                  //LED亮度
			sendDataToProcessing('B',BPM);   //发送心率，B开头
			sendDataToProcessing('Q',IBI);   //发送周期，Q开头
			QS = false;                      //每次发送完成后都重置标志
			OLED_P8x16Str(40,0,DisBuff);		 //显示
			OLED_P8x16Str(40,4,DisIBI);		 //显示
		}
		delay(138);                         
	}
}
/************向上位机传数据**************/
void sendDataToProcessing(char symbol, int dat )
{
    putchar(symbol);                	// 告知上位机数据类型
	printf("%d\r\n",dat);			// 上传数据
}
/***********串口初始化******************/
void UART_init(void)
{
	PCON &= 0x7f;  //波特率不倍速
	SCON = 0x50;   //8位数据，可变波特率
	BRT = 0xFD;    //独立波特率产生器初值
	AUXR |= 0x04;  //时钟设置为1T模式
 	AUXR |= 0x01;  //选择独立波特率产生器
	AUXR |= 0x10;  //启动波特率产生
}
/***********串口发送数据的函数***********/
char putchar(unsigned char dat)
{
	TI=0;
	SBUF=dat;
	while(!TI);
	TI=0;
	
	return SBUF;
}
/************定时器初始化2ms中断一次****************/
void T0_init(void){     
  
	TMOD |= 0x01;	
	TL0=T0MS;
	TH0=T0MS>>8;
	TR0=1;
	ET0=1;	
	EA=1;             
}
/**************ADC初始化********************/
void ADC_init(unsigned char channel)
{
	P1ASF=ADC_MASK<<channel;	//使能ADC通道
	ADC_RES=0;					//清除标志
	ADC_RESL=0;	
	AUXR1 |= 0x04;				//检查
	ADC_CONTR=channel|ADC_POWER|ADC_SPEEDLL|ADC_START;	//开始
}
/************模拟信号初始化************/
unsigned int analogRead(unsigned char channel)
{
	unsigned int result;

	ADC_CONTR &=!ADC_FLAG;	//清除ADC标志
	result=ADC_RES;
	result=result<<8;
	result+=ADC_RESL;
	ADC_CONTR|=channel|ADC_POWER|ADC_SPEEDLL|ADC_START;
	return result;
}
/*****************************主要函数**********************************/
/*********Timer 0中断子程序，每2MS中断一次，读取AD值，计算心率值********/
void Timer0_rountine(void) interrupt 1
{                       
	int N;
	unsigned char i;
	unsigned int runningTotal = 0;  		   //十个心跳间隙              

	EA=0;                                      //关中断，防止再次进入中断
	TL0=T0MS;
	TH0=T0MS>>8;							   //重装计数值
	Signal = analogRead(PulsePin);             //读取传感器信号 
	sampleCounter += 2;                        //计数
	N = sampleCounter - lastBeatTime;          //因为最后一次心跳监测时间可避免噪音


    //查找心电信号的波峰和波谷
	if(Signal < thresh && N > (IBI/5)*3)
	{       									
		if (Signal < Trough)
		{                        				//波谷
      		Trough = Signal;                    //记录波谷值
    	}
  	}

  	if(Signal > thresh && Signal > Peak)
	{          									
    	Peak = Signal;                          //波峰
  	}                                        	//记录波峰值


  	//计算脉冲
  	if (N > 250)
	{                                   			//利用波峰波谷消除高频噪声
    	if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) )
		{        
      		Pulse = true;                           //找到脉冲，标志位置位
      		IBI = sampleCounter - lastBeatTime;     //测节拍时间
      		lastBeatTime = sampleCounter;           //下一次脉冲到来时间

      		if(secondBeat)
			{//节拍
        		secondBeat = false;                  	//标志位复位
        		for(i=0; i<=9; i++)
				{            							//记录IBI
         	 		rate[i] = IBI;                      
        		}
      		}

      		if(firstBeat)
			{//节拍到达
       		 	firstBeat = false;                   //清除标志位
       		 	secondBeat = true;                   //下一个节拍标志位
        		EA=1;                                //开中断，允许接收下一个脉冲信号
        		return;                              //此时IBI无效
      		}

			for(i=0; i<=8; i++)
			{
        		rate[i] = rate[i+1];                   
        		runningTotal += rate[i];             
      		}

      		rate[9] = IBI;                          //添加新的有效IBI值

			DisIBI[3] = IBI%10+48;					//取个位数,并转换成字符
			DisIBI[2] = IBI%100/10+48;				//取十位数
			DisIBI[1] = IBI%1000/100+48;			//取百位数
			DisIBI[0] = IBI/1000+48;				//取千位数

      		runningTotal += rate[9];                
     	 	runningTotal /= 10;                     //计算IBI平均值
     	 	BPM = 60000/runningTotal;               //计算心率
			if(BPM>200) BPM=200;					//限制BPM最高显示值
			if(BPM<30)  BPM=30;						//限制BPM最低显示值

			DisBuff[2] = BPM%10+48;					//取个位数,并转换成字符
			DisBuff[1] = BPM%100/10+48;				//取十位数
			DisBuff[0] = BPM/100+48;	   			//百位数

			if(DisBuff[0]==48)
				DisBuff[0]=32;
    		QS = true;                              //心率得出的标志位置位
		}                       
	}

  	if (Signal < thresh && Pulse == true)
  	{//本次信号结束
    	Pulse = false;                    //复位
    	amp = Peak - Trough;              //计算振幅
    	thresh = amp/2 + Trough;          //将threah设置为振幅的一半
    	Peak = thresh;           
    	Trough = thresh;
  	}

	if (N > 2500)
	{//等待信号的时间到，无信号到来
    	thresh = 512;                     //默认
    	Peak = 512;                       //默认
    	Trough = 512;                     //默认
    	lastBeatTime = sampleCounter;     //默认       
    	firstBeat = true;                 //默认
    	secondBeat = false;               //默认
	}

  EA=1;                               //中断处理完成后，打开中断
}