#ifndef _OLED_H_
#define _OLED_H_

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stc12c5a.h"
#include "oledfont.h"


#define uint unsigned int
#define uchar unsigned char
/*
* Function List:
*
* 1. static void delay(uint z) -- ��ʱ����,����
* 2. static void IIC_Start() -- ����I2C����
* 3. static void IIC_Stop() -- �ر�I2C����
* 4. static void Write_IIC_Byte(uchar IIC_Byte) -- ͨ��I2C����дһ��byte������
* 5. static void OLED_WrDat(uchar dat) -- ��OLED��д����
* 6. static void OLED_WrCmd(uchar cmd) -- ��OLED��д����
* 7. static void OLED_Set_Pos(uchar x, uchar y) -- ������ʾ����
* 8. static void OLED_Fill(uchar bmp_dat) -- ȫ����ʾ(��ʾBMPͼƬʱ�Ż��õ��˹���)
* 9. void OLED_CLS(void) -- ��λ/����
* 10. void OLED_Init(void) -- OLED����ʼ�����򣬴˺���Ӧ�ڲ�����Ļ֮ǰ���ȵ���
* 11. void OLED_P6x8Str(uchar x,uchar y,uchar ch[]) -- 6x8������������ʾASCII�����С���У���̫����
* 12. void OLED_P8x16Str(uchar x,uchar y,uchar ch[]) -- 8x16������������ʾASCII�룬�ǳ�����
* 13. static char* my_itoa(int value,char *str,int radix);  
* 14. char *my_ftoa(double number,int ndigit,char *buf);//������ת�ַ��� 
*************************************************************************************/
// ------------------------------------------------------------
// IO��ģ��I2Cͨ��
// SCL��P1^3
// SDA��P1^2
// ------------------------------------------------------------

#define high 1
#define low 0

#define	Brightness	0xCF 
#define X_WIDTH 	128
#define Y_WIDTH 	64

sbit SCL=P1^1; //����ʱ��
sbit SDA=P1^2; //��������
/*********************OLED���������õ���ʱ����************************************/
static void delay(uint z)
{
	uint x,y;
	for(x=z;x>0;x--)
		for(y=110;y>0;y--);
}

/**********************************************
//IIC Start
**********************************************/
static void IIC_Start()
{
   SCL = high;		
   SDA = high;
   SDA = low;
   SCL = low;
}

/**********************************************
//IIC Stop
**********************************************/
static void IIC_Stop()
{
   SCL = low;
   SDA = low;
   SCL = high;
   SDA = high;
}

/**********************************************
// ͨ��I2C����дһ���ֽ�
**********************************************/
static void Write_IIC_Byte(uchar IIC_Byte)
{
	uchar i;
	for(i=0;i<8;i++)
	{
		if(IIC_Byte & 0x80)
			SDA=high;
		else
			SDA=low;
		SCL=high;
		SCL=low;
		IIC_Byte<<=1;
	}
	SDA=1;
	SCL=1;
	SCL=0;
}

/*********************OLEDд����************************************/ 
static void OLED_WrDat(uchar IIC_Data)
{
	IIC_Start();
	Write_IIC_Byte(0x78);
	Write_IIC_Byte(0x40);			//write data
	Write_IIC_Byte(IIC_Data);
	IIC_Stop();
}
/*********************OLEDд����************************************/
static void OLED_WrCmd(uchar IIC_Command)
{
	IIC_Start();
	Write_IIC_Byte(0x78);            //Slave address,SA0=0
	Write_IIC_Byte(0x00);			//write command
	Write_IIC_Byte(IIC_Command);
	IIC_Stop();
}
/*********************OLED ��������************************************/
static void OLED_Set_Pos(uchar x, uchar y) 
{ 
	OLED_WrCmd(0xb0+y);
	OLED_WrCmd(((x&0xf0)>>4)|0x10);
	OLED_WrCmd((x&0x0f)|0x01);
} 
/*********************OLEDȫ��************************************/
static void OLED_Fill(uchar bmp_dat) 
{
	uchar y,x;
	for(y=0;y<8;y++)
	{
		OLED_WrCmd(0xb0+y);
		OLED_WrCmd(0x01);
		OLED_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
		OLED_WrDat(bmp_dat);
	}
}
/*********************OLED��λ/����************************************/
void OLED_CLS(void)
{
	uchar y,x;
	for(y=0;y<8;y++)
	{
		OLED_WrCmd(0xb0+y);
		OLED_WrCmd(0x01);
		OLED_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
		OLED_WrDat(0);
	}
}
/*********************OLED��ʼ��************************************/
void OLED_Init(void)
{
	delay(500);//��ʼ��֮ǰ����ʱ����Ҫ��
	OLED_WrCmd(0xae);//--turn off oled panel
	OLED_WrCmd(0x00);//---set low column address
	OLED_WrCmd(0x10);//---set high column address
	OLED_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WrCmd(0x81);//--set contrast control register
	OLED_WrCmd(Brightness); // Set SEG Output Current Brightness
	OLED_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
	OLED_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
	OLED_WrCmd(0xa6);//--set normal display
	OLED_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
	OLED_WrCmd(0x3f);//--1/64 duty
	OLED_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WrCmd(0x00);//-not offset
	OLED_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
	OLED_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WrCmd(0xd9);//--set pre-charge period
	OLED_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WrCmd(0xda);//--set com pins hardware configuration
	OLED_WrCmd(0x12);
	OLED_WrCmd(0xdb);//--set vcomh
	OLED_WrCmd(0x40);//Set VCOM Deselect Level
	OLED_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WrCmd(0x02);//
	OLED_WrCmd(0x8d);//--set Charge Pump enable/disable
	OLED_WrCmd(0x14);//--set(0x10) disable
	OLED_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
	OLED_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7) 
	OLED_WrCmd(0xaf);//--turn on oled panel
	OLED_Fill(0x00); //��ʼ����
	OLED_Set_Pos(0,0);
} 
/***************������������ʾ6*8һ���׼ASCII�ַ���	��ʾ�����꣨x,y����yΪҳ��Χ0��7****************/
void OLED_P6x8Str(uchar x,uchar y,uchar ch[])
{
	uchar c=0,i=0,j=0;
	while (ch[j]!='\0')
	{
		c =ch[j]-32;
		if(x>126){x=0;y++;}
		OLED_Set_Pos(x,y);
		for(i=0;i<6;i++)
		OLED_WrDat(F6x8[c][i]);
		x+=6;
		j++;
	}
}
/*******************������������ʾ8*16һ���׼ASCII�ַ���	 ��ʾ�����꣨x,y����yΪҳ��Χ0��7****************/
void OLED_P8x16Str(uchar x, uchar y,uchar ch[])
{
	uchar c=0,i=0,j=0;
	while (ch[j]!='\0')
	{
		c =ch[j]-32;
		if(x>120){x=0;y++;}
		OLED_Set_Pos(x,y);
		for(i=0;i<8;i++)
		OLED_WrDat(F8X16[c*16+i]);
		OLED_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
		OLED_WrDat(F8X16[c*16+i+8]);
		x+=8;
		j++;
	}
}

char *F2S(double d, char* str)
{
  char str1[40];
  int j=0,k,i;
  i = (int)d;  //����������������
  //d = d-(int)d;
  while(i>0)
  	{
		str1[j++] = i%10+'0';
    	i /= 10;
  	}
  for(k=0;k<j;k++)
    str[k] = str1[j-1-k]; //

  str[j++] = '.';
  d -= (int)d;
  for(i=0;i<10;i++)
  	{
    	d *= 10;
    	str[j++] = (int)d+'0';
    	d -= (int)d;
  	}
  while(str[--j]=='0');
    str[++j] = '\0';
  return str;
}								  
#endif