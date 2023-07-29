#ifndef __LCD_H
#define __LCD_H

#include "main.h"

#define u8  unsigned char 
#define u16 unsigned int 

/////////////////////////////////////�û�������///////////////////////////////////	 
//֧�ֺ��������ٶ����л�
#define USE_HORIZONTAL  		0	//�����Ƿ�ʹ�ú��� 		0,��ʹ��.1,ʹ��.

//-----------------------------SPI ��������--------------------------------------//
#define USE_HARDWARE_SPI     0  //1:Enable Hardware SPI;0:USE Soft SPI

//-------------------------��Ļ������������--------------------------------------//
#define LCD_X_SIZE	        240
#define LCD_Y_SIZE	        320

#if USE_HORIZONTAL//��������˺��� 
#define X_MAX_PIXEL	        LCD_Y_SIZE
#define Y_MAX_PIXEL	        LCD_X_SIZE
#else
#define X_MAX_PIXEL	        LCD_X_SIZE
#define Y_MAX_PIXEL	        LCD_Y_SIZE
#endif
//////////////////////////////////////////////////////////////////////////////////
	 

#define RED  	0xf800
#define GREEN	0x07e0
#define BLUE 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   	//��ɫ0 3165 00110 001011 00101
#define GRAY1   0x8410      	//��ɫ1      00000 000000 00000
#define GRAY2   0x4208      	//��ɫ2  1111111111011111

//define LCD PIN
#define	LCD_CS_SET  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET)
#define	LCD_CS_CLR  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET)

#define	LCD_RS_SET  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET)
#define	LCD_RS_CLR  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET)

#define	LCD_SDA_SET  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET)
#define	LCD_SDA_CLR  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET)

#define	LCD_SCL_SET  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET)
#define	LCD_SCL_CLR  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET)

#define	LCD_RST_SET  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)
#define	LCD_RST_CLR  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)

#define	LCD_LED_SET  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET)
#define	LCD_LED_CLR  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET)


void Lcd_Reset(void);
void Lcd_WriteIndex(u8 Index);
void Lcd_WriteData(u8 Data);
void Lcd_WriteReg(u8 Index,u8 Data);
u16 Lcd_ReadReg(u8 LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(u16 Color);
void Lcd_SetXY(u16 x,u16 y);
void Gui_DrawPoint(u16 x,u16 y,u16 Data);
unsigned int Lcd_ReadPoint(u16 x,u16 y);
void Lcd_SetRegion(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd);
void Lcd_WriteData_16Bit(u16 Data);


#endif