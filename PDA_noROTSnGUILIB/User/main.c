/*******************************************************************************
*                 
*                 		       ���пƼ�
--------------------------------------------------------------------------------
* ʵ �� ��		 : SPI-FLASHʵ��
* ʵ��˵��       : 
* ���ӷ�ʽ       : 
* ע    ��		 : FLASH������flash.c��
*******************************************************************************/

#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "tftlcd.h"
#include "key.h"
#include "spi.h"
#include "flash.h"


//Ҫд�뵽25Q64���ַ�������
const u8 text_buf[]="www.prechin.net";
#define TEXT_LEN sizeof(text_buf)


int main()
{
	//u8 i=0;
	u8 key;
	u8 buf[30];
	
	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�ж����ȼ����� ��2��
	LED_Init();
	USART1_Init(115200);
	TFTLCD_Init();			//LCD��ʼ��
	KEY_Init();
	EN25QXX_Init();
	
	FRONT_COLOR=BLACK;
	LCD_ShowString(10,10,tftlcd_data.width,tftlcd_data.height,16,"PRECHIN STM32F4");
	LCD_ShowString(10,30,tftlcd_data.width,tftlcd_data.height,16,"www.prechin.net");
	LCD_ShowString(10,50,tftlcd_data.width,tftlcd_data.height,16,"FLASH-SPI Test");
	LCD_ShowString(10,70,tftlcd_data.width,tftlcd_data.height,16,"K_UP:Write   KEY0:Read");
	FRONT_COLOR=RED;
	
	while(EN25QXX_ReadID()!=EN25Q128)			//��ⲻ��EN25Q64
	{
		printf("EN25Q128 Check Failed!\r\n");
		LCD_ShowString(10,150,tftlcd_data.width,tftlcd_data.height,16,"EN25Q128 Check Failed!  ");
		delay_ms(1000);
	}
	printf("EN25Q128 Check Success!\r\n");
	LCD_ShowString(10,150,tftlcd_data.width,tftlcd_data.height,16,"EN25Q128 Check Success!");
		
	LCD_ShowString(10,170,tftlcd_data.width,tftlcd_data.height,16,"Write Data:");
	LCD_ShowString(10,190,tftlcd_data.width,tftlcd_data.height,16,"Read Data :");
	
	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY_UP_PRESS)
		{
			EN25QXX_Write((u8 *)text_buf,0,TEXT_LEN);
			printf("���͵����ݣ�%s\r\n",text_buf);
			LCD_ShowString(10+11*8,170,tftlcd_data.width,tftlcd_data.height,16,"www.prechin.net");
		}
		if(key==KEY0_PRESS)
		{
			EN25QXX_Read(buf,0,TEXT_LEN);
			printf("���յ����ݣ�%s\r\n",buf);
			LCD_ShowString(10+11*8,190,tftlcd_data.width,tftlcd_data.height,16,buf);
		}
		
		/*i++;
		if(i%20==0)
		{
			LED1=!LED1;
		}*/
		
		delay_ms(10);
			
	}
}
