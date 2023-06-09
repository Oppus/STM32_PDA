/*******************************************************************************
*                 
*                 		       普中科技
--------------------------------------------------------------------------------
* 实 验 名		 : SPI-FLASH实验
* 实验说明       : 
* 连接方式       : 
* 注    意		 : FLASH驱动在flash.c内
*******************************************************************************/

#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "tftlcd.h"
#include "key.h"
#include "spi.h"
#include "flash.h"


//要写入到25Q64的字符串数组
const u8 text_buf[]="www.prechin.net";
#define TEXT_LEN sizeof(text_buf)


int main()
{
	//u8 i=0;
	u8 key;
	u8 buf[30];
	
	SysTick_Init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
	LED_Init();
	USART1_Init(115200);
	TFTLCD_Init();			//LCD初始化
	KEY_Init();
	EN25QXX_Init();
	
	FRONT_COLOR=BLACK;
	LCD_ShowString(10,10,tftlcd_data.width,tftlcd_data.height,16,"PRECHIN STM32F4");
	LCD_ShowString(10,30,tftlcd_data.width,tftlcd_data.height,16,"www.prechin.net");
	LCD_ShowString(10,50,tftlcd_data.width,tftlcd_data.height,16,"FLASH-SPI Test");
	LCD_ShowString(10,70,tftlcd_data.width,tftlcd_data.height,16,"K_UP:Write   KEY0:Read");
	FRONT_COLOR=RED;
	
	while(EN25QXX_ReadID()!=EN25Q128)			//检测不到EN25Q64
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
			printf("发送的数据：%s\r\n",text_buf);
			LCD_ShowString(10+11*8,170,tftlcd_data.width,tftlcd_data.height,16,"www.prechin.net");
		}
		if(key==KEY0_PRESS)
		{
			EN25QXX_Read(buf,0,TEXT_LEN);
			printf("接收的数据：%s\r\n",buf);
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
