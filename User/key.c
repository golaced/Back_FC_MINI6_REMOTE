#include "stm32f10x.h"
#include "key.h"
#include "delay.h"
/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly_Remotor
 * ������������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/
		
//����IO��ʼ������
void keyInit(void) 
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin  = KEY_SEL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = KEY_1 | KEY_2 |KEY_3 | KEY_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ; //�������� 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

u8 key_o[5];
void KEY_Scan(float dt)
{	 
	static u32 cnt;
	u8 sel;
	
  key_o[4]=READ_KEY_SEL();
	key_o[0]=READ_KEY_1();
	key_o[1]=READ_KEY_2();
	key_o[2]=READ_KEY_3();
	key_o[3]=READ_KEY_4();
}












