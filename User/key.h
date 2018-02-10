#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
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

#define KEY_1  		GPIO_Pin_15
#define KEY_2  		GPIO_Pin_14
#define KEY_3   	GPIO_Pin_13
#define KEY_4   	GPIO_Pin_12
#define KEY_SEL   GPIO_Pin_8

#define READ_KEY_1()  	GPIO_ReadInputDataBit(GPIOB,KEY_1)	//��ȡ�󰴼�
#define READ_KEY_2()   	GPIO_ReadInputDataBit(GPIOB,KEY_2)	//��ȡ�Ұ���
#define READ_KEY_3()  	GPIO_ReadInputDataBit(GPIOB,KEY_3)	//��ȡҡ��1����
#define READ_KEY_4()  	GPIO_ReadInputDataBit(GPIOB,KEY_4)	//��ȡҡ��1����
#define READ_KEY_SEL()  !GPIO_ReadInputDataBit(GPIOA,KEY_SEL)	//��ȡҡ��2����

//IO��ʼ��
void keyInit(void);

 //����ɨ�躯��		
void KEY_Scan(float dt);

extern u8 key_o[5];

#endif



