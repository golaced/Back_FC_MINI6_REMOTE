#ifndef __ADC_H
#define __ADC_H	
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEKս��STM32������
//ADC ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/7
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define  ADC_BAT		0
#define  ADC_YAW		2
#define  ADC_THRUST		1

//��ʼ��ADC��ʹ��DMA����
void Adc_Init(void);
void ADC_Filter(uint16_t* adc_val);	//ADC��ֵ�˲�
uint16_t getAdcValue(uint8_t axis);
extern int rc_off[2];
//�ɿ����ݽṹ
typedef struct 
{
	u16 roll;
	u16 pitch;
	u16 yaw;
	u16 thrust;
	u16 bat,bat_percent;
}joystickFlyui16_t;
extern joystickFlyui16_t adc_rc;
void getFlyDataADCValue(void);
#endif 