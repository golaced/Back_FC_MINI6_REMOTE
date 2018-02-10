#include "head.h"
#include "table.h"
#include "spi.h"
#include "nrf.h"
#include "adc.h"
#include "key.h"
#include "mpu9250.h"
#include "FLASH.h"
#include "imu.h"
#include "mymath.h"
#include "rc_mine.h"
#include "beep.h"
#include "gui_basic.h"
#include "hw_config.h"
#include "data_transfer.h"

void OLED_TASK(float dt)
{
	 int cx,cy;
	 char temp[10][20]={'\0'};
   GUI_RectangleFill(42,1,123,55,0);
   GUI_Rectangle(42,1,123,55,1);
   cx=LIMIT(80+(Rc_Get.ROL-1500)*0.1,42+3,123-3);
   cy=LIMIT(28+(Rc_Get.PIT-1500)*0.1,1+3,55-3);
	 GUI_CircleFill(cx,cy,3,1);  
   GUI_Circle(80,28,5,1);   
   //
   OLED_Refresh_Gram(); 
   //
   OLED_P6x8Str(3,0,"P:");
   my_itoa(Rc_Get.PIT,temp[0]);
	 OLED_P6x8Str(4*3,0,temp[0]);

   OLED_P6x8Str(3,1,"R:");
   my_itoa(Rc_Get.ROL,temp[1]);
	 OLED_P6x8Str(4*3,1,temp[1]);

   OLED_P6x8Str(3,2,"T:");
   my_itoa(Rc_Get.THROTTLE,temp[2]);
	 OLED_P6x8Str(4*3,2,temp[2]);

   OLED_P6x8Str(3,3,"Y:");
   my_itoa(Rc_Get.YA,temp[3]);
	 OLED_P6x8Str(4*3,3,temp[3]);
   //GPS
   OLED_P6x8Str(45,0,"GPS:");
   my_itoa(plane.gps_sv,temp[0]);
	 OLED_P6x8Str(45+5*5,0,temp[0]);
  //lock
   if(!plane.lock)
   OLED_P6x8Str(90,0,"*lock*");
   else
	 OLED_P6x8Str(90,0,"*fly!*");
   //SSR
   OLED_P6x8Str(3,4,"SR:");
   my_itoa(plane.rssi,temp[3]);
	 OLED_P6x8Str(4*5,4,temp[3]);
   OLED_P6x8Str(4*(5+3),4,"%");
   //BAT_RC
   OLED_P6x8Str(3,5,"BR:");
   my_itoa(adc_rc.bat_percent,temp[4]);
	 OLED_P6x8Str(4*5,5,temp[4]);
   OLED_P6x8Str(4*(5+3),5,"%");
   //BAT_FLY
   OLED_P6x8Str(3,6,"BF:");
   my_itoa(adc_rc.bat_percent,temp[5]);
	 OLED_P6x8Str(4*5,6,temp[5]);
	 OLED_P6x8Str(4*(5+3),6,"%");
	 //MODE
   OLED_P6x8Str(3,7,"M:");
   if(height_mode==0)
		 OLED_P6x8Str(4*3,7,"Munl ");
	 else if(height_mode==1&&pos_mode==0)
		 OLED_P6x8Str(4*3,7,"Alt B");
	 else if(height_mode==2&&pos_mode==0)
		 OLED_P6x8Str(4*3,7,"Alt S");
	 else if(pos_mode==1)
		 OLED_P6x8Str(4*3,7,"Pos  ");
	 else if(pos_mode==2)
		 OLED_P6x8Str(4*3,7,"Smart");
	 
	
}

void BEEP_TASK(float dt)
{
 	Play_Music_Task(RC_ERO_BEEP,dt); 
}

float dt[10]={0},off_att[2];
u16 key_sel_down=0;
int main(void)
{	u8 i;

	delay_init(72);		//延时初始化
	TIM3_Config();
	Cycle_Time_Init();
	OLED_Init();
	USART_init();	
	Adc_Init();
	SPI1_Init();		
	Nrf24l01_Init(MODEL_TX2,40);// 伪双工  主接收
	Nrf24l01_Check();
	Mpu9250_Init();
	Parameter_Init();
	Time4ON();
  keyInit(); 
	OLED_Fill(0x00);OLED_P6x8Str(3,0,"By Golaced-BIT ");OLED_P8x16Str(43,3,"Welcome!");
	delay_ms(20000);
	OLED_Fill(0x00);
	Beep_Init(0,72-1);
	Play_Music_Direct(START_BEEP);
	usb_vcp_init();
	__enable_irq();
	delay_ms(5000);
	
	while(1){

		
		 if(flag_ms[2]==1)
		 {
      flag_ms[2]=0;
			dt[0] = Get_Cycle_T(0)/1000000.0f;	
      if(dt[0]>0.005)
				dt[0]=0.005;
			else if(dt[0]<0.0015)
				dt[0]=0.0015;
			getFlyDataADCValue();
			MPU9250_ReadValue();
			MPU6050_Data_Prepare(dt[0]);
		 } 
		
     if(flag_ms[5]==1)
		 {
      flag_ms[5]=0;
			dt[1] = Get_Cycle_T(1)/1000000.0f;	  
			if(dt[1]>0.01)
				dt[1]=0.01;
			else if(dt[1]<0.0025)
				dt[1]=0.0025;
			//IMUupdate(0.5f *dt[1],mpu6050_fc.Gyro_deg.x, mpu6050_fc.Gyro_deg.y, mpu6050_fc.Gyro_deg.z, mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,&Rol_fc,&Pit_fc,&Yaw_fc); 
			madgwick_update_new( mpu6050_fc.Acc.x, mpu6050_fc.Acc.y, mpu6050_fc.Acc.z,mpu6050_fc.Gyro_deg.x/57.3, mpu6050_fc.Gyro_deg.y/57.3, mpu6050_fc.Gyro_deg.z/57.3,20,20,20, &Rol_fc,&Pit_fc,&Yaw_fc,dt[1]); 
			adc_rc.pitch=LIMIT(-my_deathzoom1(Pit_fc-off_att[0],1.5),-45,45)/45.*500+1500;
			adc_rc.roll=LIMIT(my_deathzoom1(Rol_fc-off_att[1],1.5),-45,45)/45.*500+1500; 
			Rc_Get.PIT=LIMIT(adc_rc.pitch,1000,2000);
			Rc_Get.ROL=LIMIT(adc_rc.roll,1000,2000);
			Rc_Get.YA=LIMIT(adc_rc.yaw,1000,2000);
			Rc_Get.THROTTLE=LIMIT(adc_rc.thrust,1000,2000);
			
			if(key_o[4]){
           key_sel_down++;
        }
				if(key_sel_down>2/0.005){
					  key_sel_down=0;
					  off_att[0]=Pit_fc;
					  off_att[1]=Rol_fc;
					  Play_Music_Direct(RC_RESET_BEEP);
				}

		 } 
  
		 if(flag_ms[10]==1)
		 {
			dt[2] = Get_Cycle_T(2)/1000000.0f;	   
      flag_ms[10]=0;
		 }
		 
		  if(flag_ms[20]==1)
		 {
			dt[3] = Get_Cycle_T(3)/1000000.0f;	   
      flag_ms[20]=0;
      Nrf_Check_Event();
			RC_Send_Task();	
			ANO_DT_Data_Exchange(); 
		 }
		 
		 if(flag_ms[25]==1)
		 {
			dt[4] = Get_Cycle_T(4)/1000000.0f;	   
      flag_ms[25]=0;
			
		 }
		 
		 if(flag_ms[50]==1)
		 {
			dt[5] = Get_Cycle_T(5)/1000000.0f;	   
      flag_ms[50]=0;
			OLED_TASK(0.05);  
			BEEP_TASK(0.05); 
			KEY_Scan(0.05);	
		 }
		 
		 
	}
}
/*********************************************END OF FILE**********************/

