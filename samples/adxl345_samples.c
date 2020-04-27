#include "adxl345.h"
#include <stdio.h>


//��ȡADXL��ƽ��ֵ
//x,y,z:��ȡ10�κ�ȡƽ��ֵ
void ADXL345_RD_Avval(rt_int16_t *x,rt_int16_t *y,rt_int16_t *z)
{
	rt_int16_t tx=0,ty=0,tz=0;	   
	rt_uint8_t i;  
	for(i=0;i<10;i++)
	{
		adxl345_get_xyz(x,y,z);
		tx+=(rt_int16_t)*x;
		ty+=(rt_int16_t)*y;
		tz+=(rt_int16_t)*z;	   
	}
	*x=tx/10;
	*y=ty/10;
	*z=tz/10;
} 

//�Զ�У׼
//xval,yval,zval:x,y,z���У׼ֵ
void ADXL345_AUTO_Adjust()
{
	rt_int16_t tx,ty,tz;
	rt_uint8_t i;
	rt_int16_t offx=0,offy=0,offz=0;
	adxl345_set_power_mode(0x0);      	                    //�Ƚ������ģʽ.
	rt_thread_mdelay(2);
	adxl345_set_register_value(ADXL345_DATA_FORMAT,0X0B);	//�͵�ƽ�ж����,13λȫ�ֱ���,��������Ҷ���,16g���� 
//	adxl345_set_register_value(ADXL345_BW_RATE,0x0A);		//��������ٶ�Ϊ100Hz
	adxl345_set_register_value(ADXL345_POWER_CTL,0x08);	   	//����ģʽ
	adxl345_set_register_value(ADXL345_INT_ENABLE,0x80);	//ʹ������׼���ж�	 
//	adxl345_set_offset(0x00,0x00,0x00);
	rt_thread_mdelay(2);
	for(i=0;i<10;i++)
	{
		ADXL345_RD_Avval(&tx,&ty,&tz);
		offx+=tx;
		offy+=ty;
		offz+=tz;
	}	 		
	offx/=10;
	offy/=10;
	offz/=10;
	offx=-offx/4;
	offy=-offy/4;
	offz=-(offz-256)/4;	 
    rt_kprintf("offx:%d offy:%d offz:%d\r\n",offx,offy,offz);	
    adxl345_set_offset(offx,offy,offz);
	adxl345_set_power_mode(0x0);  //�ٴ���ģʽ.
} 

void adxl345_sample()
{
 rt_int16_t	x         = 0;  // X-axis's output data.
 rt_int16_t	y         = 0;  // Y-axis's output data.
 rt_int16_t	z         = 0;  // Z-axis's output data.
 float gx 			  = 0;
 float gy			  = 0;
 float gz			  = 0;
	 
 char   intSource = 0;  // Value of the ADXL345_INT_SOURCE register.

   if(adxl345_init() != RT_EOK)
		return ;

//   ADXL345_AUTO_Adjust();
	
   adxl345_set_tap_detection(ADXL345_SINGLE_TAP |
                             ADXL345_DOUBLE_TAP, // Tap type.
                             ADXL345_TAP_Z_EN,   // Axis control.
                            0x10,                // Tap duration.
                            0x10,                // Tap latency.
                            0x40,                // Tap window. 
                            0x10,                // Tap threshold.
                            0x00);               // Interrupt Pin.
	
    adxl345_set_free_fall_detection(0x01,  // Free-fall detection enabled.
                                    0x05,  // Free-fall threshold.
                                    0x14,  // Time value for free-fall detection.
                                    0x00); // Interrupt Pin.
    
    /* Set the range and the resolution. */
    adxl345_set_range_resolution(ADXL345_RANGE_PM_16G, ADXL345_FULL_RES);
    adxl345_set_power_mode(0x1);          // Measure mode.


	 while(1)
    {
        /* Read and display the output data of each axis. */ 
        adxl345_get_xyz(&x, &y, &z);
		adxl345_get_g_xyz(&gx, &gy, &gz);
        printf("x:%d y:%d z:%d gx:%f gy:%f gz:%f\r\n",x,y,z,gx,gy,gz);
        intSource = adxl345_get_register_value(ADXL345_INT_SOURCE);
        if((intSource & ADXL345_SINGLE_TAP) != 0)
        {
			rt_kprintf("Single Tap\r\n");
        }
        if((intSource & ADXL345_DOUBLE_TAP) != 0)
        {
			rt_kprintf("Double Tap\r\n");
        }
        if((intSource & ADXL345_FREE_FALL) != 0)
        {
			rt_kprintf("Free-Fall\r\n");
        }
        rt_thread_mdelay(100);
    }
}
MSH_CMD_EXPORT(adxl345_sample,adxl345 sample);
