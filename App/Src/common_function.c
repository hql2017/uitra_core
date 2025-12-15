/*
 * common_function.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */

#include "common_function.h" 

/*************************************** *//*
CRC16——modbus
uint16_t crc_table_list[256]=
{0x40BF,0x807E,

}
/ ****
  * @brief MAAX->MIN，冒泡排序
  * @param dataBuff
  * @note   
  * @retval None
  */
 void sord_data(unsigned  short int *dataBuff,unsigned  short int length,unsigned char flag )
 {
	unsigned  short int tempValue=0,i,j;	
	for(j=0;j+1<length;j++)
	{
		for(i=0;i+j+1<length;i++)
		{
			if(flag==0)//min->MAX
			{
				if(dataBuff[i]>dataBuff[i+1])
				{
					tempValue = dataBuff[i];				
					dataBuff[i]= dataBuff[i+1];
					dataBuff[i+1]=tempValue;
				}				
			}
			else
			{
				if(dataBuff[i]<dataBuff[i+1])
				{
					tempValue = dataBuff[i];				
					dataBuff[i]= dataBuff[i+1];
					dataBuff[i+1]=tempValue;
				}				
			}
		}
	}
		
} 

// 初始化滤波器
void kalman_filter_init(KalmanFilter* kf, double initial_estimate, double variance) {
    kf->last_estimate = initial_estimate;
    kf->estimate = initial_estimate;
    kf->variance = variance;
    kf->kalman_gain = 0.0;
    kf->error_estimate = 1.0;
} 

// 更新滤波器状态
double kalman_filter_update(KalmanFilter* kf, double measurement) {
    // 计算卡尔曼增益
    kf->kalman_gain = kf->error_estimate / (kf->error_estimate + kf->variance);
    // 更新误差估计值
    kf->error_estimate = (1 - kf->kalman_gain) * kf->error_estimate + kf->kalman_gain * (measurement - kf->last_estimate);
    // 更新估计值
    kf->estimate = kf->last_estimate + kf->kalman_gain * (measurement - kf->last_estimate);
    // 更新上次估计值
    kf->last_estimate = kf->estimate;
    return kf->estimate;
}





 void DWT_Init(void)
{
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
}       

 void delay_us(volatile uint32_t nus)
{   
    volatile uint32_t start=DWT->CYCCNT,tcnt = 0;        //刚进入时的计数器值   
    volatile uint32_t ticks=nus*(SystemCoreClock/1000000);//nus需要的节拍数 
    while(tcnt<ticks)
    {
        tcnt=  DWT->CYCCNT- start;
    }                                   
}
void delay_ms(volatile uint32_t nms)
{   
    #if 1
    HAL_Delay(nms);
    #else
    while(nms--)
    {
        delay_us(1000);
    }    
    #endif                                
}
//返回HAL_OK表示无变化
HAL_StatusTypeDef compare_buff_no_change(unsigned char *buff1,unsigned char *buff2,unsigned int length)
{   
	HAL_StatusTypeDef ret=HAL_OK;
	for(unsigned int i=0;i<length;i++)
	{
		if(buff1[i]!=buff2[i])
		{
			ret=HAL_ERROR;
			break;
		}
	}	
  return ret;                        
}
// CRC32校验函数,LSB
unsigned  int crc32_MPEG(unsigned char *pData,unsigned  int length)
{
	uint32_t crc = 0xFFFFFFFF;// 初始值
	uint32_t CRC32_POLY=0xEDB88320; // 多项式0x04C11DB7的反码
    for (int i = 0; i < length; i++) 
	{
        crc ^= pData[i]; // 将当前字节与CRC寄存器进行XOR
        for (int j = 0; j < 8; j++) 
		{
            if (crc & 1) 
			{ // 如果最低位是1
                crc = (crc >> 1) ^ CRC32_POLY; // 右移一位后与多项式进行XOR
            } 
			else 			
			{
                crc >>= 1; // 否则只右移一位
            }
        }
    }	
	return crc;
}
// 和校验函数
 unsigned int sumCheck(unsigned char *pData,unsigned  int length)
{
 unsigned  int sum;
	for (int i = 0; i < length; i++)
	{
		sum+=pData[i];
	}
	return sum;
}
// CRC校验函数,LSB
 unsigned short int crc16Num(unsigned char *pData,unsigned  int length)
{
	uint16_t crc = 0xFFFF;	
	for (int i = 0; i < length; i++)
	{
		crc ^= pData[i];			
		for (int j = 0; j < 8; j++)
		{
			if (crc & 1)
			{
				crc >>= 1;
				crc ^= 0xA001;			
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc;
}
#if 0
uint16_t crc16_modbus(const  uint8_t *data, uint16_t len)
{  
	uint16_t crc = 0xFFFF;  // 初始值
	for (size_t pos = 0; pos < len; pos++) 
	{
		crc ^= (uint16_t)data[pos];  // XOR输入字节到CRC寄存器
		for (uint8_t i = 8; i != 0; i--) 
		{  // 处理每个字节的每一位
			if ((crc & 0x0001) != 0) 
			{  //如果最低位是1
				crc >>= 1;
				crc ^= 0xA001;
			} 
			else 
			{
				crc >>= 1;
			}
		}
	}
	//最终结果需要再次进行反转，因为Modbus协议要求最终的CRC是高字节在前，低字节在后
	return  (crc << 8) | (crc >> 8);
	
}
#endif
  