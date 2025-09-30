
/**
 * @file max1865atp_bsp.c
 *
 * @brief This file contains all basic communication and device setup for the ADS1118 device family.
 * @warning This software AD1118 Drivers
 *
 * @copyright http://www.ti.com/
 */
 #include "main.h"
 #include "max31865atp_bsp.h"
 #include "spi.h"
 #include "stdlib.h"
 #include "math.h"
 
 #define K_MIN_TEPMRATRUE -40.0
 #define K_MAX_TEPMRATRUE 150.0  

  /********************MAX31865*****************************************/	
#define DRDY HAL_GPIO_ReadPin(ADS1118_DRDY_in_GPIO_Port, ADS1118_DRDY_in_Pin)//iso module is not connect

static void setMax3185CS(unsigned char sel)
{	
	if(sel==GPIO_PIN_RESET)				HAL_GPIO_WritePin(GPIOE, ADS1118_CS_out_Pin, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOE, ADS1118_CS_out_Pin, GPIO_PIN_SET);
}
	
static void  waitForDRDYinterrupt(uint32_t dTim )
{    
// HAL_Delay(150);
//delay_us(800);
 //HAL_delay(dTim);
 //osDelay(dTim);
	uint32_t timeout;
	timeout=0;
	while(DRDY==GPIO_PIN_SET)
	{
		HAL_Delay(1);
		timeout++; 
		if(timeout>dTim) break;
	}
}

#ifdef MAX_31865_PT100	
float Get_pt_tempture(void)//PT100
{
  float temps;
  uint16_t dtemp[2];
  uint16_t data_temp;  
  waitForDRDYinterrupt(70);
  dtemp[0]=MAX31865_SB_Read(0x01);
  dtemp[1]=MAX31865_SB_Read(0x02);
  data_temp=(dtemp[0]<<7)+(dtemp[1]>>1);//Get 15Bit DATA;
  temps=data_temp;
  temps=(temps*402)/32768;//Here is the rtd R value;
  temps=(temps-100)/0.385055;//A gruad
  DEBUG_PRINTF("T=%.2f  \r\n",temps);
  return temps;
}
#else 
float Get_pt_tempture(void)//PT1000
{
  float temps;
  uint16_t dtemp[2];
  uint16_t data_temp;  
  //waitForDRDYinterrupt(50);
  dtemp[0]=MAX31865_SB_Read(0x01);
  dtemp[1]=MAX31865_SB_Read(0x02);
  data_temp=(dtemp[0]<<7)|(dtemp[1]>>1);//Get 15Bit DATA;
  temps=data_temp;	
  temps=(temps*0.1220703125);//4000)/32768;//Here is the rtd R value;
  temps=(temps-1000)/3.85055;//A gruad
  //DEBUG_PRINTF("T=%.1f %d \r\n",temps,data_temp);
  return temps;
}
#endif
unsigned char MAX31865_SB_Read(unsigned char addr)//SPI Single-Byte Read
{
	unsigned char TXdata[2], RXdata[2];
	HAL_StatusTypeDef err;
	TXdata[0]=addr;
	TXdata[1]=0;
	setMax3185CS(0);
	delay_us(10);
	err=HAL_SPI_TransmitReceive(&hspi4, TXdata, RXdata, 2,100);// HAL_MAX_DELAY);
	if(err!=HAL_OK) 
	{  
		memset(RXdata,0xFF,2);		
	}
	setMax3185CS(1);   
	return RXdata[1];
}
void MAX31865_SB_Write(unsigned char addr,unsigned char wdata)//SPI Single-Byte Write
{
    unsigned char TXdata[2], RXdata[2];
    HAL_StatusTypeDef err;
    TXdata[0]=addr;
    TXdata[1]=wdata;
    setMax3185CS(0);
    delay_us(10);
    err=HAL_SPI_TransmitReceive(&hspi4, TXdata, RXdata, 2,100);// HAL_MAX_DELAY);
    if(err!=HAL_OK) 
    {  
      memset(RXdata,0xFF,2);		
    }
    setMax3185CS(1);   
}

void max_31865_pt1000(void)
{	//50HZ
  setMax3185CS(1);	
  delay_us(1);	
	MAX31865_SB_Write(0x80,0xC1);//二线、四线配置
	// MAX31865_SB_Write(0x80,0xD1);//三线配置
}
	

	
	

	