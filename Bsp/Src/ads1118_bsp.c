/**
 * @file ads1118.c
 *
 * @brief This file contains all basic communication and device setup for the ADS1118 device family.
 * @warning This software AD1118 Drivers
 *
 * @copyright http://www.ti.com/
 *
 */

#include "ads1118_bsp.h"
#include "spi.h"
#include "stdlib.h"
#include "math.h"
#define LOW   0
#define HIGH  1

 const char *adcRegisterNames[NUM_REGISTERS] = {"DATA", "CONFIG"};

//static const float k_temprature_voltage_list[20]={-1.527,-1.156,-0.777,0.392,0,0.397,0.798,1.203,1.611,2.022,\
2.436,2.850,3.266,3.681,4.095,4.508,4.919,5.327,5.733,6.137};

/****热电偶 -40~150 测温分度表 */
static const float k_temprature_voltage_list[191]={
    -1.527,//-40
    -1.490,-1.453,-1.416,-1.379,-1.342,-1.305,-1.268,-1.231,-1.193,-1.156,//-39~-30
    -1.118,-1.081,-1.043,-1.005,-0.968,-0.930,-0.892,-0.854,-0.816, -0.777,
    -0.739,-0.701,-0.662,-0.624,-0.585,-0.547,-0.508,-0.469,-0.431,-0.392,
    -0.353,-0.314,-0.275,-0.236,-0.197,-0.157,-0.118,-0.079,-0.039,0,//0
    0.039,0.079,0.119,0.158,0.198,0.238,0.277,0.317,0.357, 0.397,
    0.437,0.477,0.517,0.557,0.597,0.637,0.677,0.718,0.758,0.798,
    0.838,0.879,0.919,0.960,1.000,1.041,1.081,1.122,1.162,1.203,
    1.244,1.285,1.325,1.366,1.407,1.448,1.489,1.529,1.570,1.611,
    1.652,1.693,1.734,1.776,1.817,1.858,1.899,1.940,1.981,2.022,    
    2.064,2.105,2.146,2.188,2.229,2.270,2.312,2.353,2.394,2.436,
    2.477,2.519,2.560,2.601,2.643,2.684,2.726,2.767,2.809,2.850,
    2.892,2.933,2.975,3.016,3.058,3.100,3.141,3.183,3.224,3.266,
    3.307,3.349,3.390,3.432,3.473,3.515,3.556,3.598,3.639,3.681,
    3.722,3.764,3.805,3.847,3.888,3.930,3.971,4.012,4.054,4.095,
    4.137,4.178,4.219,4.261,4.302,4.343,4.384,4.426,4.467,4.508,
    4.549,4.590,4.632,4.673,4.714,4.755,4.796,4.837,4.878,4.919,
    4.960,5.001,5.042,5.083,5.124,5.164,5.205,5.246,5.287,5.327,
    5.368,5.409,5.450,5.490,5.531,5.571,5.612,5.652,5.693,5.733,
    5.774,5.814,5.855,5.895,5.936,5.976,6.016,6.057,6.097,6.137//150
};

#define K_MIN_TEPMRATRUE -40.0
#define K_MAX_TEPMRATRUE 150.0



static float tmprature_T0_cold_junction=25.0f,history_tmprature_T0=25.0f; //冷端补偿
static float tmprature_T1_cool_water,history_tmprature_T1;//冷却循环水
static float tmprature_T2_laser,history_tmprature_T2; //激光器温度

 //****************************************************************************
 //
 // Internal variables
 //
 //****************************************************************************
 
 // Array used to recall device register map configurations*/
 static uint16_t registerMap[NUM_REGISTERS];
 
  /**
  * @brief tmprature_cal
  * @param   int16_t  adValue
  * @note   pga:CONFIG_PGA_0p256V; unit mV
  * @retval None
  */
static float volta_tmprature_ret(float temprature)
{
    float vol;
    unsigned short int i;
    return vol;
}
 /**
  * @brief tmprature_cal
  * @param   int16_t  adValue
  * @note   pga:CONFIG_PGA_0p256V; unit mV
  * @retval None
  */
static float tmprature_cal(short int adValue)
{
    float T,temp,cold;
    unsigned short int i;
    unsigned short int temp2;
    //voltage=(adValue*256/32768);//  //voltage=(adValue*pga/32768) ;PGA=256mV  
    //voltage=(adValue/128;    
    //voltage=adValue*0.0078125;  
    /***********�ֶȱ�У׼*********/
    if(tmprature_T0_cold_junction<-40) temp2=0;
    else if(tmprature_T0_cold_junction>150) temp2=190;
    else  {
        temp2=(unsigned short int)(tmprature_T0_cold_junction+40);
    } 
    /****************************/  
    temp=(adValue*0.0078125)+k_temprature_voltage_list[temp2];//mV    
    if(temp<k_temprature_voltage_list[0])
    {
        T=-40.0;
        return T;
    }
    if(temp>=k_temprature_voltage_list[190])
    {
        T=150.0;
        return T;
    }
    for(i=0;i<190;i++)
    {
        if((temp<k_temprature_voltage_list[i+1])&&(temp>=k_temprature_voltage_list[i])) 
        {
            //if(temp>k_temprature_voltage_list[i]+0.016)  T=((short int)i)*1.0-39.5;//half
            //else T=((short int)i)*1.0-40.0;//
            T=((short int)i)*1.0-40+(temp-k_temprature_voltage_list[i])*0.1 ;//half            
            break;
        }          
    }      
    return T;
}
 static void setCS(unsigned char sel)
 {
    if(sel==LOW)   HAL_GPIO_WritePin(ADS1118_CS_out_GPIO_Port, ADS1118_CS_out_Pin, GPIO_PIN_RESET);
    else   HAL_GPIO_WritePin(ADS1118_CS_out_GPIO_Port, ADS1118_CS_out_Pin, GPIO_PIN_SET);
 }  
 static void spiSendReceiveArrays(unsigned char *TXdata, unsigned char *RXdata, unsigned short int dLength)
 {
    HAL_StatusTypeDef err;
    setCS(LOW);
    delay_us(50);
    err=HAL_SPI_TransmitReceive(&hspi4, TXdata, RXdata, dLength, HAL_MAX_DELAY);
    if(err!=HAL_OK) 
    {
        memset(RXdata,0xFF,dLength);
    }
    setCS(HIGH);   
 }

 static void  waitForDRDYinterrupt(uint32_t dTim )
 {    
   // HAL_Delay(150);

    delay_us(800);
    //HAL_delay(dTim);
    //osDelay(dTim);
 }
 //****************************************************************************
 //
 // Function Definitions
 //
 //****************************************************************************
 /**
  *
  * @brief restoreRegisterDefaults()
  * Updates the registerMap[] array to its default values.��registerMap[]�������ΪĬ��ֵ��
  *
  * NOTES:
  * - If the MCU keeps a copy of the ADS1118 register settings in memory,
  * then it is important to ensure that these values remain in sync with the
  * actual hardware settings. In order to help facilitate this, this function
  * should be called after powering up or resetting the device.
  *
  * - Reading back all of the registers after resetting the device can
  * accomplish the same result.�ڸ�λ�豸�󣬿��Զ������мĴ��������ͬ�Ľ����
  *
  * @return none
  */
 void restoreRegisterDefaults(void)
 {
     registerMap[CONVERSION_ADDRESS] = CONVERSION_DEFAULT;
     registerMap[CONFIG_ADDRESS]     = CONFIG_DEFAULT;
 }
 
 /**
  *
  * @brief adcStartup()
  * Example start up sequence for the ADS1118.ADS1118����˳��ʾ����
  *
  * Before calling this function, the device must be powered and
  * the SPI/GPIO pins of the MCU must have already been configured.
  *
  * @return none
  */
 void adcStartup(void)
 {   
     //
     // (OPTIONAL) Provide additional delay time for power supply settling
     //
     HAL_Delay(1);
 
     //
     // (REQUIRED) Initialize internal 'registerMap' array with device default settings
     //
     restoreRegisterDefaults();
     
     //
     // (OPTIONAL) Read back all registers and Check STATUS register (if exists) for faults
     //     
     registerMap[CONFIG_ADDRESS]  =    readSingleRegister(CONFIG_ADDRESS); 
     
    // registerMap[CONFIG_ADDRESS] &=0XF1FF;  //先复位分辨率
    //registerMap[CONFIG_ADDRESS] |=CONFIG_PGA_0p256V|;  //改变分辨率方便计算
    //registerMap[CONFIG_ADDRESS] |=CONFIG_DR_860SPS;  //最高采样率 complete_delay=(1000/860)=1.18ms;默认complete_delay=(1000/128)=7.82125ms
    // DEBUG_PRINTF("default_cfg=0x%04x=%04x\r\n",CONFIG_DEFAULT,registerMap[CONFIG_ADDRESS] );
 }
 
 /**
  *
  * @brief readSingleRegister()
  * Reads the contents of a single register at the specified address.
  *
  * @param[in] address��Ҫ��ȡ�ļĴ�����8λ��ַ��address Is the 8-bit address of the register to read.
  *
  * @return 16λ�Ĵ�����ȡ���Ϊ�޷���ֵ����ת��Ϊ������2�Ĳ��롣The 16-bit register read result as an unsigned value, but conversion
  * is binary 2's complement.
  */
 uint16_t readSingleRegister(uint8_t address)
 {
     //
     // Check that the register address is in range
     //
     assert(address <= MAX_REGISTER_ADDRESS);
     uint8_t dLength;
     uint8_t regTXdata[4] = {0};
     uint8_t regRXdata[4] = {0};
     uint16_t regValue = 0;

     regTXdata[0] = registerMap[CONFIG_ADDRESS]>>8;
     regTXdata[1] = registerMap[CONFIG_ADDRESS]&0xFF; 
     dLength=2;     
     if(address == 1) dLength = 4;//config   
     spiSendReceiveArrays(regTXdata, regRXdata, dLength); 
     //DEBUG_PRINTF("scfg=0x%02x=%02x=%02x=%02x\r\n",regRXdata[0] ,regRXdata[1] ,regRXdata[2] ,regRXdata[3]  );
     if(address == 1) //config
     {
        regValue =  combineBytes(regRXdata[2], regRXdata[3]);   
     }  
     else  regValue =  combineBytes(regRXdata[0], regRXdata[1]);      
     return regValue;
 }
 
 /**
  *
  * @brief writeSingleRegister()
  * Writes data to a single register and also read it back for internal
  * confirmation.
  *
  * @param[in] address Is the address of the register to write to.
  * @param[in] data Is the value to write.
  *
  * @return uint16_t of config register
  */
 uint16_t writeSingleRegister(uint8_t address, uint16_t data)
 {
     //
     // ���Ĵ�����ַ�Ƿ��ڷ�Χ��Check that the register address is in range
     //
     assert(address <= MAX_REGISTER_ADDRESS);
 
     uint8_t regData[4], RxData[4];   
     regData[0] =data>>8;
     regData[1] = data&0xFF; 
     regData[2] =0x00;
     regData[3] = 0x00; 
     // д��Ӧ���������У�Ȼ���ٻض��Խ�����֤Write should take place immediately and then read back for verification
     spiSendReceiveArrays(regData, RxData, 4);  
     // ����յ��������Ƿ��뷢�͵�������ͬCheck if what was received was the same as what was sent
     //registerMap[CONVERSION_ADDRESS] = combineBytes(RxData[0], RxData[1]);
     if(data == combineBytes(RxData[2], RxData[3]))//config
     {
        registerMap[CONFIG_ADDRESS] = data;
        return data;
     }  
     return 0;
 }
 
 /**
  * @brief readData()
  * Reads ADC conversion data.��ȡADCת�����ݡ�
  *
  * @return  int16_t Conversion register contents
  *
  */
 int16_t readData(void)
 {
     uint16_t regValue = 0;  
     static uint16_t flag;
     
     // Read conversion results
    //registerMap[CONVERSION_ADDRESS] = readSingleRegister(CONVERSION_ADDRESS);//读取上一次转换值
    // Get the contents of the config register, we want to keep the configuration just start the conversion��ȡ���üĴ��������ݺ�����Ҫ���ָտ�ʼ������ת��
    if(flag==0)
    {     
        regValue=CONFIG_SS_CONV_START |CONFIG_PGA_0p256V |CONFIG_DR_860SPS| registerMap[CONFIG_ADDRESS]|TS_MODE_TS;//冷端温度           
    }
    else  if(flag==1)
    {   
        regValue=CONFIG_SS_CONV_START | CONFIG_PGA_0p256V|CONFIG_DR_860SPS|CONFIG_MUX_AIN0_AIN1|(registerMap[CONFIG_ADDRESS])&0xFFEF;//CONFIG_MUX_AIN0_AIN1         
    }
    else
    {    
        regValue=CONFIG_SS_CONV_START | CONFIG_PGA_0p256V|CONFIG_DR_860SPS|registerMap[CONFIG_ADDRESS]|TS_MODE_ADC|CONFIG_MUX_AIN2_AIN3;//ADC2, 3      
    }     
    //regValue =CONFIG_SS_CONV_START | registerMap[CONFIG_ADDRESS]|TS_MODE_TS;//
    // Write to the config register and start a conversion
    writeSingleRegister(CONFIG_ADDRESS, regValue); 
    HAL_Delay(2);
    // Monitor DRDY
    // and hold CS low
     //setCS(LOW);   
    // HAL_Delay(1); 
    // waitForDRDYinterrupt(1000);
    //setCS(HIGH);
    // Read conversion results
    registerMap[CONVERSION_ADDRESS] = readSingleRegister(CONVERSION_ADDRESS);   
    if(flag==0)
    { 
        tmprature_T0_cold_junction=((int16_t) registerMap[CONVERSION_ADDRESS]>>2)*0.03125;//冷端补偿
        DEBUG_PRINTF("T0=%.1fad=%d\r\n", tmprature_T0_cold_junction,(int16_t) registerMap[CONVERSION_ADDRESS]);        
    }
    else  if(flag==1)
    {  
        tmprature_T1_cool_water=tmprature_cal((int16_t) registerMap[CONVERSION_ADDRESS]);//循环水温度
        DEBUG_PRINTF("T1=%.1fad=%d\r\n", tmprature_T1_cool_water,(int16_t) registerMap[CONVERSION_ADDRESS]);
    }
    else
    {   
        tmprature_T2_laser=tmprature_cal((int16_t) registerMap[CONVERSION_ADDRESS]);//激光器温度
        DEBUG_PRINTF("T2=%.1fad=%d\r\n",tmprature_T2_laser,(int16_t) registerMap[CONVERSION_ADDRESS]);
    } 
    flag++;
    flag%=3;
    return (int16_t) registerMap[CONVERSION_ADDRESS];
 } 
 /**
  * @brief startAdcConversion()
  * ����ADCת����Starts the ADC conversion.
  *
  * @return  uint16_t configuration register
  *
  */
 uint16_t startAdcConversion()
 {
     // Get the contents of the config register, we want to keep the configuration just start the conversion
     uint16_t regValue = registerMap[CONFIG_ADDRESS];
 
     regValue = CONFIG_SS_MASK | regValue;
     writeSingleRegister(CONFIG_ADDRESS, regValue);
 
     return regValue;
 }
 
 /**
  * @brief stopAdcConversion()
  *
  * @brief   Stops the ADC conversion if in continuous mode.�����������ģʽ����ֹͣADCת����
  *
  * @return  uint16_t configuration register
  *
  */
 uint16_t stopAdcConversion()
 {
     // ��ȡ���üĴ���������Get the contents of the config register
     uint16_t regValue = registerMap[CONFIG_ADDRESS]; 
     regValue =  regValue | CONFIG_MODE_SS;
     writeSingleRegister(CONFIG_ADDRESS, regValue);
     registerMap[CONFIG_ADDRESS] = regValue;
     return regValue;
 }
 
 /**
  *
  * @brief getRegisterValue()
  * Getter function to access registerMap array from outside of this module.Getter�����Ӹ�ģ���ⲿ����registerMap���顣
  *
  * @param[in] address Is the address of the register to read.
  *
  * NOTE: The internal registerMap arrays stores the last known register value,
  * since the last read or write operation to that register. This function
  * does not communicate with the device to retrieve the current register value.
  * For the most up-to-date register data or retrieving the value of a hardware
  * controlled register, it is recommend to use readSingleRegister() to read the
  * current register value.
  *
  * @return uint16_t Register value.
  */
 uint16_t getRegisterValue(uint8_t address)
 {
    assert(address <= MAX_REGISTER_ADDRESS);
    return registerMap[address];
 }
 
 //****************************************************************************
 //
 // Helper functions
 //
 //****************************************************************************
 
 /**
  *
  * @brief upperByte()
  * Takes a 16-bit word and returns the most-significant byte.����һ��16λ�ֲ��������λ�ֽڡ�
  *
  * @param[in] uint16_Word Is the original 16-bit word.
  *
  * @return 8-bit most-significant byte.8λ�����Ч�ֽڡ�
  */
 uint8_t upperByte(uint16_t uint16_Word)
 {
    uint8_t msByte;
    msByte = (uint8_t) ((uint16_Word >> 8) & 0x00FF); 
    return msByte;
 }
 
 /**
  *
  * @brief lowerByte()
  * Takes a 16-bit word and returns the least-significant byte.����һ��16λ�ֲ��������λ�ֽڡ�
  *
  * @param[in] uint16_Word Is the original 16-bit word.
  *
  * @return 8-bit least-significant byte.8λ�����Ч�ֽڡ�
  */
 uint8_t lowerByte(uint16_t uint16_Word)
 {
    uint8_t lsByte;
    lsByte = (uint8_t) (uint16_Word & 0x00FF); 
    return lsByte;
 }
 
 /**
  *
  * @brief combineBytes()
  * Takes two 8-bit words and returns a concatenated 16-bit word.��������8λ�ֲ�����һ�����ӵ�16λ�֡�
  *
  * @param[in] upperByte Is the 8-bit value that will become the MSB of the 16-bit word.upperByte�ǽ���Ϊ16λ�ֵ�MSB��8λֵ��
  * @param[in] lowerByte Is the 8-bit value that will become the LSB of the 16-bit word.lowerByte�ǽ���Ϊ16λ�ֵ�LSB��8λֵ��
  *
  * @return concatenated unsigned 16-bit word.
  */
 uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte)
 {
     uint16_t combinedValue;
     combinedValue = ((uint16_t) upperByte << 8) | ((uint16_t) lowerByte);
 
     return combinedValue;
 }
 
 /**
  *
  * @brief combineDataBytes()
  * Combines ADC data bytes into a single signed 32-bit word.��ADC�����ֽںϲ�Ϊ���������ŵ�32λ�֡�
  *
  * @param[in] dataBytes Is a pointer to uint8_t[] where the first element is the MSB.dataBytes��ָ��uint8_t[]��ָ�룬���е�һ��Ԫ����MSB��
  *
  * @return Returns the signed-extend 32-bit result.���ش�������չ��32λ�����
  */
 int32_t signExtend(const uint8_t dataBytes[])
 {
     int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
     int32_t lowerByte   = ((int32_t) dataBytes[1] << 16);
 
     return (((int32_t) (upperByte | lowerByte)) >> 16);                 // Right-shift of signed data maintains signed bit�з������ݵ����Ʊ����з���λ
 }
 
 /**
  *
  * @brief ads1118_measure_internal_temperature_example()
  * Captures the internal temperature sensor data and returns temperature in deg C�����ڲ��¶ȴ��������ݲ��ԡ�Ϊ��λ�����¶�
  *
  * @return Returns floating point value of temperature in deg C�����¶ȵĸ���ֵ����λΪ��
  */
 float ads1118_measure_internal_temperature_example(void)
 {
     float temp_C;
     int16_t cValue;
     /* Store current configuration value to restore later�洢��ǰ����ֵ�Ա��Ժ�ָ� */
     uint16_t regValue = getRegisterValue(CONFIG_ADDRESS);
 
     /* Change configuration to temperature sensor mode�����ø���Ϊ�¶ȴ�����ģʽ */
     registerMap[CONFIG_ADDRESS] = regValue | TS_MODE_TS;
     /* If operating in continuous conversion mode read the data a second time 
      *  to make sure that the previous data was not with an incorrect configuration ���������ת��ģʽ�¹�������ڶ��ζ�ȡ������ȷ��֮ǰ������û�����ô���
      */
     if(regValue & CONFIG_MODE_SS) 
         /* Start conversion and read result��ʼת������ȡ��� */
         cValue = readData();
     else
     {	/* Start conversion and read result */
         cValue = readData();
         /* Start conversion and read result */
         cValue = readData();
     }
 #ifdef ADS1018
     /* Temperature is 12-bit left-justified so need to get the correct value �¶���12λ����룬������Ҫ�õ���ȷ��ֵ*/
     cValue = cValue >> 4;
     /* Use proper coefficient to get the temperatureʹ���ʵ���ϵ���õ��¶� */
     temp_C = cValue * 0.125;
 #else
     /* Temperature is 14-bit left-justified so need to get the correct value�¶���14λ����룬������Ҫ�õ���ȷ��ֵ */
     cValue = cValue >> 2;
     /* Use proper coefficient to get the temperatureʹ���ʵ���ϵ���õ��¶� */
     temp_C = cValue * 0.03125;
 #endif
     /* Restore the previous configuration�ָ���ǰ������ */
     writeSingleRegister(CONFIG_ADDRESS, regValue);
 
     return temp_C;
 }
 //*********************api************************************* */
  /************************************************************************//**
  * @brief 启动指定通道测量
  * @param adChannel,指定通道号,0冷端；1：K1； 2：K2；
  * @note   
  * @retval 
  *****************************************************************************/
 void app_ads1118_channel_sampling_start(unsigned char adChannel)
 {
    uint16_t regValue = 0; 
	if(adChannel==0)//le
    {     
        regValue=CONFIG_SS_CONV_START |CONFIG_PGA_0p256V |CONFIG_DR_250SPS| registerMap[CONFIG_ADDRESS]|TS_MODE_TS;//冷端温度           
    }
    else  if(adChannel==1)
    {   
        regValue=CONFIG_SS_CONV_START | CONFIG_PGA_0p256V|CONFIG_DR_250SPS|CONFIG_MUX_AIN0_AIN1|(registerMap[CONFIG_ADDRESS])&0xFFEF;//CONFIG_MUX_AIN0_AIN1         
    }
    else
    {    
        regValue=CONFIG_SS_CONV_START | CONFIG_PGA_0p256V|CONFIG_DR_250SPS|registerMap[CONFIG_ADDRESS]|TS_MODE_ADC|CONFIG_MUX_AIN2_AIN3;//ADC2, 3      
    }   
    writeSingleRegister(CONFIG_ADDRESS, regValue);   
 }
   /************************************************************************//**
  * @brief 读取指定通道测量值
  * @param adChannel,指定通道号,0冷端；1：K1； 2：K2；
  * @note   
  * @retval 该通道测量值
  *****************************************************************************/
 float  app_ads1118_channel_get_value(unsigned char adChannel)
 {
    float ret_tmprature;
    // Monitor DRDY
    // and hold CS low
    //setCS(LOW);   
    // HAL_Delay(1); 
    // waitForDRDYinterrupt(1000);
    // HAL_Delay(2);
    //setCS(HIGH);
    // Read conversion results

    registerMap[CONVERSION_ADDRESS] = readSingleRegister(CONVERSION_ADDRESS);  
    if(adChannel==0)
    {         
        tmprature_T0_cold_junction=((int16_t) registerMap[CONVERSION_ADDRESS]>>2)*0.03125;//冷端补偿
       // DEBUG_PRINTF("T0=%.1fad=%d\r\n", tmprature_T0_cold_junction,(int16_t) registerMap[CONVERSION_ADDRESS]); 
       if(fabs(history_tmprature_T0-tmprature_T0_cold_junction)>0.2) 
       {
        ret_tmprature = tmprature_T0_cold_junction;
        history_tmprature_T0=tmprature_T0_cold_junction;         
       }
       else  ret_tmprature = history_tmprature_T0;  
    }
    else  if(adChannel==1)
    {  
        tmprature_T1_cool_water=tmprature_cal((int16_t) registerMap[CONVERSION_ADDRESS]);//循环水温度
       //DEBUG_PRINTF("T1=%.1fad=%d\r\n", tmprature_T1_cool_water,(int16_t) registerMap[CONVERSION_ADDRESS]);       
        if(fabs(history_tmprature_T1-tmprature_T1_cool_water)>0.2) 
        {
         ret_tmprature = tmprature_T1_cool_water;
         history_tmprature_T1=tmprature_T1_cool_water;          
        }
        else  ret_tmprature = tmprature_T1_cool_water;  

    }
    else
    {   
        tmprature_T2_laser=tmprature_cal((int16_t) registerMap[CONVERSION_ADDRESS]);//激光器温度     
      //  DEBUG_PRINTF("T2=%.1fad=%d\r\n",tmprature_T2_laser,(int16_t) registerMap[CONVERSION_ADDRESS]);
        if(fabs(history_tmprature_T2-tmprature_T2_laser)>0.2) 
        {
         ret_tmprature = tmprature_T2_laser;
         history_tmprature_T1=tmprature_T2_laser;          
        }
        else  ret_tmprature = tmprature_T2_laser;  
    }     
    return ret_tmprature;
 }

  /************************************************************************//**
  * @brief 读取指定通道测量值
  * @param adChannel,指定通道号,0冷端；1：K1； 2：K2；
  * @note   
  * @retval 该通道测量值
  *****************************************************************************/
 void app_ads1118_startup(void)
 {
    adcStartup();
 }