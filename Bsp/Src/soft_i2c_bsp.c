

#include "soft_i2c_bsp.h"

/*================================= */

#define SDA_H     HAL_GPIO_WritePin(V160_I2C1_SDA_GPIO_Port, V160_I2C1_SDA_Pin, GPIO_PIN_SET)
#define SCL_H     HAL_GPIO_WritePin(V160_I2C1_SCK_GPIO_Port, V160_I2C1_SCK_Pin, GPIO_PIN_SET)

#define SDA_L     HAL_GPIO_WritePin(V160_I2C1_SDA_GPIO_Port, V160_I2C1_SDA_Pin, GPIO_PIN_RESET)
#define SCL_L     HAL_GPIO_WritePin(V160_I2C1_SCK_GPIO_Port, V160_I2C1_SCK_Pin, GPIO_PIN_RESET)

#define SDA_read  HAL_GPIO_ReadPin(V160_I2C1_SDA_GPIO_Port,V160_I2C1_SDA_Pin)

 uint8_t  data[64];
/* Private function prototypes -----------------------------------------------*/
static uint32_t I2C_Speed =SOFT_I2C400KHz;// SOFT_I2C400KHz;//10khZ    6;   //400KHz

void soft_I2C_GPIO_Init(void)
{	 
  GPIO_InitTypeDef GPIO_InitStructure;	
	__HAL_RCC_GPIOD_CLK_ENABLE();	
  GPIO_InitStructure.Pin = V160_I2C1_SDA_Pin|V160_I2C1_SCK_Pin;
  GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH ;
  GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_OD;   
  SDA_H;
  SCL_H;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/**/
static void I2C_delay(int cnt)
{
    delay_us(cnt);
//while(cnt--);// This sentence is shielded,I2C speed is 800KHz
}
/*********************************************************************************************************/
static int soft_I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay(I2C_Speed);
    if(!SDA_read)
    {
        return FALSE1;
    }
    SDA_L;
    I2C_delay(I2C_Speed);
    if(SDA_read)
    {
        return FALSE1;
    }
    SCL_L;
    I2C_delay(I2C_Speed);
		
	
    return TRUE1;
}
/*********************************************************************************************************/
static void soft_I2C_Stop(void)
{
    SCL_L;
    I2C_delay(I2C_Speed);
    SDA_L;
    I2C_delay(I2C_Speed);
    SCL_H;
    I2C_delay(I2C_Speed);
    SDA_H;
    I2C_delay(I2C_Speed);
}
/*********************************************************************************************************/
static void soft_I2C_Ack(void)
{
    SCL_L;
    SDA_L;
    I2C_delay(I2C_Speed);
    SCL_H;
    I2C_delay(I2C_Speed);
    SCL_L;
}
/*********************************************************************************************************/
static void soft_I2C_NoAck(void)
{
    SCL_L;
    SDA_H;
    I2C_delay(I2C_Speed);
    SCL_H;
    I2C_delay(I2C_Speed);
    SCL_L;
}
/*********************************************************************************************************/
static int soft_I2C_WaitAck(void)
{
    SCL_L;
    SDA_H;
    I2C_delay(I2C_Speed);
    SCL_H;
    I2C_delay(I2C_Speed);
    if(SDA_read)
    {
        SCL_L; 
        return FALSE1;
    }
    SCL_L;
    return TRUE1;
}
/*********************************************************************************************************/
void soft_I2C_SendByte(uint8_t  SendByte)
{
    uint8_t  i;
	 SCL_L;
		I2C_delay(I2C_Speed);
    for(i=0; i<8; i++)
    {       
        if(SendByte&0x80)
            SDA_H;
        else
            SDA_L;
        SendByte<<=1;
        I2C_delay(I2C_Speed);
        SCL_H;
        I2C_delay(I2C_Speed);  
				SCL_L;
			I2C_delay(I2C_Speed);				
    }
//Please let SDA open drain here	
		SDA_H;		
   // SCL_L;
}
/*********************************************************************************************************/
uint8_t  soft_I2C_ReceiveByte(void)
{
    uint8_t  i=8;
    uint8_t  ReceiveByte=0;
    SDA_H;
    while(i--)
    {
        ReceiveByte<<=1;
        SCL_L;
        I2C_delay(I2C_Speed);
        SCL_H;
        I2C_delay(I2C_Speed);
        if(SDA_read)
        {
            ReceiveByte|=0x01;
        }
    }
    SCL_L;
    return ReceiveByte;
}

/*********************************************************************************************************/
uint8_t  soft_I2C_WriteByte(int DeviceAddress, unsigned char  SendByte)
{
    uint8_t  Resend_Flag=0;
    uint8_t  n=2;
    if(!soft_I2C_Start())return FALSE1;
    soft_I2C_SendByte(DeviceAddress & 0xFE);//mask the LSB
    if(!soft_I2C_WaitAck()) {
        Resend_Flag=1;
    }  
    soft_I2C_SendByte(SendByte);
    if(!soft_I2C_WaitAck()) {
        Resend_Flag=1;
    }
    soft_I2C_Stop();
    while((1==Resend_Flag)&&(n--)&&data[60]==0)//resend n times  data[60]==0 I2C need resend
    {
        Resend_Flag=0;
        if(!soft_I2C_Start())return FALSE1;
        soft_I2C_SendByte(DeviceAddress & 0xFE);
        if(!soft_I2C_WaitAck()) {
            Resend_Flag=1;
        }        
        soft_I2C_SendByte(SendByte);
        if(!soft_I2C_WaitAck()) {
            Resend_Flag=1;
        }
        soft_I2C_Stop();
    }
    if(1==Resend_Flag)return FALSE1;
    else return TRUE1;
}

/*********************************************************************************************************/
uint8_t  soft_I2C_WriteBuffer(uint8_t * pBuffer, int length, int DeviceAddress )
{
    unsigned char Resend_Flag=0;
    int length_local=length;
    int byte_count=0;
    if(!soft_I2C_Start())return FALSE1;
    soft_I2C_SendByte(DeviceAddress & 0xFE);
    if(!soft_I2C_WaitAck())//first check ack
    {
        soft_I2C_Stop();
        if(!soft_I2C_Start())return FALSE1;//re-start
        soft_I2C_SendByte(DeviceAddress & 0xFE);//send second time
        if(!soft_I2C_WaitAck())//second check ack
        {
            soft_I2C_Stop();
            if(!soft_I2C_Start())return FALSE1;
            soft_I2C_SendByte(DeviceAddress & 0xFE);//send third time
            if(!soft_I2C_WaitAck())
            {
                soft_I2C_Stop();//release IIC bus
                return FALSE1;//third check ack, if still non-ack, return false
            }
        }
    }   
    while(length_local--)
    {
        soft_I2C_SendByte(*pBuffer);
        if(!soft_I2C_WaitAck()) {
            Resend_Flag=1;
        }
        pBuffer++;
        byte_count++;
        if(Resend_Flag==1)break;
    }
    soft_I2C_Stop();

    if(Resend_Flag==1)
    {
        Resend_Flag=0;
        length_local=length;
        pBuffer=pBuffer-byte_count;
        if(!soft_I2C_Start())return FALSE1;//re-start
        soft_I2C_SendByte(DeviceAddress & 0xFE);//send second time
        if(!soft_I2C_WaitAck())
        {
            soft_I2C_Stop();
            if(!soft_I2C_Start())return FALSE1;
            soft_I2C_SendByte(DeviceAddress & 0xFE);//send third time
            if(!soft_I2C_WaitAck())
            {
                Resend_Flag=1;
                soft_I2C_Stop();//release IIC bus
                return FALSE1;//check ack, if still non-ack, return false
            }
        }        
        while(length_local--)
        {
            soft_I2C_SendByte(*pBuffer);
            if(!soft_I2C_WaitAck())
            {
                Resend_Flag=1;
                soft_I2C_Stop();//release IIC bus
                return FALSE1;//check ack, if still non-ack, return false
            }
            pBuffer++;
        }
        soft_I2C_Stop();
    }
    if(1==Resend_Flag)return FALSE1;
    else return TRUE1;
}
/**********************************

***********************************************************************/
uint8_t  soft_I2C_ReadByte(int DeviceAddress)
{
    uint8_t  rec;
    if(!soft_I2C_Start())return FALSE1;    
    soft_I2C_SendByte(DeviceAddress | 0x01);
    soft_I2C_WaitAck();
    rec=soft_I2C_ReceiveByte();
    soft_I2C_NoAck();
    soft_I2C_Stop();
    return rec;
}

/*********************************************************************************************************/
uint8_t  soft_I2C_ReadBuffer(uint8_t * pBuffer,   int length, int DeviceAddress)
{
    if(length == 0)
        return FALSE1;
    if(!soft_I2C_Start())return FALSE1; 		
    soft_I2C_SendByte(DeviceAddress | 0x01);
    soft_I2C_WaitAck();		
    while(length--)
    {
       *pBuffer = soft_I2C_ReceiveByte();
       if(length == 0)soft_I2C_NoAck();
       else soft_I2C_Ack();
        pBuffer++;
    }
    soft_I2C_Stop();
    return TRUE1;
}
