/*
 * EEROM_M24C32_bsp.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Hql2017
 */
#include "main.h"
#include "i2c.h" 
#include "eeprom_bsp.h"

//The 7-bit IIC address 
#define  EEPROM_REAL_ADDR   0x51<<1 //0x50 EEPROM 

#define  EEPROM_M24C32_MAX_BYTE_LENGTH   4096//(1024*4)-1  //4096Byte
#define 	EEPROM_M24C32_PAGE_LENGTH    32 //128ҳ

#define   EEPROM_I2C_TIMEOUT     100

//д�����ź�
#define  EEPROM_WC_ENABLE  HAL_GPIO_WritePin(EEROM_W_EN_out_GPIO_Port, EEROM_W_EN_out_Pin, GPIO_PIN_RESET)
#define  EEPROM_WC_DISABLE  HAL_GPIO_WritePin(EEROM_W_EN_out_GPIO_Port, EEROM_W_EN_out_Pin, GPIO_PIN_SET)

#if 0

#define I2CT_FLAG_TIMEOUT 0x1000
//IIC_SDA��IO��������
#define SDA_IN()   GPIOModeSet(GPIO_Mode_Out_OD);//GPIOModeSet(GPIO_Mode_IN_FLOATING)	//PB7����ģʽ
#define SDA_OUT()  GPIOModeSet(GPIO_Mode_Out_OD) //PB7���ģʽ
//IO��������	 
#define IIC_SCL_L()     		GPIO_ResetBits(GPIOB,GPIO_Pin_6)      //SCL
#define IIC_SCL_H()       		GPIO_SetBits(GPIOB,GPIO_Pin_6)     //SCL
#define IIC_SDAOUT_H()  	  	GPIO_SetBits(GPIOB,GPIO_Pin_7)       //���SDA
#define IIC_SDAOUT_L()  	  	GPIO_ResetBits(GPIOB,GPIO_Pin_7)       //���SDA
#define IIC_SDAIN()       GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)//����SDA 
//IIC��ӦIO�ڵĳ�ʼ��
void IIC_Simulate_Init(void)
{		
#if USE_HARDWARE_IIC  
	IIC_Hard_Init();	
#else
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //ʹ��GPIOBʱ��
  //GPIOB6��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIOB7,SDA��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
//	GPIO_SetBits(GPIOB,GPIO_Pin_6);
//	GPIO_SetBits(GPIOB,GPIO_Pin_7);
  IIC_Stop();
#endif
}
/*******************************************************************************
*************************����ΪIO��ģ��IICͨ��**********************************
*******************************************************************************/
//IIC��ʼ�ź�
void IIC_Start(void)
{
	//SDA_OUT();     //����SDA��Ϊ���
	IIC_SCL_L() ;	//����ʱ���߻�ÿ���Ȩ
	delay_us(1);	
	IIC_SDAOUT_H();	
	delay_us(1);	
	IIC_SCL_H() ;
	delay_us(1);
 	IIC_SDAOUT_L();
	delay_us(1);
	IIC_SCL_L();     //׼�����ͻ�������� 
	delay_us(1);
}	

//IICֹͣ�ź�
void IIC_Stop(void)
{
	//SDA_OUT();    //����SDA��Ϊ���
	IIC_SDAOUT_L();
  delay_us(1); 	
	IIC_SCL_H() ;
	delay_us(1);				
	IIC_SDAOUT_H(); //����I2C���߽����ź�
	delay_us(1);	
}
/****************************************************************************
* ��    ��: uint8_t MCU_Wait_Ack(void)
* ��    �ܣ�MCU�ȴ����豸Ӧ���źŵ���
* ��ڲ�������
* ���ز�����1:����Ӧ��ʧ��  0:����Ӧ��ɹ�
* ˵    ����  ?B
****************************************************************************/
uint8_t MCU_Wait_Ack(void)
{
	uint8_t WaitTime=0,ret=0;
	//SDA_IN(); //����SDA��Ϊ���� 
	IIC_SDAOUT_H() ;
	delay_us(1);
	 IIC_SCL_H() ;   
	delay_us(1);	 
	while(IIC_SDAIN())
	{
		WaitTime++;
		if(WaitTime>200)
		{
			WaitTime=0;
			IIC_Stop();
			return 1;
		}
	}
	ret=0;
	IIC_SCL_L() ; 
	delay_us(1);
	return ret;  
}

/****************************************************************************
* ��    ��: uint8_t void MCU_Send_Ack(void)
* ��    �ܣ�MCU����ACKӦ��,��֪24cxx
* ��ڲ�������
* ���ز�����
* ˵    ����  ?B
****************************************************************************/
void MCU_Send_Ack(void)
{
	//SDA_OUT();		
	IIC_SDAOUT_L();
	delay_us(1);
	IIC_SCL_H() ;
	delay_us(1);
	IIC_SCL_L() ;
  delay_us(1);
	IIC_SDAOUT_H();//�ͷ�SDA���� 
}

/****************************************************************************
* ��    ��: uint8_t void MCU_Send_Ack(void)
* ��    �ܣ�MCU������ACKӦ��	
* ��ڲ�������
* ���ز�����
* ˵    ����  ?B
****************************************************************************/  
void MCU_NOAck(void)
{
  //IIC_SCL_L() ;
	SDA_OUT();
	IIC_SDAOUT_H() ;
	delay_us(1);
	IIC_SCL_H() ;
	delay_us(1);
	IIC_SCL_L();
	delay_us(1);	
}	
/****************************************************************************
* ��    ��: void IIC_write_OneByte(uint8_t Senddata)
* ��    �ܣ�IICдһ���ֽڵ�EEPROM	
* ��ڲ�����Senddata:д���8λ����
* ���ز�����
* ˵    ����  ?B
****************************************************************************/	  
void IIC_write_OneByte(uint8_t Senddata)
{  
#if USE_HARDWARE_IIC
//Ӳ��д�ֽ�	
#else
    uint8_t t;   
	  //SDA_OUT(); 	    
    IIC_SCL_L() ;    //����ʱ�ӿ�ʼ���ݴ���
    delay_us(1);
    for(t=0;t<8;t++)
    { 
			if((Senddata&0x80)!=0)		
			{
				IIC_SDAOUT_H();
			}	
			else
			{
				IIC_SDAOUT_L();
			}
			Senddata<<=1; 
			delay_us(1);   
			IIC_SCL_H();
			delay_us(1);
			IIC_SCL_L();
			delay_us(1);
    }	 
#endif
} 
/****************************************************************************
* ��    ��: void IIC_Read_OneByte(uint8_t Senddata)
* ��    �ܣ�IIC��ȡһ���ֽ�
* ��ڲ�����ack=1������ACK��ack=0������nACK 
* ���ز�����������8λ����
* ˵    ����  ?B
****************************************************************************/	  
uint8_t IIC_Read_OneByte(uint8_t ack)
{
	uint8_t i,receivedata=0;
#if USE_HARDWARE_IIC  
	//Ӳ����	
#else		
		//SDA_IN();       //����SDA��Ϊ����
    for(i=0;i<8;i++ )
		{	
			receivedata<<=1;	
		  IIC_SCL_H() ;	
			delay_us(1);				
			if(IIC_SDAIN()) receivedata+=1; 						
			IIC_SCL_L(); 
			delay_us(1);						
    }
		//SDA_OUT();       //����SDA��Ϊ��		
    if (!ack)
        MCU_NOAck();//����nACK
    else
        MCU_Send_Ack(); //����ACK  
#endif		
    return receivedata;
}
/*******************************IO��ģ��IIC*************************************
*******************************************************************************/
#endif
/**
  * @brief EEPROM_M24C32 初始化
  * @param void
  *         
  * @note   无
  * @retval None
  */
 void EEPROM_M24C32_init(void)
{  
	unsigned char status;
  app_I2C_start(&hi2c3);
	status=EEPROM_M24C32_Test();
	DEBUG_PRINTF("24C32 init ok=%x",status);	
}
/**
  * @brief EEPROM_M24C32_I2C_Read
  * @param   uint16_t addr:device IIC address
            uint8_t* rBuf��receive data buff 
            uint16_t len:data length
  * @note   EEPROM_M24C32_I2C_Read 
  * @retval None
  */
static HAL_StatusTypeDef EEPROM_M24C32_I2C_Read(unsigned short int regAddr,unsigned char *buff,unsigned short int len)
{
	HAL_StatusTypeDef err;
	err=HAL_I2C_Mem_Read(&hi2c3,EEPROM_REAL_ADDR,regAddr,I2C_MEMADD_SIZE_16BIT,buff,len,EEPROM_I2C_TIMEOUT); 
	return err;	
}
/**
  * @brief EEPROM_M24C32_I2C_Write
  * @param   uint16_t addr:device IIC address��
            uint8_t* rBuf��receive data buff 
            uint16_t len:data length
  * @note   EEPROM_M24C32_I2C_Write
  * @retval None
  */
static HAL_StatusTypeDef EEPROM_M24C32_I2C_Write(unsigned short int regAddr,unsigned char *buff,unsigned short int len)
{
	HAL_StatusTypeDef err;
	err=HAL_I2C_Mem_Write(&hi2c3,EEPROM_REAL_ADDR,regAddr,I2C_MEMADD_SIZE_16BIT,buff,len,EEPROM_I2C_TIMEOUT);  
	return err; 
}


/****************************************************************************
* ��    ��: ҳдuint8_t EEPROM_M24C32_Test(void)
* ��    �ܣ�����EEPROM_M24C32��ҳд�Ƿ�����
* ��ڲ�������
* ���ز���������1:���ʧ��
            ����0:���ɹ� 
* ˵    ����  ?B
****************************************************************************/
uint8_t EEPROM_M24C32_PageTest(void)
{
	uint8_t Testdat[100]={2},i;	
	for(i=0;i<100;i++)
	{
		Testdat[i]=5;
	}
	EEPROM_M24C32_I2C_Write(384,Testdat,100);
//	Testdata=EEPROM_M24C32_ReadByte(0); 	
//	if(Testdata==0xAC) 	return 0;
//	else                             
//	{
//		EEPROM_M24C32_WriteByte(0,0XAC);
//	  	Testdata=EEPROM_M24C32_ReadByte(0);
//		if(Testdata==0xAC) return 0;
//	}
	return 1;									  
}
/****************************************************************************
* ��    ��: uint32_t Buf_4Byte(uint8_t *pBuffer,uint32_t Date_4Byte,uint8_t Byte_num,uint8_t mode)
* ��    �ܣ���λ�����ֽڻ�ת
* ��ڲ�����mode��1:��λ��ת�ֳ��ֽ�   0:�ֽںϲ���һ����λ��
            Byte_num����Ҫת�����ֽ���
            *pBuffer���ֽڽ���������ֽ���������
            Date_4Byte����λ����
* ���ز�����modeΪ0ʱ�����ض�λ��
* ˵    ����Byte_num���Ϊ4���ֽڣ��ú����ں���Ĵ�����У��ʱ��ȡУ��ֵ���õ�
****************************************************************************/
uint32_t Buf_4Byte(uint8_t *pBuffer,uint32_t Date_4Byte,uint8_t Byte_num,uint8_t mode)
{
	uint8_t i; uint32_t middata=0;
	if(mode)    
	{
		for(i=0;i<Byte_num;i++)
		{
			*pBuffer++ =(Date_4Byte>>(8*i))&0xff;
		}
		return 0; 
	} 
	else       
	{
		Date_4Byte=0;
		pBuffer+=(Byte_num-1);
		for(i=0;i<Byte_num;i++)
		{ 		
			middata<<=8;
			middata+= *pBuffer--;			   
		}
		return middata;	
	}
}
/****************************************************************************
* ��    ��: void EEPROM_M24C32_Read(uint8_t ReadAddr,uint8_t *pBuffer,uint16_t ReadNum)
* ��    �ܣ���EEPROM_M24C32�����ָ����ַ��ʼ����ָ������������,
* ��ڲ�����ReadAddr :��ʼ�����ĵ�ַ  0~255
            pBuffer  :���������׵�ַ
            ReadNum:Ҫ�������ݵĸ���
* ���ز�����
* ˵    ����  ?B
****************************************************************************/
unsigned char EEPROM_M24C32_Read(unsigned short int ReadAddr, unsigned char *pBuffer, unsigned short int ReadNum)
{
	unsigned int i;
  HAL_StatusTypeDef err=	EEPROM_M24C32_I2C_Read(ReadAddr,pBuffer,ReadNum);
	if(err!=HAL_OK)
	{
		return 1;
	}
	return 0;	//�ɹ�
}
/****************************************************************************
* ��    ��: void EEPROM_M24C32_Write(uint8_t WriteAddr,uint8_t *pBuffer,uint16_t WriteNum)

* ��    �ܣ���EEPROM_M24C32�����ָ����ַ��ʼд��ָ������������
* ��ڲ�����WriteAddr :��ʼд��ĵ�ַ  0~255
            pBuffer  :���������׵�ַ
            WriteNum:Ҫд�����ݵĸ���
* ���ز�����errCode 0:ok 1:�ռ䲻��
* ˵    ����  ?B
****************************************************************************/
unsigned char EEPROM_M24C32_Write(unsigned short int WriteAddr,unsigned char *pBuffer,unsigned short int WriteNum)
{
	uint16_t i,pageNum,j;
	if((WriteNum+WriteAddr)>EEPROM_M24C32_MAX_BYTE_LENGTH)  return 1;//����Χ
	i=EEPROM_M24C32_PAGE_LENGTH-(WriteAddr%EEPROM_M24C32_PAGE_LENGTH);//��ҳʣ��ռ�
	EEPROM_WC_ENABLE;
	if(i<WriteNum)//��Ҫ��ҳ
	{
		EEPROM_M24C32_I2C_Write(WriteAddr,pBuffer,i);//��ҳ
		WriteAddr+=i;
		pBuffer+=i;
		WriteNum-=i;
		pageNum=WriteNum/EEPROM_M24C32_PAGE_LENGTH;
		while(pageNum)
		{
			EEPROM_M24C32_I2C_Write(WriteAddr,pBuffer,EEPROM_M24C32_PAGE_LENGTH);			
			pageNum--;
			WriteAddr+=EEPROM_M24C32_PAGE_LENGTH;
			pBuffer+=EEPROM_M24C32_PAGE_LENGTH;	      			
		}	
	}
	j=WriteNum%EEPROM_M24C32_PAGE_LENGTH;
	if(j)  EEPROM_M24C32_I2C_Write(WriteAddr,pBuffer,j);//��󲿷�	
	HAL_Delay(10);	//�ȴ�д�� 	
	if(WriteNum>59) HAL_Delay(5);//�ֽ�̫���ٵȴ�5ms
	EEPROM_WC_DISABLE;
	return 0;		
}
/****************************************************************************
* ��    ��: void EEPROM_M24C32_PageRead(uint16_t page,uint8_t *pBuffer,uint8_t ReadNum)
* ��    �ܣ���EEPROM_M24C32�����ָ��ҳ������,
* ��ڲ�����page :��ʼ�����ĵ�ַ  0~255
            pBuffer  :���������׵�ַ
            ReadNum:Ҫ�������ݵĸ���
* ���ز�����
* ˵    ����  ?B
****************************************************************************/
void EEPROM_M24C32_PageRead(uint16_t page,uint8_t *pBuffer,uint8_t ReadNum)
{
	EEPROM_M24C32_I2C_Read(page*EEPROM_M24C32_PAGE_LENGTH,pBuffer,ReadNum);
	HAL_Delay(5);	
} 
/****************************************************************************
* ��    ��: uint8_t EEPROM_M24C32_Test(void)
* ��    �ܣ�����EEPROM_M24C32�Ƿ�����
* ��ڲ�������
* ���ز���������1:���ʧ��
            ����0:���ɹ� 
* ˵    ����  ?B
****************************************************************************/
uint8_t EEPROM_M24C32_Test(void)
{
	uint8_t Testdata,err;
	err =EEPROM_M24C32_Read(0,&Testdata,1); 
	if(err==0) return 0;
//	if(Testdata==0xAC) 	return 0;//
//	else                             
//	{
//		Testdata=0xAC;
//		EEPROM_M24C32_Write(0,&Testdata,1);
//		HAL_Delay(5);
//	  	EEPROM_M24C32_Read(0,&Testdata,1);	
//		if(Testdata==0xAC) return 0;
//	}
	return 0;;									  
}
/****************************************************************************
* ��    ��: 按字读取
* ��    �ܣ���EEPROM_M24C32�����ָ����ַ��ʼ����ָ������������,
* ��ڲ�����ReadAddr :��ʼ�����ĵ�ַ  0~255
            pBuffer  :���������׵�ַ
            ReadNum:Ҫ�������ݵĸ���
* ���ز�����
* ˵    ����  ?B
****************************************************************************/
unsigned char EEPROM_M24C32_Read_WORD(unsigned short int ReadAddr, unsigned int *pBuffer, unsigned short int ReadNum)
{
	unsigned int i;
	unsigned char *pByteBuffer;
	unsigned short int len;
	len=ReadNum*4;
  	EEPROM_M24C32_I2C_Read(ReadAddr,pByteBuffer,len);
	for(int i=0;i<ReadNum;i++)
	{
		*pBuffer=(pByteBuffer[0]<<24)|(pByteBuffer[1]<<16)|(pByteBuffer[2]<<8)|pByteBuffer[3];		
		pByteBuffer+=4;
		pBuffer++;
	}

	return 0;	//�ɹ�
}
/****************************************************************************
* ��    ��: 按字写
* ��    �ܣ���EEPROM_M24C32�����ָ����ַ��ʼ����ָ������������,
* ��ڲ�����ReadAddr :��ʼ�����ĵ�ַ  0~255
            pBuffer  :���������׵�ַ
            ReadNum:Ҫ�������ݵĸ���
* ���ز�����
* ˵    ����  ?B
****************************************************************************/
unsigned char EEPROM_M24C32_write_WORD(unsigned short int writeAddr, unsigned int *pBuffer, unsigned short int WriteNum)
{
	unsigned int i;
	unsigned char *pByteBuffer;
	unsigned short int len;
	len=WriteNum*4;
	for(int i=0;i<WriteNum;i++)
	{
		pByteBuffer[WriteNum*4]  =*pBuffer>>24;
		pByteBuffer[1+4*WriteNum]=*pBuffer>>16;
		pByteBuffer[2+4*WriteNum]=*pBuffer>>8;
		pByteBuffer[3+4*WriteNum]=*pBuffer;	
		pBuffer++;
	}
	EEPROM_M24C32_I2C_Write(writeAddr,pByteBuffer,WriteNum*4);

	return 0;	//�ɹ�
}