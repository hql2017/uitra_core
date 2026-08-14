#include "hsm_iii_bsp.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 
#include <usart.h>
#include "can1.h"
#include "fdcan.h"

#include "cmsis_os.h"
#include "lwrb.h"

 osThreadId  hmiTaskHandle;
 osThreadId  laserTaskHandle;
 void hmiTask(void const *argument);
 void laserTask(void const *argument);
/*��������֡ʹ���жϷ�ʽ��������֡���������������ݡ� */
#define MAX_LWRB_UART_QUEN  256
static uint8_t lwrb_rx_buff[MAX_LWRB_UART_QUEN];  // 
static lwrb_t rs232_lwrb;//
static uint8_t lwrb_len=0;
static uint8_t RS232_rx_data[8];  // ���ջ�����
uint32_t foot_Switch_mark = 0;
Frame_t frame;//����Э��ṹ������

uint8_t RS232_tx_data[32] = {0x19,  //[0] ʾ������
															0xEC,  //[1] ʾ������
															0x00,  //[2] Ԥ��IO1״̬
															0x00,  //[3] Ԥ��IO2״̬
															0x00,  //[4] ʾ������
															0x00,  //[5] ʾ������
															0x00,  //[6] ʾ������
															0x00,  //[7] ʾ������
															//		
															0x01,  //[8] ʾ������
															0x00,  //[9] ʾ������
															0x00,  //[10] ʾ������
															0x01,  //[11] Ԥȼ״̬0ʧ�ܣ�1�ɹ�
															0x00,  //[12] ʾ������
															0x00,  //[13] ʾ������
															0x00,  //[14] ʾ������
															0x00,  //[15] ʾ������
															//	
															0x00,  //[16] ʾ������
															0x00,  //[17] ʾ������
															0x00,  //[18] ʾ������
															0x00,  //[19] ʾ������
															0x00,  //[20] ʾ������
															0x00,  //[21] ʾ������
															0x00,  //[22] ʾ������
															0x00,  //[23] ʾ������
															//		
															0x00,  //[24] ʾ������
															0x00,  //[25] ʾ������
															0x00,  //[26] ʾ������
															0x00,  //[27] ʾ������
															0x00,  //[28] ʾ������
															0x00,  //[29] ʾ������
															0x00,  //[30] ʾ������
															0x00,  //[31] ʾ������
															};
/* A�巢�����ݵ�B��,0x11���� 0x1E�쳣 */

DEVICE_STATUS               device_status;
LOCAL_LASER_CONFIG_PARARM   laser_config_param;	

//Э������
void INIT_Fill_Frame(void) {
    // ��ʼ��־ (0x55AA)
    frame.headerH = 0x55; // ���ֽ�
    frame.headerL = 0xAA; // ���ֽ�

    // Э�����ͺ�ָ������
    frame.protocol_type = 0x00; // ʾ��ֵ
    frame.cmd_type = 0x01;      // ʾ��ֵ

    // �������� (0x1234)
    frame.SEQH = 0x00; // ���ֽ�
    frame.SEQL = 0x00; // ���ֽ�

    // ���ݳ��� (0x0020)
    frame.data_lengthH = 0x00; // ���ֽ�
    frame.data_lengthL = 0x20; // ���ֽ�

    // ��������
    memset(frame.data, 0,32);// sizeof(frame.data)); // ��ʼ��Ϊ 0
	
		//У���
		frame.checksum = Calculate_Checksum(&frame); //0x00;
}

// ��� data[32] ����
void Fill_Data(Frame_t *myframe, uint8_t *data) {
    // �����ݿ������ṹ����
    memcpy(myframe->data, data, 32);

    // ����У���
    myframe->checksum = Calculate_Checksum(myframe);
}

/*����У���*/
uint8_t Calculate_Checksum(Frame_t *userframe) {
    uint8_t checksum = 0;
    uint8_t *data = (uint8_t*)userframe;
    for (size_t i = 0; i < sizeof(Frame_t) - 1; i++) { // ������У����ֶ�
        checksum += data[i];
    }
    return checksum; // ����У��͵ķ���
}

void send_data_to_TFT(uint8_t *data,uint8_t len)
{
		Frame_t sFram;
		sFram.data_lengthH=0x55;
		sFram.data_lengthL=0xAA;
		sFram.protocol_type=0x00;//����ָ��
		sFram.cmd_type=0x01;//����ָ��
		sFram.SEQH=0x00;
		sFram.SEQL=0x00;
		sFram.data_lengthH=len/256;
		sFram.data_lengthL=len%256;
	//*(sFram.data)=*data;
	//Fill_Data(&frame, data);
	//��䱾������		
		if(len>32) len=32;//�̶�32λ
		memcpy(sFram.data,data,len);
		sFram.checksum=Calculate_Checksum(&sFram);	
		HAL_UART_Transmit(&hlpuart1,(uint8_t *)&sFram,sizeof(Frame_t),1000);	//���ͽ��յ�������			
}

// ����У���
uint8_t Calculate_Checksum_Rx(uint8_t *data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum; // ����У��͵ķ���
}

// �ж�֡ͷ�����Ƿ�Ϊ 0x55AA0001
bool Check_Header(uint8_t *data) {
    uint16_t header = (data[0] << 8) | data[1];
    return (header == 0x55AA); // �ж��Ƿ���� 0x55AA
}
// �ж�У����Ƿ�һ��
bool Verify_Checksum(uint8_t *data, size_t length) {
    uint8_t calculated_checksum = Calculate_Checksum_Rx(data, length - 1); // �ų�У����ֶ�
    return (calculated_checksum == data[length - 1]); // �Ƚϼ���ֵ�����ֵ
}
//���⹤��״̬����
void laser_status_handle(void)
{		
	//laser_menu_change_flag=1;//״̬�����־
}

//���ݽ���,�̵���
void reg0_1_relay_handle(uint16_t info)
{	
	uint8_t temp[8]={0};
	//?��??��???????
	if(info==0x02)//???��
	{	
		device_status.relay_stauts=0x0002;//?��??��???		
	}
	else if(info==0x00)//???��
	{//??��??????��
		device_status.relay_stauts=0x0000;//?��??��???
	}	
	else
	{
		device_status.relay_stauts=info;//?��??��???
	}	
//	if((laser_config_param.relay_control_info&0x0001)!=flag_rgb)		
	temp[0]=(info&0x01);//??????????,?????��1KHz
	//CAN_APP_SEND_DATA(CAN_SET_MCU2_GX_LED_MODE,temp,CAN_MCU2_STATUS_ID);
  //return ack��		
}
//���ݽ���,�̵���
void reg2_lowVoltage_handle(uint8_t lowVoltageInfo)
{	
	uint8_t i;
	if(lowVoltageInfo&0x01)//1
	{		  
	}
	else 
	{
	}	
}

//�Ƿ�����������
void reg3_pulse_handle(uint8_t pulseCount)
{
//	//�򿪸�ѹ��Դ�ͷ��ȽӴ���,��Э�鲻ͬ
//    if(data[3] == 0x01)
//		{			
//			HV_POWER_ON;//�򿪸�ѹ��Դ�ͷ��ȽӴ���
//		}
//		else
//			HV_POWER_OFF;//�򿪸�ѹ��Դ�ͷ��ȽӴ���
	if(pulseCount&0x01)//1
	{		  
	}
	else 
	{
	}	
}
 //reg4,Э������û�ã���Ҫ����
//reg5����8�䶯��Ƶ��
//reg9����12qmcs���Ƶ��
void reg5_12_SMCS_QMSC_frequency_handle(uint32_t SMCS_freq,uint32_t QMSC_freq)
{
	//�ı��䶯��Ƶ��
}


//reg13����16,NL ,NH��ѹ
void reg13_16_NL_NM_voltage_handle(uint16_t NL,uint16_t NM)
{

}
//����Ƶ��
void reg17_18_JG_frequency_handle(uint16_t JG_freq)
{
	
}

//������壬�������
void reg19_22_CHG_START_pulse_handle(uint16_t CHG_pulse,uint16_t START_pulse)
{
	
}

//������壬�������
void reg23_TEC_temprature_set_handle(uint8_t temprature)
{
		//CAN_APP_SEND_DATA(CAN_MCU2_DUPLEX_TEC,(uint8_t *)&temprature,CAN_MCU2_STATUS_ID);	
}
void Parse_hmi_handle(uint8_t *data, size_t length) 
{
//	laser_config_param.relay_control_info=data[0]*256+data[1];
	reg0_1_relay_handle(data[0]*256+data[1]);

	laser_config_param.laser_type_flag=data[2];
	reg2_lowVoltage_handle(laser_config_param.laser_type_flag);	
	laser_config_param.soft_control_mm=data[3];
	device_status.soft_stauts=0x02;	//?????��?��?��??
//if(laser_config_param.relay_control_info==0x02) device_status.soft_stauts=0x02;	//?????��?��?��??
//if(laser_config_param.relay_control_info==0x00)  device_status.soft_stauts=0x00;//?????��??
//data[4]��ʱû��
	laser_config_param.RDB_freq=(data[5]<<24)|(data[6]<<16)|(data[7]<<8)|(data[8]);
	device_status.H_coolant_status=0x01;//test	
	laser_config_param.QB_freq=(data[9]<<24)|(data[10]<<16)|(data[11]<<8)|(data[12]);
	reg5_12_SMCS_QMSC_frequency_handle(laser_config_param.RDB_freq,laser_config_param.QB_freq);	
	laser_config_param.NL_voltate=(data[13]*256+data[14])*3;
	laser_config_param.NM_voltate=(data[15]*256+data[16])*3;
	if(laser_config_param.NL_voltate>4700) laser_config_param.NL_voltate=4700;
	if(laser_config_param.NM_voltate>4700) laser_config_param.NM_voltate=4700;
//reg13_16_NL_NM_voltage_handle(laser_config_param.NL_voltate,laser_config_param.NM_voltate);
	laser_config_param.laser_freq=data[17]*256+data[18];	
	if(laser_config_param.laser_freq>60) laser_config_param.laser_freq=60;
	if(laser_config_param.laser_freq<5)  laser_config_param.laser_freq=5;
//reg17_18_JG_frequency_handle(laser_config_param.laser_freq);
	laser_config_param.CD_pulse=data[19]*256+data[20];
	laser_config_param.CG_pulse=data[21]*256+data[22];//????????	
	reg19_22_CHG_START_pulse_handle(laser_config_param.CD_pulse,laser_config_param.CG_pulse);
	laser_config_param.set_TEC_temprature=data[23];	
	reg23_TEC_temprature_set_handle(laser_config_param.set_TEC_temprature);
	laser_status_handle();			
//	printf("info%d\n",device_status.soft_stauts);
}
// ��Ļ���ݽ�������

uint32_t Parse_Data(uint8_t *data, uint32_t length) 
{	
	Frame_t *pFrame;
	uint32_t offset=0,pack_len,handle_data_len=0;
	//sourch head
//	for(offset=0;offset<length-1;offset++)
	while(offset+1<length)
	{
		if(*(data+offset)==0x55&&*(data+offset+1)==0xAA)
		{						
			break;
		}
		offset++;
	}
	if(offset==(length-1)) return offset;//package err//return ret;
	pack_len = length - offset;		
	if(pack_len<sizeof(Frame_t)) return length;//-1;//len err//return ret;
	pFrame =(Frame_t *)(data+offset);
	offset+=sizeof(Frame_t);
	if(Verify_Checksum(data+offset,sizeof(Frame_t))==false) return offset;	// -1;// 41))//crc err//return ret;		
	if(pFrame->protocol_type!=0x10||pFrame->cmd_type!=0x06) return offset;	//-1;//cmd err//return ret;
	// handle
	handle_data_len=(pFrame->data_lengthH)*256+pFrame->data_lengthL;
	Parse_hmi_handle(pFrame->data, handle_data_len);	//data handle				
	return offset;
}
/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{			
	if(UartHandle->Instance==hlpuart1.Instance)
	{		
		lwrb_len=lwrb_write(&rs232_lwrb, RS232_rx_data, 1);			
		if(HAL_UART_Receive_IT(&hlpuart1, RS232_rx_data,1)!=HAL_OK)
		{
			/*Transfer error in reception process */
			Error_Handler();	
			DEBUG_PRINTF("LPUART1 HAL_UART_Receive_IT error\r\n");		
		}			
	}		
}		

/**
  * @brief hmiTask
  * @param  UartHandle: hmiTask
  * @note   HMI MANAGE
  * @retval None
  */
void hmiTask(void const *argument)
{
	uint8_t  flag_rgb,last_len,len,temp[8]={0};
	uint8_t rs232_rx_buff[MAX_LWRB_UART_QUEN];
	uint32_t  rb_offset = 0,i;
	TickType_t hmi_heart_time=0; 
	uint32_t ret_len=0;		
	lwrb_init(&rs232_lwrb, lwrb_rx_buff, MAX_LWRB_UART_QUEN);
	HAL_UART_Receive_IT(&hlpuart1, RS232_rx_data,1);
	for(;;)//20ms
	{			
		len=lwrb_get_linear_block_read_length(&rs232_lwrb);
		if(len&&last_len&&len==last_len)
		{
			printf("len=%d",len);	
			rb_offset+=lwrb_read(&rs232_lwrb,rs232_rx_buff+rb_offset,MAX_LWRB_UART_QUEN-rb_offset);			
			ret_len=Parse_Data(rs232_rx_buff, rb_offset);
			if(ret_len>0)		
			{	
				for(i = 0; i < rb_offset-ret_len;i++)
				{
					rs232_rx_buff[i] = rs232_rx_buff[ret_len+i];
				}						
			}	
			rb_offset -= ret_len;
			last_len=0;			
		}
		else
		{
			//printf("len %d last_len %d\r\n",len,last_len);
			last_len=len;			
		}		
		hmi_heart_time+=20;
		if(hmi_heart_time>1000)
		{
			hmi_heart_time=0;
			temp[0]=3;
			temp[1]=5;
			HAL_GPIO_TogglePin(GPIOC, READY_SYS_LED4_Pin);						
			CAN_APP_SEND_DATA(CAN_POWER_READ_INFO,temp,CAN_IDENTIFIER_POWER_ID);
			//send_data_to_TFT((DEVICE_STATUS *)&device_status,41);	
		}		
		osDelay(20);
	}
}
void laserTask(void const *argument)
{
	uint8_t laser_sta = 0;
	for(;;)
	{				
		osDelay(1);
	}
}
/*�ض���pritf();*/
/* support printf function, usemicrolib is unnecessary */
#if (__ARMCC_VERSION > 6000000)
  __asm (".global __use_no_semihosting\n\t");
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
  FILE __stdout;
#else
 #ifdef __CC_ARM
  #pragma import(__use_no_semihosting)
  struct __FILE
  {
    int handle;
  };
  FILE __stdout;
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
 #endif
#endif

#if defined (__GNUC__) && !defined (__clang__)
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
  * @brief  retargets the c library printf function to the usart.
  * @param  none
  * @retval none
  */
PUTCHAR_PROTOTYPE
{
	if (HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF) != HAL_OK)
  {
    Error_Handler();
  }
	return ch;
}

#if defined (__GNUC__) && !defined (__clang__)
int _write(int fd, char *pbuffer, int size)
{
  for(int i = 0; i < size; i ++)
  {
    __io_putchar(*pbuffer++);
  }

  return size;
}
#endif
