#ifndef __hsm_iii_bsp_H
#define __hsm_iii_bsp_H	

#include "main.h"
#include <stdbool.h>

//��Ļ��������
//data[0]=INFO_H  ���ֽ�;data[1] =INFO_L �����ֽڣ����ģʽ
//RELAY_INFO_BIT //bit0~10��11λ��Ч��

#define RELAY_ON   1 //
#define RELAY_OFF  0
#define RELEY_STATUS_INFO_MASK    0x07FF; //��11λ��Ч


/*��ŷ�/�̵�������*/
#define HV_POWER_ON				HAL_GPIO_WritePin(Solenoid_Coil1_EN_GPIO_Port, Solenoid_Coil1_EN_Pin, GPIO_PIN_SET)
#define HV_POWER_OFF			HAL_GPIO_WritePin(Solenoid_Coil1_EN_GPIO_Port, Solenoid_Coil1_EN_Pin, GPIO_PIN_RESET)
#define HV_POWER_Status		HAL_GPIO_ReadPin(Solenoid_Coil1_Status_GPIO_Port, Solenoid_Coil1_Status_Pin)	
			
#define HV_AIR_IN_ON			HAL_GPIO_WritePin(Solenoid_Coil2_EN_GPIO_Port, Solenoid_Coil2_EN_Pin, GPIO_PIN_SET)
#define HV_AIR_IN_OFF			HAL_GPIO_WritePin(Solenoid_Coil2_EN_GPIO_Port, Solenoid_Coil2_EN_Pin, GPIO_PIN_RESET)
#define HV_AIR_IN_Status		HAL_GPIO_ReadPin(Solenoid_Coil2_Status_GPIO_Port, Solenoid_Coil2_Status_Pin)				 
			
#define Solenoid_Coil3_ON			HAL_GPIO_WritePin(Solenoid_Coil3_EN_GPIO_Port, Solenoid_Coil3_EN_Pin, GPIO_PIN_SET)
#define Solenoid_Coil3_OFF		HAL_GPIO_WritePin(Solenoid_Coil3_EN_GPIO_Port, Solenoid_Coil3_EN_Pin, GPIO_PIN_RESET)
#define Solenoid_Coil3_Status		HAL_GPIO_ReadPin(Solenoid_Coil3_Status_GPIO_Port, Solenoid_Coil3_Status_Pin)	
			
#define HV_AIR_OUT_ON			HAL_GPIO_WritePin(Solenoid_Coil4_EN_GPIO_Port, Solenoid_Coil4_EN_Pin, GPIO_PIN_SET)
#define HV_AIR_OUT_OFF		HAL_GPIO_WritePin(Solenoid_Coil4_EN_GPIO_Port, Solenoid_Coil4_EN_Pin, GPIO_PIN_RESET)
#define HV_AIR_OUT_Status		HAL_GPIO_ReadPin(Solenoid_Coil4_Status_GPIO_Port, Solenoid_Coil4_Status_Pin)	
//����������
#define CC4V2_LASER980_ON			HAL_GPIO_WritePin(CC_4V2_EN_GPIO_Port , CC_4V2_EN_Pin, GPIO_PIN_SET)
#define CC4V2_LASER980_OFF		HAL_GPIO_WritePin(CC_4V2_EN_GPIO_Port, CC_4V2_EN_Pin, GPIO_PIN_RESET)
///*���ÿ���*/
//#define air_pump_ON 	HAL_GPIO_WritePin(Compressed_air_pump_GPIO_Port, Compressed_air_pump_Pin, GPIO_PIN_SET) 
//#define air_pump_OFF 	HAL_GPIO_WritePin(Compressed_air_pump_GPIO_Port, Compressed_air_pump_Pin, GPIO_PIN_RESET) 
///*���ÿ��ع����ź�0*/
//#define air_pump_status	 HAL_GPIO_ReadPin(Compressed_air_pump_status_GPIO_Port, Compressed_air_pump_status_Pin)

////ˮѭ���ÿ���
//#define Liquid_cooled_pump_ON				HAL_GPIO_WritePin(Liquid_cooled_pump_GPIO_Port, Liquid_cooled_pump_Pin, GPIO_PIN_SET)
//#define Liquid_cooled_pump_OFF				HAL_GPIO_WritePin(Liquid_cooled_pump_GPIO_Port, Liquid_cooled_pump_Pin, GPIO_PIN_RESET)

////ˮѭ����������
//#define WATER_CYCLE_status		HAL_GPIO_ReadPin(WATER_CYCLE_OK_GPIO_Port, WATER_CYCLE_OK_Pin)
////ADS1118׼�������ź�
//#define ADS1118_DRDY_status		HAL_GPIO_ReadPin(ADS1118_DRDY_GPIO_Port, ADS1118_DRDY_Pin)

//#define BATCH_DATA_LEN 3 //ADC-DMA���䵽Ŀ���ַ�����ݳ���
//#define BATCH_DATA_LEN2 2 //ADC-DMA���䵽Ŀ���ַ�����ݳ���
////�䶯����ͣ
//#define TMC_2226_ON			HAL_GPIO_WritePin(TMC_2226_EN_GPIO_Port, TMC_2226_EN_Pin, GPIO_PIN_RESET);//�������
//#define TMC_2226_OFF		HAL_GPIO_WritePin(TMC_2226_EN_GPIO_Port, TMC_2226_EN_Pin, GPIO_PIN_SET);//ֹͣ���
////�䶯����ת����
//#define TMC_2226_water_outlet		HAL_GPIO_WritePin(TMC_2226_DIR_GPIO_Port, TMC_2226_DIR_Pin, GPIO_PIN_SET)
//#define TMC_2226_water_intlet		HAL_GPIO_WritePin(TMC_2226_DIR_GPIO_Port, TMC_2226_DIR_Pin, GPIO_PIN_RESET)

//#define TMC_2226_index_out		HAL_GPIO_ReadPin(TMC_2226_INDEX_OUT_GPIO_Port, TMC_2226_INDEX_OUT_Pin)

/*��̤����*/
#define FOOT_SWITH_status		HAL_GPIO_ReadPin(FOOT_SWITH_GPIO_Port, FOOT_SWITH_Pin)

/*WN8��Դ����*/
#define	Xenon_capacity_charge_OFF		HAL_GPIO_WritePin(Xenon_capacity_charge_GPIO_Port, Xenon_capacity_charge_Pin, GPIO_PIN_SET);
#define	Xenon_capacity_charge_ON 	HAL_GPIO_WritePin(Xenon_capacity_charge_GPIO_Port, Xenon_capacity_charge_Pin, GPIO_PIN_RESET);

#define	Xenon_capacity_discharge_ON		HAL_GPIO_WritePin(Xenon_capacity_discharge_GPIO_Port, Xenon_capacity_discharge_Pin, GPIO_PIN_SET);
#define	Xenon_capacity_discharge_OFF	HAL_GPIO_WritePin(Xenon_capacity_discharge_GPIO_Port, Xenon_capacity_discharge_Pin, GPIO_PIN_RESET);

#define	Xenon_ignition_control_ON		HAL_GPIO_WritePin(Xenon_ignition_control_GPIO_Port, Xenon_ignition_control_Pin, GPIO_PIN_SET);
#define	Xenon_ignition_control_OFF	HAL_GPIO_WritePin(Xenon_ignition_control_GPIO_Port, Xenon_ignition_control_Pin, GPIO_PIN_RESET);

#define	Xenon_ignition_LEVEL_state	HAL_GPIO_ReadPin(Xenon_ignition_state_GPIO_Port, Xenon_ignition_state_Pin);

	
/*���崮����Ļ����֡�ṹ��*/
typedef struct __attribute__((packed)) {
    uint8_t headerH ;       // ��ʼ��־���ֽ� (0x55)
    uint8_t headerL;       // ��ʼ��־���ֽ� (0xAA)
    uint8_t protocol_type; // Э������
    uint8_t cmd_type;      // ָ������
    uint8_t SEQH;          // �������ݸ��ֽ�
    uint8_t SEQL;          // �������ݵ��ֽ�
    uint8_t data_lengthH;  // ���ݳ��ȸ��ֽ�
    uint8_t data_lengthL;  // ���ݳ��ȵ��ֽ�
    uint8_t data[32];      // ��������	 
    uint8_t checksum;      // У���
} Frame_t;
	
typedef struct local_spram{ //?????��??????
	char temprature;//?��??,-20~100????��?can
	char temprature_TEC;//-20~100??????????-??��?can
	uint8_t IO1;//?��??
	uint8_t IO2;//?��??
	uint8_t laser_protect1;//?��??��??��???����?????
	uint8_t laser_protect2;
	uint8_t laser_protect3;
	uint8_t laser_protect4;
	uint8_t H_coolant_status;//����Ƶ��ȴҺ
	uint8_t L_coolant_status;//����Ƶ��ȴҺ
	uint8_t JT_status;
	uint8_t pro_work_status;//Ԥȼ״̬
	uint8_t depth_coolant_water;//Һλ
	uint32_t pulse_count;//��������
	uint16_t relay_stauts;//�̵���״̬
	uint8_t laser_standby_status;//low_voltage_stauts;//������ѹ�ź�
	uint8_t soft_stauts;//��������ź�
}DEVICE_STATUS;	//??????��?????��???
typedef struct tft_param{ 
	uint16_t relay_control_info;//?????��??????????data0-1
	uint8_t laser_type_flag;//data2/1064?��?????��???? >=0x10;980???��????0x04<0x10??0x00???????��??
	//uint8_t soft_stauts;//?��??????data4???��????
	uint8_t soft_control_mm;//data3?��??????0x00??��??��????1?????��?��?��????????��???
	uint32_t  RDB_freq;//????��?????,???��??data5-8,0~100KHz,???��PWM?��??
	uint32_t  QB_freq;//??��???????data9-12??0~100KHz,
	uint16_t 	NL_voltate;//NL????????????????data13-14??0~5000mV
	uint16_t 	NM_voltate;//NM????????????????data15-16 ??0~5000mV
	uint16_t 	laser_freq;//?��????????data17-18  ??0~60HZ?????????��??50~60HZ((20~16.6Ms)=200~166*100us)
	uint16_t 	CD_pulse;//??????????data19-20??0??????0xFFFF????
	uint16_t 	CG_pulse;//?????????��????start pulse	data21-22
	char 	set_TEC_temprature;//?��?��????????????data21-23	,-10~50
}LOCAL_LASER_CONFIG_PARARM;	//��???????????,????????,��????��??????????

uint8_t Calculate_Checksum(Frame_t *userframe);
uint8_t Calculate_Checksum_Rx(uint8_t *data, size_t length);
bool Check_Header(uint8_t *data);
bool Verify_Checksum(uint8_t *data, size_t length);

extern void INIT_Fill_Frame(void);
void Fill_Data(Frame_t *myframe, uint8_t *data);
void send_data_to_TFT(uint8_t *data,uint8_t len);
uint32_t  Parse_Data(uint8_t *data, uint32_t length);
extern void HMI_manage_task(uint32_t systemTime);



#endif /* __hsm_iii_bsp_H */