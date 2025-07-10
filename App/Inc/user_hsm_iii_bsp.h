#ifndef __hsm_iii_bsp_H
#define __hsm_iii_bsp_H	

#include "main.h"
#include <stdbool.h>

//屏幕参数解析
//data[0]=INFO_H  高字节;data[1] =INFO_L ，低字节；大端模式
//RELAY_INFO_BIT //bit0~10共11位有效。

#define RELAY_ON   1 //
#define RELAY_OFF  0
#define RELEY_STATUS_INFO_MASK    0x07FF; //低11位有效


/*电磁阀/继电器开关*/
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
//激光器开关
#define CC4V2_LASER980_ON			HAL_GPIO_WritePin(CC_4V2_EN_GPIO_Port , CC_4V2_EN_Pin, GPIO_PIN_SET)
#define CC4V2_LASER980_OFF		HAL_GPIO_WritePin(CC_4V2_EN_GPIO_Port, CC_4V2_EN_Pin, GPIO_PIN_RESET)
///*气泵开关*/
//#define air_pump_ON 	HAL_GPIO_WritePin(Compressed_air_pump_GPIO_Port, Compressed_air_pump_Pin, GPIO_PIN_SET) 
//#define air_pump_OFF 	HAL_GPIO_WritePin(Compressed_air_pump_GPIO_Port, Compressed_air_pump_Pin, GPIO_PIN_RESET) 
///*气泵开关过流信号0*/
//#define air_pump_status	 HAL_GPIO_ReadPin(Compressed_air_pump_status_GPIO_Port, Compressed_air_pump_status_Pin)

////水循环泵开关
//#define Liquid_cooled_pump_ON				HAL_GPIO_WritePin(Liquid_cooled_pump_GPIO_Port, Liquid_cooled_pump_Pin, GPIO_PIN_SET)
//#define Liquid_cooled_pump_OFF				HAL_GPIO_WritePin(Liquid_cooled_pump_GPIO_Port, Liquid_cooled_pump_Pin, GPIO_PIN_RESET)

////水循环流量开关
//#define WATER_CYCLE_status		HAL_GPIO_ReadPin(WATER_CYCLE_OK_GPIO_Port, WATER_CYCLE_OK_Pin)
////ADS1118准备就绪信号
//#define ADS1118_DRDY_status		HAL_GPIO_ReadPin(ADS1118_DRDY_GPIO_Port, ADS1118_DRDY_Pin)

//#define BATCH_DATA_LEN 3 //ADC-DMA传输到目标地址的数据长度
//#define BATCH_DATA_LEN2 2 //ADC-DMA传输到目标地址的数据长度
////蠕动泵启停
//#define TMC_2226_ON			HAL_GPIO_WritePin(TMC_2226_EN_GPIO_Port, TMC_2226_EN_Pin, GPIO_PIN_RESET);//启动电机
//#define TMC_2226_OFF		HAL_GPIO_WritePin(TMC_2226_EN_GPIO_Port, TMC_2226_EN_Pin, GPIO_PIN_SET);//停止电机
////蠕动泵旋转方向
//#define TMC_2226_water_outlet		HAL_GPIO_WritePin(TMC_2226_DIR_GPIO_Port, TMC_2226_DIR_Pin, GPIO_PIN_SET)
//#define TMC_2226_water_intlet		HAL_GPIO_WritePin(TMC_2226_DIR_GPIO_Port, TMC_2226_DIR_Pin, GPIO_PIN_RESET)

//#define TMC_2226_index_out		HAL_GPIO_ReadPin(TMC_2226_INDEX_OUT_GPIO_Port, TMC_2226_INDEX_OUT_Pin)

/*脚踏开关*/
#define FOOT_SWITH_status		HAL_GPIO_ReadPin(FOOT_SWITH_GPIO_Port, FOOT_SWITH_Pin)

/*WN8电源控制*/
#define	Xenon_capacity_charge_OFF		HAL_GPIO_WritePin(Xenon_capacity_charge_GPIO_Port, Xenon_capacity_charge_Pin, GPIO_PIN_SET);
#define	Xenon_capacity_charge_ON 	HAL_GPIO_WritePin(Xenon_capacity_charge_GPIO_Port, Xenon_capacity_charge_Pin, GPIO_PIN_RESET);

#define	Xenon_capacity_discharge_ON		HAL_GPIO_WritePin(Xenon_capacity_discharge_GPIO_Port, Xenon_capacity_discharge_Pin, GPIO_PIN_SET);
#define	Xenon_capacity_discharge_OFF	HAL_GPIO_WritePin(Xenon_capacity_discharge_GPIO_Port, Xenon_capacity_discharge_Pin, GPIO_PIN_RESET);

#define	Xenon_ignition_control_ON		HAL_GPIO_WritePin(Xenon_ignition_control_GPIO_Port, Xenon_ignition_control_Pin, GPIO_PIN_SET);
#define	Xenon_ignition_control_OFF	HAL_GPIO_WritePin(Xenon_ignition_control_GPIO_Port, Xenon_ignition_control_Pin, GPIO_PIN_RESET);

#define	Xenon_ignition_LEVEL_state	HAL_GPIO_ReadPin(Xenon_ignition_state_GPIO_Port, Xenon_ignition_state_Pin);

	
/*定义串口屏幕数据帧结构体*/
typedef struct __attribute__((packed)) {
    uint8_t headerH ;       // 起始标志高字节 (0x55)
    uint8_t headerL;       // 起始标志低字节 (0xAA)
    uint8_t protocol_type; // 协议类型
    uint8_t cmd_type;      // 指令类型
    uint8_t SEQH;          // 保留数据高字节
    uint8_t SEQL;          // 保留数据低字节
    uint8_t data_lengthH;  // 数据长度高字节
    uint8_t data_lengthL;  // 数据长度低字节
    uint8_t data[32];      // 数据内容	 
    uint8_t checksum;      // 校验和
} Frame_t;
	
typedef struct local_spram{ //?????è??????
	char temprature;//?·??,-20~100????×?can
	char temprature_TEC;//-20~100??????????-??×?can
	uint8_t IO1;//?¤??
	uint8_t IO2;//?¤??
	uint8_t laser_protect1;//?¤??±??¤???·×?????
	uint8_t laser_protect2;
	uint8_t laser_protect3;
	uint8_t laser_protect4;
	uint8_t H_coolant_status;//高重频冷却液
	uint8_t L_coolant_status;//低重频冷却液
	uint8_t JT_status;
	uint8_t pro_work_status;//预燃状态
	uint8_t depth_coolant_water;//液位
	uint32_t pulse_count;//脉冲数量
	uint16_t relay_stauts;//继电器状态
	uint8_t laser_standby_status;//low_voltage_stauts;//反馈低压信号
	uint8_t soft_stauts;//反馈软件信号
}DEVICE_STATUS;	//??????×?????·???
typedef struct tft_param{ 
	uint16_t relay_control_info;//?????÷??????????data0-1
	uint8_t laser_type_flag;//data2/1064?¤?????ú???? >=0x10;980???ú????0x04<0x10??0x00???????¤??
	//uint8_t soft_stauts;//?í??????data4???é????
	uint8_t soft_control_mm;//data3?í??????0x00??±??í????1?????é?¤?¨????????×???
	uint32_t  RDB_freq;//????±?????,???à??data5-8,0~100KHz,???úPWM?÷??
	uint32_t  QB_freq;//??±???????data9-12??0~100KHz,
	uint16_t 	NL_voltate;//NL????????????????data13-14??0~5000mV
	uint16_t 	NM_voltate;//NM????????????????data15-16 ??0~5000mV
	uint16_t 	laser_freq;//?¤????????data17-18  ??0~60HZ?????????é??50~60HZ((20~16.6Ms)=200~166*100us)
	uint16_t 	CD_pulse;//??????????data19-20??0??????0xFFFF????
	uint16_t 	CG_pulse;//?????????í????start pulse	data21-22
	char 	set_TEC_temprature;//?è?¨????????????data21-23	,-10~50
}LOCAL_LASER_CONFIG_PARARM;	//·???????????,????????,±????¤??????????

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