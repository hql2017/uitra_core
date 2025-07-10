#include "soft_uart.h"
#include "main.h"
#include "tim.h"
#include "common_function.h"

unsigned char soft_uart_tx_buf[SOFT_UART_TX_MAX_BYTE];

unsigned char soft_uart_rx_buf[SOFT_UART_RX_MAX_BYTE];

unsigned char soft_uart_tx_num;

unsigned char soft_send_phase;

unsigned char soft_rx_state_code;
#define SOFT_UART_IO_PIN_TX    auxiliary_bulb_pwm_Pin
#define SOFT_UART_IO_PORT_TX  auxiliary_bulb_pwm_GPIO_Port

//设置用于模拟的IO口
#define SOFT_UART_TX_H    HAL_GPIO_WritePin(SOFT_UART_IO_PORT_TX, SOFT_UART_IO_PIN_TX, GPIO_PIN_SET)
#define SOFT_UART_TX_L    HAL_GPIO_WritePin(SOFT_UART_IO_PORT_TX, SOFT_UART_IO_PIN_TX, GPIO_PIN_RESET)

#define SOFT_UART_RX    HAL_GPIO_ReadPin(SOFT_UART_IO_PORT_TX,H_AIR_ERSOFT_UART_IO_PIN_TXROR_Pin)
  
	/* TIM16 init function */

void Soft_Uart_Time(unsigned char isr_time_us)
{
//  htim16.Instance = TIM16;
//  htim16.Init.Prescaler = 269;
//  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim16.Init.Period = isr_time_us-1;
//  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim16.Init.RepetitionCounter = 0;
//  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
//  {
//    Error_Handler();
//  }	
}

/***************************
函数名：Soft_Uart_Time_Init
功能  ：定时器初始化
***************************/
void Soft_Uart_Time_Init(void)
{
   Soft_Uart_Time(SOFT_UART_TIME_ISR);	
}

/***************************
函数名：Soft_Uart_Init
功能  ：UART初始化
***************************/
void Soft_Uart_Init(void)
{
	//TX设置为推挽输出，RX设置为输入上拉,只保留TX
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = SOFT_UART_IO_PIN_TX;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SOFT_UART_IO_PORT_TX, &GPIO_InitStruct);				 
	Soft_Uart_Time_Init();
}

/***************************
函数名：Soft_Uart_Tx_Nbyte
功能  ：UART发送N字节
str   ：字符串
n_byte：需要发送的字节数
返回值：状态码，检查发送的字节数
                                是否超过最大设定
***************************/
unsigned char Soft_Uart_Tx_Nbyte(unsigned char *str,unsigned char n_byte)
{
		unsigned char i;
		if(soft_uart_tx_num != 0)
		{
			return SOFT_TX_BUSY;
		}
		if(n_byte > SOFT_UART_TX_MAX_BYTE)
		{
				return SOFT_TX_DATA_COPY_ERR;
		}
		for(i = 0;i < n_byte;i++)
						soft_uart_tx_buf[i] = *str++;
		
		soft_uart_tx_num  = n_byte;
		soft_send_phase         = SOFT_TX_PHASE_START_SINGLE;
		Soft_Uart_Drive();
	//	HAL_TIM_Base_Start_IT(&htim16);//启动传输
		return SOFT_TX_DATA_COPY_OK;
}

/***************************
函数名：Soft_Uart_Drive
功能  ：发送、接收，置于
                                定时器中断内
***************************/
void Soft_Uart_Drive(void)
{
		static unsigned char tx_bit_num = 0,tx_byte_num = 0,time_count = 0,tx_tem = 0;
		static unsigned char rx_bit_num = 0,rx_byte_num = 0,rx_time_count = 0,rx_tem = 0,rx_phase = 0,rx_xd = 0;
		++time_count;
		if(time_count >= SET_BIT_TIME_COUNT)
		{//1bit时间
			time_count = 0;
			//TX
			if(soft_uart_tx_num)
			{                        
				if(soft_send_phase == SOFT_TX_PHASE_SEND)
				{//开始发送数据
					if(tx_tem&0x01)
					{
						SOFT_UART_TX_H;
					}
					else
					{
						SOFT_UART_TX_L;
					}
					tx_tem >>= 1;
					if(++tx_bit_num >= 8)
					{
						tx_bit_num = 0;
						tx_byte_num++;
						soft_send_phase = SOFT_TX_PHASE_STOP_SINGLE;
						delay_us(SET_BIT_TIME_COUNT);
					}
					
				}
				else if(soft_send_phase == SOFT_TX_PHASE_START_SINGLE)
				{//发送起始码
					soft_send_phase = SOFT_TX_PHASE_SEND;
					SOFT_UART_TX_L;
					tx_tem = soft_uart_tx_buf[tx_byte_num];
					delay_us(SOFT_UART_TIME_ISR);
				}
				else if(soft_send_phase == SOFT_TX_PHASE_STOP_SINGLE)
				{//发送停止码
					SOFT_UART_TX_H;//停止码
					delay_us(SET_1_2BIT_TIME_COUNT);
					soft_send_phase = SOFT_TX_PHASE_START_SINGLE;
					if(tx_byte_num >= soft_uart_tx_num)
					{
						soft_send_phase = SOFT_TX_PHASE_FINISH;
					}
				}
				else if(soft_send_phase == SOFT_TX_PHASE_FINISH)
				{//发送一阵数据完成
					soft_send_phase = SOFT_TX_PHASE_WAIT_WORK;
					soft_uart_tx_num = 0;
					tx_byte_num = 0;
					//关闭定时器
					   //	HAL_TIM_Base_Stop_IT(&htim16);
				}
			}
		}   
/*		
		//RX
		if(rx_phase == SOFT_RX_PHASE_DECODING)
		{
			if(SOFT_UART_RX)
							rx_xd++;
			++rx_time_count;
			if(rx_time_count == SET_BIT_TIME_COUNT)
			{
				rx_time_count = 0;
				rx_tem >>= 1;
				rx_tem  &= 0x7F;//若是右移后最高位自动补0，则不用这一行
				if(rx_xd >= SET_1_2BIT_TIME_COUNT)
				{
								rx_tem |= 0x80;
				}
				rx_xd = 0;
				if(++rx_bit_num >= 8)
				{//收满1byte
								rx_bit_num = 0;
								rx_phase++;
				}
			}
		}
		else if(rx_phase == SOFT_RX_PHASE_WAIT_START)
		{
				if(SOFT_UART_RX == 0)
				{
						rx_time_count++;
						if(rx_time_count >= (SET_1_2BIT_TIME_COUNT+1))
						{
										rx_time_count = 0;
										rx_phase++;
						}
				}
				else
				{
						rx_time_count = 0;
				}
		}
		else if(rx_phase == SOFT_RX_PHASE_RECV_STOP)
		{
				if(SOFT_UART_RX)
				{
						rx_time_count++;
				}
				else
				{
						if(rx_time_count >= SET_1_2BIT_TIME_COUNT)
						{//若是存在干扰，可能接收不到完整的停止信号，判断停止信号若是超过1/2也算通过
										rx_time_count = SET_BIT_TIME_COUNT;
						}
						else
						{
								rx_time_count = 0;
						}
				}
				if(rx_time_count >= SET_BIT_TIME_COUNT)
				{//成功收到结束码1bit
						rx_time_count = 0;
						rx_phase = SOFT_RX_PHASE_WAIT_START;
						soft_uart_rx_buf[rx_byte_num] = rx_tem;
						if(++rx_byte_num >= SOFT_UART_RX_MAX_BYTE)
						{
										rx_byte_num = 0;
										soft_rx_state_code = SOFT_RX_RECV_OK;//接收完成
						}
				}
		}
		*/
}