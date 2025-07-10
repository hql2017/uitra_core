#ifndef __SOFT_UART_H__
#define __SOFT_UART_H__
/*软件实现UART******************************************
波特率                 ：4800/9600/14400/19200
停止位                 ：1bit
奇偶校验         ：无
注意                         ：定时器的优先级尽量设高！！！
使用前定义一定时器名为Soft_Uart_Time，并将步进设置为1us。
调用Soft_Uart_Init()进行初始化。
调用Soft_Uart_Drive()在提供时基的定时器中断。
********************************************************/

extern void Soft_Uart_Time(unsigned char isr_time_us);

//设置发送区、接收区大小
#define SOFT_UART_TX_MAX_BYTE 15
#define SOFT_UART_RX_MAX_BYTE 15
//设置通信波特率
#define SOFT_UART_BAUD_RATE                BAUD_RATE_9600

//以下部分无需修改==================================================================================================================
#define BAUD_RATE_4800        4800
#define BAUD_RATE_9600        9600
#define BAUD_RATE_14400        14400
#define BAUD_RATE_19200        19200               

#if(SOFT_UART_BAUD_RATE == BAUD_RATE_4800)
                #define SOFT_UART_TIME_ISR                        52
                #define SET_BIT_TIME_COUNT                          4
                #define SET_1_2BIT_TIME_COUNT                 2
#elif(SOFT_UART_BAUD_RATE == BAUD_RATE_9600)
                #define SOFT_UART_TIME_ISR                        26
                #define SET_BIT_TIME_COUNT                          4
                #define SET_1_2BIT_TIME_COUNT                 2
#elif(SOFT_UART_BAUD_RATE == BAUD_RATE_14400)
                #define SOFT_UART_TIME_ISR                        23
                #define SET_BIT_TIME_COUNT                          3
                #define SET_1_2BIT_TIME_COUNT                 2
#elif(SOFT_UART_BAUD_RATE == BAUD_RATE_19200)
                #define SOFT_UART_TIME_ISR                        26
                #define SET_BIT_TIME_COUNT                          2
                #define SET_1_2BIT_TIME_COUNT                 1
#endif

enum SORT_TX_STATE{
        SOFT_TX_DATA_COPY_OK,        //拷贝数据成功
        SOFT_TX_DATA_COPY_ERR,        //拷贝发送的字节数超过最大设置
        SOFT_TX_BUSY,                        //当前有未发送的数据
};

enum SOFT_RX_PHASE{
        SOFT_RX_PHASE_WAIT_START,//等待接收
        SOFT_RX_PHASE_DECODING,         //接收中
        SOFT_RX_PHASE_RECV_STOP, //接收停止信号
};

enum SOFT_TX_PHASE{
        SOFT_TX_PHASE_WAIT_WORK,        //等待发送
        SOFT_TX_PHASE_START_SINGLE,        //发送起始信号
        SOFT_TX_PHASE_SEND,                        //发送数据
        SOFT_TX_PHASE_STOP_SINGLE,        //发送停止信号
        SOFT_TX_PHASE_FINISH,                //发送SOFT_UART_TX_MAX_BYTE字节完成
};

enum SOFT_RX_STATE_CODE{
        SOFT_RX_DECODING,//接收中
        SOFT_RX_RECV_OK         //接收SOFT_UART_RX_MAX_BYTE字节，1帧数据
};


//void Soft_Uart_Io_Init(void);
//void Soft_Uart_Time_Init(void);
void Soft_Uart_Init(void);
unsigned char Soft_Uart_Tx_Nbyte(unsigned char *str,unsigned char n_byte);
void Soft_Uart_Drive(void);

#endif