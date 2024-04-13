#include "distance.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include "stdio.h"
    
#define distance_size 18

// 传感器数据
static Distance_data dt_da[2];      
static uint8_t dt_init_flag = 0; 

static USARTInstance *dt_usart_instance;
static DaemonInstance *dt_daemon_instance;


 static void data_to_distance(const uint8_t*dt_data)
 {      

        // static  char part1[5], part2[5], part3[5]; 
        // strncpy(part1, dt_data, 4);    // 复制前4位
        // part1[4] = '\0';           // 确保字符串正确结束
        // strncpy(part2, dt_data + 4, 4); // 复制中间4位
        //  part2[4] = '\0';           // 确保字符串正确结束
        //  strncpy(part3, dt_data + 8, 4); // 复制最后4位
        //  part3[4] = '\0';           // 确保字符串正确结束
        sscanf(dt_data,"%4d%4d%4d",&dt_da[TEMP].l1,&dt_da[TEMP].l2,&dt_da[TEMP].r1);  //将数据转换为整型
        // dt_da[TEMP].l1=atoi(part1) ;
        // dt_da[TEMP].l2=atoi(part2) ;
        // dt_da[TEMP].r1=atoi(part3);
    // dt_da[TEMP].l1=dt_data[0];
    // dt_da[TEMP].l2=dt_data[1];
    memcpy(&dt_da[LAST],&dt_da[TEMP],sizeof(Distance_data));
 }

 static void dtRxCallback()
{
    DaemonReload(dt_daemon_instance) ;         // 先喂狗
    // if((dt_usart_instance->recv_buff[0]=0x0F) && (dt_usart_instance->recv_buff[10]=0x00 )){
    //     dt_usart_instance->flag=1;
    // }else{
    //      dt_usart_instance->flag=0;
    // }//----校验位
    //  if( dt_usart_instance->flag=1){      
    data_to_distance(dt_usart_instance->recv_buff); // 进行协议解析
    // sbus_to_rc(rc_usart_instance->recv_buff); 
    
    // if((dt_usart_instance->recv_buff[10] != 0x00)){ dt_usart_instance->connect_flag = 0;}
	// 		else {dt_usart_instance->connect_flag = 1;
			    
    //              }
    //  dt_usart_instance->flag = 0;    
    //  }
}

 static void dtLostCallback(void *id)
{
    memset(dt_da, 0, sizeof(dt_da)); // 清空遥控器数据
    USARTServiceInit(dt_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}


 Distance_data *dtInit(UART_HandleTypeDef *dt_usart_handle){
    USART_Init_Config_s conf;
    conf.module_callback = dtRxCallback;
    conf.usart_handle = dt_usart_handle;
    conf.recv_buff_size = 25;
    dt_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = dtLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    dt_daemon_instance = DaemonRegister(&daemon_conf);

    dt_init_flag = 1;
    return dt_da;
 }
 