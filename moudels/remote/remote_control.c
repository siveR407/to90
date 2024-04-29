#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 25 // 遥控器接收的buffer大小

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位
  
// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;
static  uint16_t ch[16];
/**  
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) =  0.6*(*(&rc_ctrl[TEMP].rc.rocker_l_ + i))+0.4*(*(&rc_ctrl[LAST].rc.rocker_l_ + i));
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_rx_buf 接收buffer
 */

static void sbus_to_rc(const uint8_t *sbus_rx_buf)
{   
    ch[0] =((sbus_rx_buf[2]<<8)   + (sbus_rx_buf[1])) & 0x07ff;
    ch[1] =((sbus_rx_buf[3]<<5)   + (sbus_rx_buf[2]>>3)) & 0x07ff;
    ch[2] =((sbus_rx_buf[5]<<10)  + (sbus_rx_buf[4]<<2) + (sbus_rx_buf[3]>>6)) & 0x07ff;
    ch[3] =((sbus_rx_buf[6]<<7)   + (sbus_rx_buf[5]>>1)) & 0x07ff;
    ch[4] =((sbus_rx_buf[7]<<4)   + (sbus_rx_buf[6]>>4)) & 0x07ff;
    ch[5] =((sbus_rx_buf[9]<<9)   + (sbus_rx_buf[8]<<1) + (sbus_rx_buf[7]>>7)) & 0x07ff;
    ch[6] =((sbus_rx_buf[10]<<6)  + (sbus_rx_buf[9]>>2)) & 0x07ff;
    ch[7] =((sbus_rx_buf[11]<<3)  + (sbus_rx_buf[10]>>5)) & 0x07ff;
    ch[8] =((sbus_rx_buf[13]<<8)  + (sbus_rx_buf[12])) & 0x07ff;
    ch[9] =((sbus_rx_buf[14]<<5)  + (sbus_rx_buf[13]>>3)) & 0x07ff;
    ch[10]=((sbus_rx_buf[16]<<10) + (sbus_rx_buf[15]<<2) + (sbus_rx_buf[14]>>6)) & 0x07ff;
    ch[11]=((sbus_rx_buf[17]<<7)  + (sbus_rx_buf[16]>>1)) & 0x07ff;
    ch[12]=((sbus_rx_buf[18]<<4)  + (sbus_rx_buf[17]>>4)) & 0x07ff;
    ch[13]=((sbus_rx_buf[20]<<9)  + (sbus_rx_buf[19]<<1) + (sbus_rx_buf[18]>>7)) & 0x07ff;
    ch[14]=((sbus_rx_buf[21]<<6)  + (sbus_rx_buf[20]>>2)) & 0x07ff;
    ch[15]=((sbus_rx_buf[22]<<3)  + (sbus_rx_buf[21]>>5)) & 0x07ff;
     
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ch[1] - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = ch[3]  - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = ch[0] - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = ch[2] - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.SE = ch[8] - RC_CH_VALUE_OFFSET;     
    rc_ctrl[TEMP].rc.SF = ch[9] ;                               // 应急开关
    rc_ctrl[TEMP].rc.SB = ch[5]  ;                                 // 应急开关
    rc_ctrl[TEMP].rc.SC = ch[6] ;
    rc_ctrl[TEMP].rc.SA = ch[4]  ;     
    rc_ctrl[TEMP].rc.SD = ch[7]  ;     
    rc_ctrl[TEMP].rc.SG = ch[10]  ;   
    rc_ctrl[TEMP].rc.SH = ch[11]  ;                          // 应急开关
    // RectifyRCjoystick();
    // // 开关,0左1右
    // rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    // rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // // 鼠标解析
    // rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    // rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    // rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    // rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    // //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    // *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    // if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
    //     rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    // else
    //     memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    // if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
    //     rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    // else
    //     memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    // uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
    //     key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
    //     key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
    //     key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
    //     key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
    //     key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    // for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    // {
    //     if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
    //         continue;
    //     // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
    //     if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
    //         rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
    //     // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
    //     if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
    //         rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
    //     // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
    //     if ((key_with_shift & j) && !(key_last_with_shift & j))
    //         rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    // }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
   {   
    DaemonReload(rc_daemon_instance);// 先喂狗
    if((rc_usart_instance->recv_buff[0]=0x0F) && (rc_usart_instance->recv_buff[24]=0x00 )){
        rc_usart_instance->flag=1;
    }else{
         rc_usart_instance->flag=0;
    }//----校验位
    if( rc_usart_instance->flag=1){      
    sbus_to_rc(rc_usart_instance->recv_buff); 
    
    if((rc_usart_instance->recv_buff[21] != 0x00)){ rc_usart_instance->connect_flag = 0;}
			else {rc_usart_instance->connect_flag = 1;
			    
                 }
     rc_usart_instance->flag = 0;                   
        
    } // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);
     
    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);
    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}