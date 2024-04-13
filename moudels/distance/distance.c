#include "distance.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define distance_size 18

// 传感器数据
static Distance_data dt_da[2];      
static uint8_t dt_init_flag = 0; 

static USARTInstance *dt_usart_instance;
static DaemonInstance *dt_daemon_instance;


 static void data_to_distance(const uint8_t *dt_data)
 {
    dt_da[TEMP].l1=0;
    dt_da[TEMP].l2=0;
    memcpy(&dt_da[LAST],&dt_da[TEMP],sizeof(Distance_data));
 }