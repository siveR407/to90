/**
 * @file distance.h
 * @author sive
 * @brief  测距传感器定义头文件
 * @version beta
 * @date 2024-4-11
 *
 */
 
#include <stdint.h>
#include "main.h"
#include "usart.h"

#define LAST 1
#define TEMP 0


typedef struct 
{
    uint16_t l1;
    uint16_t l2;


    /* data */
} Distance_data ;


  