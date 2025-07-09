/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-06-16     卢域                       主函数创建线程
 */

#include <rtthread.h>
#include <rtdevice.h>
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "motor.h"
#include "NRF24L01.h"
#include "arm.h"
#include "usart.h"
#include "gpio.h"

rt_mq_t nrf_data_queue;
rt_mq_t nrf_arm_queue;

int main(void)
{
    //消息队列
    nrf_data_queue = rt_mq_create("nrf_queue", sizeof(uint8_t)*32, 10, RT_IPC_FLAG_FIFO);
    //发送提醒机械臂任务执行
    nrf_arm_queue = rt_mq_create("nrf_arm_queue", sizeof(uint8_t)*1, 10, RT_IPC_FLAG_FIFO);

    //电机控制线程
    rt_thread_t control_thread = rt_thread_create("motor_ctrl", motor_control_thread, RT_NULL, 1024, 2, 10);
    if (control_thread != RT_NULL)
    {
         rt_thread_startup(control_thread);
    }

    //nrf通信线程
    rt_thread_t nrf_thread = rt_thread_create("nrf_com", nrf_com_thread, RT_NULL, 1024, 1, 10);
    if(nrf_thread != RT_NULL)
    {
        rt_thread_startup(nrf_thread);
    }

    //机械臂控制线程
    rt_thread_t arm_thread = rt_thread_create("arm_ctrl", arm_control_thread, RT_NULL, 1024, 3, 10);
    if(arm_thread != RT_NULL)
    {
        rt_thread_startup(arm_thread);
    }

    return RT_EOK;
}
