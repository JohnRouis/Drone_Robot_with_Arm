/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-07-02     卢域                     舵机机械臂相关
 */
#ifndef APPLICATIONS_ARM_H_
#define APPLICATIONS_ARM_H_

struct uart_rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

void Arm_Init(void);

void bus_servo_control(uint8_t id, uint16_t value, uint16_t time);

void bus_servo_set_id(uint8_t id);

void bus_servo_read(uint8_t id);

uint16_t bus_servo_get_value(void);

uint8_t get_Rx_state(void);

void bus_servo_uart_recv(uint8_t Rx_Temp);

void arm_control_thread(void* arg);//机械臂控制线程

#endif /* APPLICATIONS_ARM_H_ */
