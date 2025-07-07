/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-07-02     卢域                       舵机机械臂
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "arm.h"
#include "drv_common.h"
#include <stdlib.h>
#include <rtdbg.h>
#include "board.h"
#include "usart.h"
#include "gpio.h"

#define UART_NAME   "uart3"
#define RX_MAX_BUF    8

rt_device_t serial;//串口设备
//收集数据相关
uint8_t Rx_Data[24] = {0};
uint8_t Rx_index = 0;
uint8_t Rx_Flag = 0;
uint8_t RecvFlag = 0;
//串口2配置
struct serial_configure uart3_set_parg = RT_SERIAL_CONFIG_DEFAULT;

/* 串口中断接收数据 */
rt_err_t rx_irq(rt_device_t dev,rt_size_t size)
{
//    rt_size_t len;
    uint8_t res[8];
    rt_device_read(serial, 0, res, size);
//    rt_device_write(serial, 0, &res[0], sizeof(res[0]));
    bus_servo_uart_recv(res[0]);
    return 0;
}

void Arm_Init(void)
{
    serial = rt_device_find(UART_NAME);
    if(serial == RT_NULL)
    {
        rt_kprintf("fail to open\n");
    }
    else {
        rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, (void*)&uart3_set_parg);
        rt_device_open(serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
        rt_device_set_rx_indicate(serial, rx_irq);
    }

}

/* 控制总线舵机，
 * id：要控制的id号，0xfe为全体控制
 * value：位置值（96~4000）
 * time：运行的时间，时间越小，运行越快，最小为0
 * */
void bus_servo_control(uint8_t id, uint16_t value, uint16_t time)
{
    if (value >= 96 && value <= 4000)
    {
        const uint8_t s_id = id & 0xff;
        const uint8_t len = 0x07;
        const uint8_t cmd = 0x03;
        const uint8_t addr = 0x2a;

        const uint8_t pos_H = (value >> 8) & 0xff;
        const uint8_t pos_L = value & 0xff;

        const uint8_t time_H = (time >> 8) & 0xff;
        const uint8_t time_L = time & 0xff;

        const uint8_t checknum = (~(s_id + len + cmd + addr + pos_H + pos_L + time_H + time_L)) & 0xff;

        unsigned char data[11] = {0};
        data[0] = 0xff;
        data[1] = 0xff;
        data[2] = s_id;
        data[3] = len;
        data[4] = cmd;
        data[5] = addr;
        data[6] = pos_H;
        data[7] = pos_L;
        data[8] = time_H;
        data[9] = time_L;
        data[10] = checknum;

        rt_device_write(serial, 0, data, 11);
    }
}

/* 写入目标ID(1~250) */
void bus_servo_set_id(uint8_t id)
{
    if ((id >= 1) && (id <= 250))
    {
        const uint8_t s_id = 0xfe; /* 发送广播的ID */
        const uint8_t len = 0x04;
        const uint8_t cmd = 0x03;
        const uint8_t addr = 0x05;
        const uint8_t set_id = id; /* 实际写入的ID */

        const uint8_t checknum = (~(s_id + len + cmd + addr + set_id)) & 0xff;
        // const uint8_t data[] = {0xff, 0xff, s_id, len, cmd, addr, set_id, checknum};
        unsigned char data[8] = {0};
        data[0] = 0xff;
        data[1] = 0xff;
        data[2] = s_id;
        data[3] = len;
        data[4] = cmd;
        data[5] = addr;
        data[6] = set_id;
        data[7] = checknum;
        rt_device_write(serial, 0, data, 8);
    }
}

/* 发送读取舵机位置命令 */
void bus_servo_read(uint8_t id)
{
    if (id > 0 && id <= 250)
    {
        const uint8_t s_id = id & 0xff;
        const uint8_t len = 0x04;
        const uint8_t cmd = 0x02;
        const uint8_t param_H = 0x38;
        const uint8_t param_L = 0x02;

        const uint8_t checknum = (~(s_id + len + cmd + param_H + param_L)) & 0xff;
        // const uint8_t data[] = {0xff, 0xff, s_id, len, cmd, param_H, param_L, checknum};

        unsigned char data[8] = {0};
        data[0] = 0xff;
        data[1] = 0xff;
        data[2] = s_id;
        data[3] = len;
        data[4] = cmd;
        data[5] = param_H;
        data[6] = param_L;
        data[7] = checknum;
        rt_device_write(serial, 0, data, 8);
    }
}

//转化接收到的值为位置数
uint16_t bus_servo_get_value(void)
{
    uint8_t checknum = (~(Rx_Data[2] + Rx_Data[3] + Rx_Data[4] + Rx_Data[5] + Rx_Data[6])) & 0xff;
    if(checknum == Rx_Data[7])
    {
        // uint8_t s_id = Rx_Data[2];
        uint16_t value_H = 0;
        uint16_t value_L = 0;
        uint16_t value = 0;

        value_H = Rx_Data[5];
        value_L = Rx_Data[6];
        value = (value_H << 8) + value_L;
        return value;
    }
    return 0;
}

uint8_t get_Rx_state(void)
{
    return RecvFlag;
}

//处理串口数据，如果符合协议则设置RecvFlag = 1
void bus_servo_uart_recv(uint8_t Rx_Temp)
{
    switch(Rx_Flag)
    {
        case 0:
            if(Rx_Temp == 0xff)
            {
                Rx_Data[0] = 0xff;
                Rx_Flag = 1;
            }
            else if (Rx_Temp == 0xf5)
            {
                Rx_Data[0] = 0xff;
                Rx_Data[1] = 0xf5;
                Rx_Flag = 2;
                Rx_index = 2;
            }
            break;

        case 1:
            if(Rx_Temp == 0xf5)
            {
                Rx_Data[1] = 0xf5;
                Rx_Flag = 2;
                Rx_index = 2;
            } else
            {
                Rx_Flag = 0;
                Rx_Data[0] = 0x0;
            }
            break;

        case 2:
            Rx_Data[Rx_index] = Rx_Temp;
            Rx_index++;

            if(Rx_index >= RX_MAX_BUF)
            {
                Rx_Flag = 0;
                RecvFlag = 1;
            }
            break;

        default:
            break;
    }
}

void arm_fixed_execution(void)
{

}

void arm_control_thread(void* arg)
{   //控制2 3 4 6舵机
    uint8_t id2 = 2;
    uint8_t id3 = 3;
    uint8_t id4 = 4;
    uint8_t id6 = 6;
    Arm_Init();

    while(1)
    {
        //机械臂固定执行
        //先立起来
        bus_servo_control(id2, 2048, 1000);
        bus_servo_control(id3, 2048, 1000);
        bus_servo_control(id4, 2048, 1000);
        bus_servo_control(id6, 3000, 1000);

        //往下面探


    }
}
