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
#include "NRF24L01.h"
#include "motor.h"

#define UART3_NAME   "uart3"
#define UART1_NAME   "uart1"
#define RX_MAX_BUF    8

#define MAX_BUFFER_SIZE  24  // 1字节帧头 + 2字节X + 2字节Y + 4字节Size

rt_device_t serial_k230;//串口设备k230

rt_device_t serial_arm;//机械臂串口设备

//机械臂收集数据相关
uint8_t Rx_Data[64] = {0};
uint8_t Rx_index = 0;
uint8_t Rx_Flag = 0;
uint8_t RecvFlag = 0;

//机械臂追踪相关
uint8_t delta_x = 0;//x轴差距
uint8_t delta_y = 0;//y轴差距
uint8_t delta_size = 0;//大小差距

uint8_t arm_ids[6] = {1, 2, 3, 4, 5, 6};//舵机ID号
//开始执行夹去任务
uint16_t servo_value[6] = {2048, 2048, 3000, 3000, 1500, 1200};//记录舵机当前位置
uint16_t servo_init_value[6] = {2048, 2048, 3072, 3072, 1500, 1200};//初始化使用
//画面中心点
int16_t center_x = 160;
int16_t center_y = 120;

//串口2配置
struct serial_configure uart3_set_parg = RT_SERIAL_CONFIG_DEFAULT;
struct serial_configure uart1_set_parg = RT_SERIAL_CONFIG_DEFAULT;
//k230串口接收机状态
static uint8_t rx_buffer[MAX_BUFFER_SIZE];
static uint8_t rx_index = 0;
vision_data_t vision_data = {0};
static rt_bool_t receiving_message = RT_FALSE;

/* 串口中断接收数据 */
rt_err_t rx_irq(rt_device_t dev,rt_size_t size)
{
    uint8_t temp;
    if(rt_device_read(serial_arm, 0, &temp, 1))
    {
        bus_servo_uart_recv(temp);
    }

    return RT_EOK;
}

//K230相关数据接收
rt_err_t uart1_rx_irq(rt_device_t dev, rt_size_t size)
{
    uint8_t temp;//缓存区

    while(rt_device_read(serial_k230, 0, &temp, 1) == 1)
    {
        // 如果是开始字符'#'，重置缓冲区并开始新消息
        if (temp == '#')
        {
            rx_index = 0;
            receiving_message = RT_TRUE;
            rx_buffer[rx_index++] = temp;
        }
        // 如果正在接收消息
        else if (receiving_message)
        {
                    // 存储字符到缓冲区
            if (rx_index < 9)//存8位到缓存区里
            {
                rx_buffer[rx_index++] = temp;
            }

             // 如果收到换行符，表示消息结束
            if (temp == '@')
            {
                rt_device_write(serial_arm, 0, rx_buffer , sizeof(rx_buffer));
                int16_t temp_x = (rx_buffer[1] - '0') * 100 + (rx_buffer[2] - '0') * 10 + rx_buffer[3] - '0';
                int16_t temp_y = (rx_buffer[5] - '0') * 100 + (rx_buffer[6] - '0') * 10 + rx_buffer[7] - '0';

                vision_data.delta_x = center_x - temp_x;
                vision_data.delta_y = center_y - temp_y;
                vision_data.updated = RT_TRUE;

                receiving_message = RT_FALSE;//重置
                rx_index = 0;
            }
        }
    }
    return RT_EOK;
}

void Arm_Init(void)
{
    serial_arm = rt_device_find(UART3_NAME);
    if(serial_arm == RT_NULL)
    {
        rt_kprintf("fail to open\n");
    }
    else {
        rt_device_control(serial_arm, RT_DEVICE_CTRL_CONFIG, (void*)&uart3_set_parg);
        rt_device_open(serial_arm, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
        rt_device_set_rx_indicate(serial_arm, rx_irq);
    }

    serial_k230 = rt_device_find(UART1_NAME);
//    rt_hw_serial_register(&serial_k230,
//                          UART1_NAME,
//                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_RX,
//                          0);
    if(serial_k230 == RT_NULL)
    {
        rt_kprintf("fail to find uart1\n");
    }
    else
    {
        rt_device_control(serial_k230, RT_DEVICE_CTRL_CONFIG, (void*)&uart1_set_parg);
        rt_device_open(serial_k230, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_INT_RX);
        rt_device_set_rx_indicate(serial_k230, uart1_rx_irq);
    }


    for(int i = 0; i < 6; i++)
    {
        bus_servo_control(i + 1, servo_init_value[i], 1500);
        rt_thread_mdelay(1000);
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

        rt_device_write(serial_arm, 0, data, 11);


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
        rt_device_write(serial_arm, 0, data, 8);
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
        rt_device_write(serial_arm, 0, data, 8);
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
//固定动作 text
void arm_fixed_execution(void)
{

    //机械臂固定执行
    //先立起来
    bus_servo_control(2, 2048, 1500);
    rt_thread_mdelay(1000);
    bus_servo_control(3, 2048, 1500);
    rt_thread_mdelay(1000);
    bus_servo_control(4, 2048, 1500);
    rt_thread_mdelay(1000);
    bus_servo_control(6, 3000, 1500);
    rt_thread_mdelay(1000);
    //往下面探
    bus_servo_control(2, 2900, 1500);
    rt_thread_mdelay(1000);
    bus_servo_control(3, 2550, 1500);
    rt_thread_mdelay(1000);
    bus_servo_control(4, 2300, 1500);
    rt_thread_mdelay(1000);
    //开夹子 闭夹子
    bus_servo_control(6, 1200, 3000);
    rt_thread_mdelay(1000);
    bus_servo_control(6, 3000, 1000);
    rt_thread_mdelay(1000);
    bus_servo_control(2, 2048, 1000);

}


void servo_delay(void)
{
    for(int i = 0; i < 3000; i++)
    {
        for(int j = 0; j < 4000; j++);
    }
}

void servo_delay_ms(int ms)
{
    for(int i = 0; i < ms; i++);
}

//机械臂控制线程
void arm_control_thread(void* arg)
{
    uint8_t arm_buf[32] = {0};
    uint8_t flag1 = 0, flag2 = 0;
    extern rt_mq_t nrf_arm_queue;


    Arm_Init();
    rt_thread_mdelay(10);

    while(1)
    {  //收取nrf执行机械臂任务信号
        rt_size_t size = rt_mq_recv(nrf_arm_queue, arm_buf, sizeof(arm_buf), 10);
        //执行机械臂夹取任务
//        rt_device_write(serial_arm, 0, rx_buffer, sizeof(rx_buffer));
        if(size > 0 && arm_buf[3] != 0 && flag1 == 0)//识别到一次就执行
        {
            flag1 = 1;
//            rt_thread_delete(nrf_com_thread);
//            rt_thread_delete(motor_control_thread);
        }

        if(flag1)
        {
            if(flag2 == 0)
            {
                //初始化手臂位置
                for(int i = 5; i >= 0; i--)
                {
                    bus_servo_control(i + 1, servo_value[i], 2000);
                    servo_delay_ms(1000);
                    //rt_thread_mdelay(10);
                }
                flag2++;//退出
                flag1++;
            }
            //校准流程
            if(flag1 == 2 && vision_data.updated)
            {
               if(vision_data.delta_x > 5)
               {
                   servo_value[0]+=3;
                   bus_servo_control(1, servo_value[0], 2000);
                   servo_delay_ms(1500);
               }
               else if(vision_data.delta_x < -5)
               {
                   servo_value[0]-=3;
                   bus_servo_control(1, servo_value[0], 2000);
                   servo_delay_ms(1500);
               }

               //rt_thread_mdelay(100);
               if(vision_data.delta_y > 5)
               {
                   servo_value[3]-=5;
                   bus_servo_control(4, servo_value[3], 2000);
                   servo_delay_ms(1500);
               }
               else if(vision_data.delta_y < -5)
               {
                   servo_value[3]+=5;
                   bus_servo_control(4, servo_value[3], 2000);
                   servo_delay_ms(1500);
               }

               vision_data.updated = RT_FALSE;//已经使用需要重新赋值
               //对准了，开始夹取
               if((vision_data.delta_x >= -5 && vision_data.delta_x <= 5) &&
                   (vision_data.delta_y >= -5 && vision_data.delta_y <= 5))
               {
                   flag1++;
               }
            }
            //探头夹取
            if(flag1 == 3)
            {




                servo_value[3] = 2700;//4号
                bus_servo_control(4, servo_value[3], 2000);
                servo_delay_ms(3000);

                servo_value[2] = 2048;//3号伸直
                bus_servo_control(3, servo_value[2], 2000);
                servo_delay_ms(3000);

                servo_value[1] = 3200;//2号往下探
                bus_servo_control(2, servo_value[1], 1000);
                servo_delay_ms(3000);
                flag1++;
            }
            //举起
            if(flag1 == 4)
            {
                servo_value[0] = 1500;
                bus_servo_control(1, servo_value[0], 2000);
                servo_delay_ms(2000);
                flag1 = 0;//退出任务
            }
        }
        rt_thread_mdelay(10);
    }

}
