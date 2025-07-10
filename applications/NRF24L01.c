/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-06-16     卢域                 软件SPI实现NRF24L01通信
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "NRF24L01.h"
#include "drv_common.h"
#include <stdlib.h>
#include <rtdbg.h>
#include "arm.h"

uint8_t T_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};
uint8_t R_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};

struct nrf24l01 nrf = {GET_PIN(E, 6), GET_PIN(E, 5), GET_PIN(E, 4), GET_PIN(E, 2),
GET_PIN(B, 9), GET_PIN(B, 8)};

void NRF24L01_port_init(void)
{
    rt_pin_mode(nrf.CE, PIN_MODE_OUTPUT);
    rt_pin_mode(nrf.CSN, PIN_MODE_OUTPUT);
    rt_pin_mode(nrf.SCK, PIN_MODE_OUTPUT);
    rt_pin_mode(nrf.MOSI, PIN_MODE_OUTPUT);

    rt_pin_mode(nrf.MISO, PIN_MODE_INPUT);
    rt_pin_mode(nrf.IRQ, PIN_MODE_INPUT);
}

void W_MOSI(uint8_t Value)
{
    rt_pin_write(nrf.MOSI, Value);
}

void W_SCK(uint8_t Value)
{
    rt_pin_write(nrf.SCK, Value);
}

void W_CSN(uint8_t Value)
{
    rt_pin_write(nrf.CSN, Value);
}

void W_CE(uint8_t Value)
{
    rt_pin_write(nrf.CE, Value);
}

uint8_t R_IRQ(void)
{
    return rt_pin_read(nrf.IRQ);
}

uint8_t R_MISO(void)
{
    return rt_pin_read(nrf.MISO);
}

uint8_t SPI_SwapByte(uint8_t Byte)
{
    uint8_t i,ByteReceive=0x00;
    for(i=0;i<8;i++)
    {
        W_MOSI(Byte&(0x80>>i));
        W_SCK(1);
        if(R_MISO()==1)
        {
            ByteReceive=ByteReceive|(0x80>>i);
        }
        W_SCK(0);
    }
    return ByteReceive;
}

void W_Reg(uint8_t Reg,uint8_t Value)
{
    W_CSN(0);//
    SPI_SwapByte(Reg);//
    SPI_SwapByte(Value);//
    W_CSN(1);//
}

uint8_t R_Reg(uint8_t Reg)
{
    uint8_t Value;
    W_CSN(0);//
    SPI_SwapByte(Reg);//
    Value=SPI_SwapByte(NOP);//
    W_CSN(1);//
    return Value;
}

void W_Buf(uint8_t Reg , uint8_t* Buf, uint8_t Len)
{
    uint8_t i;
    W_CSN(0);//
    SPI_SwapByte(Reg);
    for(i=0;i<Len;i++)
    {
        SPI_SwapByte(Buf[i]);
    }
    W_CSN(1);//
}

void R_Buf(uint8_t Reg , uint8_t* Buf, uint8_t Len)
{
    uint8_t i;
    W_CSN(0);//
    SPI_SwapByte(Reg);
    for(i=0;i<Len;i++)
    {
        Buf[i]=SPI_SwapByte(NOP);
    }
    W_CSN(1);//
}


void NRF24L01_init(void)
{
    NRF24L01_port_init();

    W_CE(0);

    W_Buf(W_REGISTER+TX_ADDR, T_ADDR, 5);//
    W_Buf(W_REGISTER+RX_ADDR_P0, R_ADDR, 5);//
    W_Reg(W_REGISTER+CONFIG,0x0F);//
    W_Reg(W_REGISTER+EN_AA,0x01);//
    W_Reg(W_REGISTER+RF_CH,0x00);//
    W_Reg(W_REGISTER+RX_PW_P0,32);//
    W_Reg(W_REGISTER+EN_RXADDR,0x01);//
    W_Reg(W_REGISTER+SETUP_RETR,0x1A);//
    W_Reg(FLUSH_RX,NOP);

    W_CE(1);
}

void nrf_receive(uint8_t* Buf)
{
    uint8_t Status;
    Status =R_Reg(R_REGISTER+STATUS);
    if(Status & RX_OK)
    {
        R_Buf(R_RX_PAYLOAD, Buf, 32);
        W_Reg(FLUSH_RX,NOP);
        W_Reg(W_REGISTER+STATUS, Status);
        HAL_Delay(1);
    }
}

uint8_t nrf_send(uint8_t* Buf)
{
    uint8_t Status;
    W_Buf(W_TX_PAYLOAD, Buf, 32);//

    W_CE(0);
    W_Reg(W_REGISTER+CONFIG,0x0E);
    W_CE(1);

    while(R_IRQ()==1);//
    Status= R_Reg(R_REGISTER+STATUS);

    if(Status & MAX_TX)//
    {
        W_Reg(FLUSH_TX,NOP);//
        W_Reg(W_REGISTER+STATUS,Status);//
        return MAX_TX;
    }
    if(Status & TX_OK)//
    {
        W_Reg(W_REGISTER+STATUS,Status);//
        return TX_OK;
    }
}

void nrf_com_thread(void* arg)
{
    extern rt_mq_t nrf_data_queue;
    extern rt_mq_t nrf_arm_queue;
    uint8_t buf[32]={0};
    NRF24L01_init();
    Arm_Init();

    extern uint16_t servo_value[6];
    extern vision_data_t vision_data;
    uint8_t flag1 = 0, flag2 = 0;
    while(1)
    {
        if(R_IRQ() == 0)
        {
            nrf_receive(buf);
        }
        else
        {
            buf[0] = 0;
            buf[1] = 0;
            buf[2] = 0;
        }
        rt_mq_send(nrf_data_queue, buf, sizeof(buf));

        if(buf[3] != 0 && flag1 == 0)//识别到一次就执行
                {
                    flag1 = 1;
                }

                if(flag1)
                {
                    if(flag2 == 0)
                    {
                        //初始化手臂位置
                        for(int i = 5; i >= 0; i--)
                        {
                            bus_servo_control(i + 1, servo_value[i], 1500);
                            rt_thread_mdelay(1000);
                        }
                        flag2++;//退出
                        flag1++;
                    }
                    //校准流程
                    if(flag1 == 2)
                    {
                       if(vision_data.delta_x > 5)
                       {
                           servo_value[0]+=3;
                           bus_servo_control(1, servo_value[0], 50);

                       }
                       else if(vision_data.delta_x < -5)
                       {
                           servo_value[0]-=3;
                           bus_servo_control(1, servo_value[0], 50);

                       }

                       rt_thread_mdelay(100);
                       if(vision_data.delta_y > 5)
                       {
                           servo_value[3]-=5;
                           bus_servo_control(4, servo_value[3], 50);

                       }
                       else if(vision_data.delta_y < -5)
                       {
                           servo_value[3]+=5;
                           bus_servo_control(4, servo_value[3], 50);

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
                        servo_value[2] = 2048;//3号伸直
                        bus_servo_control(3, servo_value[2], 1500);
                        rt_thread_mdelay(3000);

                        servo_value[3] = 2700;//4号
                        bus_servo_control(4, servo_value[3], 1000);
                        rt_thread_mdelay(3000);

                        servo_value[1] = 3200;//2号往下探
                        bus_servo_control(2, servo_value[1], 1000);
                        rt_thread_mdelay(3000);

                        flag1++;
                    }
                    //举起
                    if(flag1 == 4)
                    {
                        servo_value[1] = 2048;
                        bus_servo_control(2, servo_value[1], 2000);
                        flag1 = 0;//退出任务
                    }
                }


        rt_thread_mdelay(10);
    }
}
