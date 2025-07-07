/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-06-11     卢域                   电机驱动相关
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "motor.h"
#include "drv_common.h"
#include <stdlib.h>
#define DBG_TAG "motor"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

double Car_HW = (0.175 + 0.195) / 2; //小车旋转参数比例值 轴距轮距单位m
int targetA, targetB, targetC, targetD;

struct car_motor carMotor[MOTOR_ID_NUM] = {
        {GET_PIN(B, 3), GET_PIN(B, 4), "pwm2", 2, RT_NULL},//左前
        {GET_PIN(F, 7), GET_PIN(F, 6), "pwm2", 3, RT_NULL},//右前
        {GET_PIN(A, 11), GET_PIN(A, 10), "pwm2", 1, RT_NULL},//左后
        {GET_PIN(C, 11), GET_PIN(C, 12), "pwm2", 4, RT_NULL}//右后
};

/* 电机初始化 */
int motor_init(void)
{
    int i;
    for(i = 0; i < MOTOR_ID_NUM; i++){
        carMotor[i].pwmDev = (struct rt_device_pwm*)rt_device_find(carMotor[i].deviceName);
        if(carMotor[i].pwmDev == RT_NULL)
        {
            LOG_E("can not find %s !\n", carMotor[i].deviceName);
            return -1;
        }
        rt_pin_mode(carMotor[i].dirPin1, PIN_MODE_OUTPUT);
        rt_pin_mode(carMotor[i].dirPin2,  PIN_MODE_OUTPUT);
    }
    //启动PWM
    motor_speed(MOTOR_ID_FRONT_LEFT, 0);
    motor_speed(MOTOR_ID_FRONT_RIGHT, 0);
    motor_speed(MOTOR_ID_BEHIDE_LEFT, 0);
    motor_speed(MOTOR_ID_BEHIDE_RIGHT, 0);
    return RT_EOK;
}

/* PWM设置 */
int motor_speed(enum motor_id id, int speed)
{
    rt_uint32_t period = 50000;
    rt_uint32_t pulse = speed;

    if(id >= MOTOR_ID_NUM)
        id = 0;

    if(speed < 0){
        rt_pin_write(carMotor[id].dirPin1, PIN_LOW);
        rt_pin_write(carMotor[id].dirPin2, PIN_HIGH);
        pulse = -pulse;
    }
    else {
        rt_pin_write(carMotor[id].dirPin1, PIN_HIGH);
        rt_pin_write(carMotor[id].dirPin2, PIN_LOW);
    }

    if(carMotor[id].pwmDev){
        rt_pwm_set(carMotor[id].pwmDev, carMotor[id].pwmChanel, period, pulse);
        rt_pwm_enable(carMotor[id].pwmDev, carMotor[id].pwmChanel);
        return RT_EOK;
    }
    else{
        LOG_E("motor must init first");
        return RT_ERROR;
    }

}

/* 速度分解 */
void move_transform(int vx, int vy, int vz)
{
    targetA = vx + vy - vz * Car_HW;
    targetB = vx - vy - vz * Car_HW;
    targetC = vx + vy + vz * Car_HW;
    targetD = vx - vy + vz * Car_HW;

    motor_speed(MOTOR_ID_FRONT_LEFT, targetB);
    motor_speed(MOTOR_ID_FRONT_RIGHT, targetC);
    motor_speed(MOTOR_ID_BEHIDE_LEFT, targetA);
    motor_speed(MOTOR_ID_BEHIDE_RIGHT, targetD);
}

/* 电机控制线程 */
void motor_control_thread(void* arg)
{
    extern rt_mq_t nrf_data_queue;
    uint8_t buf[32] = {0};
    motor_init();
    while(1)
    {
        rt_size_t size = rt_mq_recv(nrf_data_queue, buf, sizeof(buf), 10);
        if(size > 0)
        {
            move_transform(buf[0] * 1000, buf[1] * 1000, buf[2] * 1000);
        }
        else
        {
            move_transform(0, 0, 0);
        }
        rt_thread_mdelay(10);
    }
}
