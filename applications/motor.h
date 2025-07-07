/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-06-11     Zero       the first version
 */
#ifndef APPLICATIONS_MOTOR_H_
#define APPLICATIONS_MOTOR_H_

#define MOTOR_ID_NUM 4

enum motor_id{
    MOTOR_ID_FRONT_LEFT = 0,
    MOTOR_ID_FRONT_RIGHT = 1,
    MOTOR_ID_BEHIDE_LEFT = 2,
    MOTOR_ID_BEHIDE_RIGHT = 3
};

#define DEVICE_NAME_LEN 16

struct car_motor{
    rt_base_t dirPin1;
    rt_base_t dirPin2;
    char deviceName[DEVICE_NAME_LEN];
    int pwmChanel;
    struct rt_device_pwm *pwmDev;
};

int motor_init(void);

int motor_speed(enum motor_id id, int speed);

void move_transform(int vx, int vy, int vz);

void motor_control_thread(void* arg);

#endif /* APPLICATIONS_MOTOR_H_ */
