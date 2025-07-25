/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-06-13     Zero       the first version
 */
#ifndef APPLICATIONS_NRF24L01_H_
#define APPLICATIONS_NRF24L01_H_
//NRF24L01结构体实例
struct nrf24l01{
    rt_base_t CE;
    rt_base_t CSN;
    rt_base_t SCK;
    rt_base_t MOSI;
    rt_base_t MISO;
    rt_base_t IRQ;
};

//寄存器地址
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

//操作指令
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define NOP           0xFF

//状态
#define RX_OK       0x40
#define TX_OK       0x20
#define MAX_TX      0x10

uint8_t R_IRQ(void);

void W_Reg(uint8_t Reg,uint8_t Value);

uint8_t R_Reg(uint8_t Reg);

void W_Buf(uint8_t Reg , uint8_t* Buf, uint8_t Len);

void R_Buf(uint8_t Reg , uint8_t* Buf, uint8_t Len);

void nrf_receive(uint8_t* Buf);

uint8_t nrf_send(uint8_t* Buf);

void NRF24L01_init(void);

void nrf_com_thread(void* arg);

#endif /* APPLICATIONS_NRF24L01_H_ */
