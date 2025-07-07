from machine import SPI
from machine import FPIOA
from machine import Pin

#寄存器地址代码
CONFIG    = 0x00
EN_AA     = 0x01
EN_RXADDR = 0x02
SETUP_AW  = 0x03
SETUP_RETR =0x04
RF_CH     = 0x05
RF_SETUP  = 0x06
STATUS    = 0x07
OBSERVE_TX= 0x08
CD        = 0x09
RX_ADDR_P0= 0x0A
RX_ADDR_P1= 0x0B
RX_ADDR_P2= 0x0C
RX_ADDR_P3= 0x0D
RX_ADDR_P4= 0x0E
RX_ADDR_P5= 0x0F
TX_ADDR   = 0x10
RX_PW_P0  = 0x11
RX_PW_P1  = 0x12
RX_PW_P2  = 0x13
RX_PW_P3  = 0x14
RX_PW_P4  = 0x15
RX_PW_P5  = 0x16
FIFO_STATUS=0x17
DYNPD     = 0x1C
FEATURE   = 0x1D

#操作指令代码
R_REGISTER  = 0x00
W_REGISTER  = 0x20
R_RX_PAYLOAD= 0x61
W_TX_PAYLOAD= 0xA0
FLUSH_TX    = 0xE1
FLUSH_RX    = 0xE2
NOP         = 0xFF

#状态
RX_OK      = 0x40
TX_OK      = 0x20
MAX_TX     = 0x10

T_ADDR = bytes([0xF0, 0xF0, 0xF0, 0xF0, 0xF0])
R_ADDR = bytes([0xF0, 0xF0, 0xF0, 0xF0, 0xF0])
class nrf24l01:

    def __init__(self):
        self.gpio = FPIOA()
        self.gpio.set_function(15, self.gpio.QSPI0_CLK)
        self.gpio.set_function(16, self.gpio.QSPI0_D0)
        self.gpio.set_function(17, self.gpio.QSPI0_D1)
        self.gpio.set_function(18, FPIOA.GPIO18)
        self.gpio.set_function(19, FPIOA.GPIO19)
        self.gpio.set_function(14, FPIOA.GPIO14)
        self.CE = Pin(18, Pin.OUT, pull = Pin.PULL_NONE, drive = 7)
        self.CSN = Pin(14, Pin.OUT, pull = Pin.PULL_NONE, drive = 7)
        self.IRQ = Pin(19, Pin.IN, pull = Pin.PULL_UP)
        self.spi = SPI(1, baudrate = 5000000, polarity = 0, phase = 0, bits = 8)
        self.NRF24L01_Init()
    @staticmethod
    def Delay_us(t_us):
        time.sleep_us(t_us)

    def W_CE(self,value):
        self.CE.value(value)

    def R_IRQ(self):
        return self.IRQ.value()

    def W_CSN(self,value):
        self.CSN.value(value)


    def SPI_SwapByte(self,byte):
        response = bytearray(1)
        self.spi.write_readinto(bytearray([byte]), response)
        return response[0]

    def R_Reg(self,reg):
        self.W_CSN(0)
        self.SPI_SwapByte(reg)
        Value = self.SPI_SwapByte(NOP)
        self.W_CSN(1)
        return Value

    def W_Reg(self,reg,value):
        self.W_CSN(0)
        self.SPI_SwapByte(reg | W_REGISTER)
        self.SPI_SwapByte(value)
        self.W_CSN(1)

    def W_Buf(self,reg,buf,len):
        self.W_CSN(0)
        self.SPI_SwapByte(reg | W_REGISTER)
        self.spi.write(buf[:len])
        self.W_CSN(1)

    def R_Buf(self,reg, len):
        self.W_CSN(0)
        self.SPI_SwapByte(reg)
        buf = bytearray(len)
        self.spi.readinto(buf)
        self.W_CSN(1)
        return buf

    def NRF24L01_Init(self):
        self.W_CE(0)
        self.W_Buf(W_REGISTER + TX_ADDR, T_ADDR, 5)
        self.W_Buf(W_REGISTER + RX_ADDR_P0, R_ADDR, 5)
        self.W_Reg(W_REGISTER + CONFIG, 0x0F)
        self.W_Reg(W_REGISTER + EN_AA, 0x01)
        self.W_Reg(W_REGISTER + RF_CH, 0x00)
        self.W_Reg(W_REGISTER + RX_PW_P0, 32)
        self.W_Reg(W_REGISTER + EN_RXADDR, 0x01)
        self.W_Reg(W_REGISTER + SETUP_RETR, 0x1A)
        self.W_Reg(FLUSH_RX, NOP)
        self.W_CE(1)
        status = self.R_Reg(R_REGISTER + STATUS)
        print("初始状态:", hex(status))
        config = self.R_Reg(R_REGISTER + CONFIG)
        print("配置寄存器:", hex(config))


    def Receive(self,buf):
        Status = self.R_Reg(R_REGISTER + STATUS)
        if Status & RX_OK:
            self.R_Buf(R_RX_PAYLOAD, Buf, 32)
            self.W_Reg(FLUSH_RX, NOP)
            self.W_Reg(W_REGISTER + STATUS, Status)
            self.Delay_us(150)

    def Send(self,buf):
        self.W_Buf(W_TX_PAYLOAD, buf, 32)
        self.W_CE(0)
        self.W_Reg(W_REGISTER + CONFIG, 0x0E)
        self.W_CE(1)
        while self.R_IRQ() == 1:
            pass
        Status = self.R_Reg(R_REGISTER + STATUS)
        print("Status:", hex(Status))
        if Status & MAX_TX:
            self.W_Reg(FLUSH_TX, NOP)
            self.W_Reg(W_REGISTER + STATUS, Status)
            print("fail send")
            return MAX_TX
        if Status & TX_OK:
            self.W_Reg(W_REGISTER + STATUS, Status)
            print("successful send")
            return TX_OK













