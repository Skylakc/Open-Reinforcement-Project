# 硬件规划
支持双路三电平控制

## 主板

STM32F103ZET6 主板功能管理（南桥） 
```
管理电源EN
测量各路电压
CPU温度保护
与CPU板通信
CPU板复位信号——经过cpld 2io
ARM板复位信号——经过cpld 2io
SPI屏幕显示基本信息
PT100温度传感器信号检出
与CPLD交流并关闭输出
ID控制
```
CPLD PWM信号保护
STM32通信内部双输入正门启动

### 电压
DVDD 3.3v - STM32 CPLD
DVDD 5v

FPGA电压 ？
TMS电压 ？
ARM电压 ？

### 接口
#### RS232
MAX232
#### CAN
CAN 收发器

#### 以太网
PHY
ARM和FPGA
#### SD卡
- ARM
- FPGA

#### HDMI
- ESD控制
- ARM
- FPGA

#### USB
- CYUSB3014 示波器USB PHY
- FT232 FPGA JTAG
- STM32F103C8T6 STM32 JTAG
- FPGA USB外设
- STM32 USB外设
- ARM USB 外设

## CPU板

- Xilinx ZYNQ XC7z020

- TI TMS28337

SRIO通信
## ARM附件
- Rockchip RK3588

## ADC板
- LVDS接口

## 信号输出卡
- 光纤头输出

### 电机功率卡
- 光耦隔离 
- MR30接口输出 

#### 电流探头卡
- AMC1301隔离放大器 
- BNC接口

### 光纤输出卡
线路驱动器

## 模拟信号输入卡
- 需要解决电流信号远距离传输问题
- 提供运放正负电压
- 提供M2接口插不同放大倍数的运放模块


### 运放模块

## 数字信号输入卡
4路reset信号 4路fault信号预留 进CPLD控制
数字信号要经过6n137光耦隔离
### 旋变解码器模块
AD2S1210
最好能输出ABZ或者直接是SPI
### 磁编码器解码模块
### 差分ABZ输入模块
### 普通ABZ输入模块

## 信号观察卡
DAC们和一堆运放
## 示波器卡












