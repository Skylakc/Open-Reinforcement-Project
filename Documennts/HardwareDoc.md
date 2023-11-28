# 硬件规划
支持双路三电平控制

## 主板

CPLD  XC2C64A-7VQG100I
STM32F103ZET6 主板功能管理（南桥） 
```
管理电源EN PMIC
主板输入电压电流功耗计算
测量各路电压 *电压FAULT关闭电源 亮Fault并显示器显示
CPU温度保护 *温度保护 关闭电源 亮Fault并显示器显示
与CPU板通信 *串口 温度信号主板电压电流功率回传
CPU板启动复位信号——经过cpld 2io 双输入正门
ARM板启动复位信号——经过cpld 2io 双输入正门
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

Fault记录
- STM32 Fault
  - 温度
  - 电压
- CPLD Fault
  - PWM
  - 
## ARM附件
- Rockchip RK3399

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


# 扭矩传感器
# 磁粉制动器

# 主要硬件型号

## 主板
- 处理器
  - STM32F103ZET6
  - XC2C64A-7VQG100I
- 电源
  - TPS
- PHY
  - USB 

## CPU板
- FPGA
- DSP
- ARM

## 大功率逆变桥信号分配板
LVDS信号内封装电流ADC 温度和母线电压ADC AD7401  

demo
输入电压10-60
输入电流80a
3.3v20a容量 （测试pw22和zxd10）
5v8a容量
tlv62130tps51200 四路电源和ddr
rtl8211 网络phy 两个
ft2232 jtag
max232 rs232架构
隔离can？
hdmi phy？
连接器
差分编码器收
编码器输入光隔离
预留旋变位置
编码器信号输入mux
cpld12线pwm保护
ad7124-8温度 相电压 母线电压
ad7124
stm32功能
四路电压保护 四路输出都留测试点
母线电压保护（母线电压电阻网络后留测试点接两路电压跟随器 一个进cpu，一个进南桥留测试点）（母线电压小功率板用pmos，大功率用交流接触器）
温度保护（读7124）
过流硬保护用比较器运放，由stm32输出pwm后通过二阶滤波器
（后期这些功能要求都用fpga实现，保护逻辑不参与软件）
LTC6820可以进行spi隔离通信，最长100米@0.5Mbit，使用CAT5电缆传输 *意义不大.jpg



第二代
隔离运放
隔离电源
as2s1210
ad7401 （母线电压 三个相电流）
zynq
大功率方案中，高端采样电阻是解决方案，但是容易导致噪声，且相电流超过100A后，检流电阻功率到达5W
换用霍尔元件测量相电流
RK3399 ARM CPU



