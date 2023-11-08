# 这是一篇电子工程开发入门指南

## 资源篇

<[GITHUB]> ~~全球最大同性交友网站~~

<[立创开源平台]> ~~中国小学生聚集地~~

<[菜鸟教程]> ~~打工人的第一本百科全书~~

<[HACKDAY.IO]> ~~不正常人类项目研究中心~~

<[CSDN]> ~~程序猿毒瘤之面向对象教程~~

<www.google.com> 谷歌娘

[GITHUB]: https://github.com
[立创开源平台]: ttps://oshwhub.co
[菜鸟教程]: ttps://www.runoob.co
[HACKDAY.IO]: ttps://hackaday.io
[CSDN]: ttps://www.csdn.net

## 项目篇

硬件的第一个项目都是~~烧掉~~点亮一颗二极管，就像任何代码第一句话都是 `Hello World!`

**任何电路无非是对能量和信号的处理**

**任何电路架构都可以分为前后级，前级指挥，后级出力**

---
### 第一集 做一个单片机最小系统并让一颗LED忽闪忽闪 再搭一个最基本的三极管放大电路（集电极输出）

### 第二集 做个能看的电子时钟 再搭一个逻辑电路 流 水 灯 ~~跑仿真啊跑仿真~~

### 第三集 把单片机的DNA都用起来得到一个缝合怪控制中心（I^2C SPI UART DMA) 再让功率管自X得到一个高频振荡（单管自激）

### 第四集 一个好玩的项目
做好玩项目之前可以研究研究运放的几种拓扑，最常见的就是比例放大 积分器 电压跟随器。加法器减法器微分器 文~~俊凯~~氏桥振荡器这种看看就行，反正也记不住

这里面最重要的是学会推运放的工作状态，就是输入输出方程，最常用的是虚短虚断

 - 一个很好看的码表 https://github.com/FASTSHIFT/X-TRACK

    还有心思可以学学半桥全桥 正激 反激 LLC 等到这会儿基本模电数电都成型了

### 第五集 转个电机玩啊
 - 转个步进电机 我的仓库 
 - 或者这个 非常推荐 https://github.com/creapunk/CLN17
 - 这个也不错 b站金色巧克力蛋糕 https://github.com/unlir/XDrive
  
- 还是别转步进电机了，那个有意思但是意思不大 整点无刷玩玩 难度逐渐上升但是大差不差
  - SimpleFOC https://simplefoc.com/ 国内版搜灯哥开源
  - 本杰明项目 VESC https://vesc-project.com/
  - ODrive https://odriverobotics.com/

一个阶段性的综合项目 https://github.com/SmallPond/X-Knob

---
如果考虑举一反三的可能性，那么简易版电子工程基本到这里就结束了

在之后的还有进阶版电源设计（各种开关电源），射频电路（万恶的RF），高速逻辑电路（PCIe，USB3，LVDS，HDMI这种等等）~~但是我都可以三分钟搞定~~

电子工程多数都是经验主义，灵活使用各类测量仪器和仿真方式是关键。软件组大哥是PSpice（NI Multisim）和Kesight ADS，硬件组大哥是示波器和频谱仪（高阶）



其他的，只能在漫长的时间里慢慢领悟了

本文档会不定期更新

多交流，预祝各位少侠成功

敬上

<p align="right">尚振宇</p>
<p align="right">Eric Shang</p>
<p align="right">2023.11.12</p>

---