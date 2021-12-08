# ArduinoChipsandBoard

## chips

### [ATmega328P](http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf) /  328

**芯片参数：**

> **Clock:** 16MHz； 
>
> **Flash:** 32KB(其中0.5K / 2KB为Bootloader)；
>
> **SRAM:**2KB；
>
> **EEPROM：**1K；

** 引脚：**

> 20个数字引脚(I/O) ，其中有6个可作为PWM输出引脚，有6个可以作为模拟输入引脚(ADC)，有2个可作为外部中断引脚(INT)；
>
> 1个UART接口，2个I2C接口，1个SPI接口；
>
> 20mA / 40mA的I/O引脚的直流电流；

** 备注：**与计算机通信需要外置的USB to TTL串口芯片(官方UNO / Nano板载ATmega16U2 / FT232RL 作为USB to TTL串行芯片)，且会占用0 1(RX TX)引脚，需要在设计时避开！

ps：与常用的USB to TTL串行芯片:FT232、CP2102、CH340等不同，官方的UNO板是拿ATmega16U2当串口通信芯片，但这块芯片其实本身就是一个MCU，直接就能作为一个小Arduino板的核心，只当一个串口芯片感觉是有点屈才了，于是已经有人在GitHub上给出了方案[HoodLoader2](https://github.com/NicoHood/HoodLoader2)来充分利用上这颗芯片，它的升级版ATmega32U4其实就是Arduino Leonardo和Arduino Micro板的核心芯片，许多的第三方代工厂也是在这里换成了更便宜的CH340来降低成本；而官方的Nano板用的USB串口芯片居然是最贵(当然也是最好)的FT232RL，要知道光这块芯片T宝价格甚至比这块板的核心ATmega328还要贵。

**官方板：**[Arduino UNO REV3](https://store.arduino.cc/usa/arduino-uno-rev3) / [Arduino Nano](https://store.arduino.cc/usa/arduino-nano)；



----

### [ATmega32U4](http://www.atmel.com/Images/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf)

**芯片参数：**

> **Clock：**16MHz；
>
> **Flash：** 32KB(其中4KB为Bootloader)；
>
> **SRAM：**2.5KB；
>
> **EEPROM：**1K；

**引脚：**

> 20个数字引脚(I/O) ，其中有7个可作为PWM输出，有12个可作为模拟输入(ADC)，有4个可作为外部中断引脚(INT)；
>
> 1个UART接口，2个I2C接口，1个SPI接口；
>
> 40mA / 20mA的I/O引脚的直流电流；

**备注：**集成USB-to-TTL串口通信功能，无需外置的USB芯片；

**官方板：**[Arduino Leonardo](https://store.arduino.cc/usa/leonardo)，[Arduino Micro](https://store.arduino.cc/usa/arduino-micro)；

ps：ATmega32U4和ATmega328非常相似，但整体来看ATmega32U4要集成度更高功能更多一些，以下是Lenardo与UNO板在设计使用上的一些区别：

1.ATmega32U4集成USB串口通信功能，不需要额外芯片，开发板的电路设计可以更加简单，紧凑；

2.ATmega32U4与计算机的串口通信是虚拟的，其在每次通电或重启时创建，这意味着：

> 32U4与计算机通信(Serial)并不会占用硬串口(引脚0和1，即RX和TX)，这使得Lenardo板的0 1引脚可以像其他引脚一样正常使用；而UNO与计算机通信则会占用硬串口，使得设计使用时需要避开0 1引脚（如果要使用Lenardo的硬串口与其他外设通信，则要使用Serial1）；
>
> 每次重启(Reset)时，Lenardo板都会与计算机短暂地断开通信(重新建立虚拟串口)；而UNO板有独立的USB芯片，可以一直保持与计算机的连接；

3.ATmega32U4可以模拟鼠标与键盘(使用[Mouse.begin()](https://www.arduino.cc/reference/en/language/functions/usb/mouse/)和[Keboard.begin()](https://www.arduino.cc/reference/en/language/functions/usb/keyboard/))，因此可以有更多有意思的玩法(比如模拟鼠标高速连点)，不过使用这两个功能一定要格外小心，毕竟这直接作为了输入设备可以控制你的计算机，可能你操作你的计算机，从而导致无法再次对它进行编程

----

### ATmega2560

**芯片参数：**

> **Clock：**16MHz；
>
> **Flash：** 256KB(其中8KB为Bootloader)；
>
> **SRAM：**8KB；
>
> **EEPROM：**4K；

**引脚：**

> 54个数字引脚(I/O) ，其中有15个可作为PWM输出，
>
> 16个模拟输入(ADC)；
>
> 20mA的I/O引脚的直流电流

**备注：**与计算机通信需要额外的外置的USB芯片(板载ATmega16U2 USB-to-TTL串行芯片)

**官方板：**[Arduino MEGA 2560 REV3](https://store.arduino.cc/usa/mega-2560-r3)

---

### [Atmel SAM3X8E ARM Cortex-M3 CPU](http://www.atmel.com/Images/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf)

**芯片参数：**

> **Clock：**84MHz；
>
> **Flash：** 512KB；
>
> **SRAM：**96KB(two banks: 64KB and 32KB)；

**引脚：**

> 54个数字引脚(I/O) ，其中有12个可作为PWM输出，
>
> 12个模拟输入(ADC)；
>
> 2个模拟输出(DAC)
>
> 130mA的I/O引脚的直流电流

**备注：** 与大多数Arduino开发板不同，Arduino Due开发板的工作电压为3.3V。I/O引脚可以承受的最大电压为3.3V。

板载ATmega16U2 USB-to-TTL串行芯片

**官方板：**[Arduino DUE](https://store.arduino.cc/usa/due)

---
