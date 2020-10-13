
# ST17H66 UART透传AT指令

# `目录`
  [AT 指令概述](#at-指令概述) 
  * [1.1 AT指令介绍](#11-at指令介绍)
  * [1.2 格式说明](#12-格式说明)
  * [1.3 指令命令符](#13-指令命令符)
  * [1.4 指令说明](#14-指令说明)
    * [AT+ERR](#141-aterr)
    * [AT+NAME](#142-atname)
    * [AT+MAC](#143-atmac)
    * [AT+CIVER](#144-atciver)
    * [AT+UART](#145-atuart)
    * [AT+AUTO+++](#146-atauto)
    * [AT+LINK](#147-atlink)
    * [AT+ADVINT](#148-atadvint)
    * [AT+POWER](#149-atpower)
    * [AT+UUID](#1410-atuuid)
    * [AT+DISCONN](#1411-atdisconn)
    * [AT++++](#1412-at)
    * [AT+Z](#1413-atz)
    * [AT+RESET](#1414-atreset)
    * [AT+RESE](#1415-atrese)
    * [AT+FLASH](#1416-atflash)

## AT 指令概述

### 1.1 AT指令介绍  

AT+指令是指在命令模式下用户通过UART与模块进行命令传递的指令集，后面将详细讲解AT+指令的格式。  
上电启动成功后，可以使用AT指令通过UART对模块进行设置。

### 1.2 格式说明
1. <>   :   表示必须包含的部分；
2. []   :   表示可选的部分；
3. AT+  :   命令消息前缀；
4. CMD  :   指令命符；
5. [op] :   指令操作符，表明是查询或是设置参数；
   1. "?"   :表示查询参数，ASCII码 0x3F
   2. "="   :表示设置参数，ASCII码 0x3D
   3. ":"   :表示响应的参数，ASCII码 0x3A
6. [data_n] :设置或查询参数时的数据；
7. <RSP>    :响应字符；
   1. "OK"  :表示成功，ASCII码 0x4F,0x4B
   2. "ERR" :表示失败，ASCII码 0x45,0x52,0x52
8. <{CR}> :结束符，回车，ASCII码 0x0D
9. <{LF}> :结束符，换行，ASCII码 0x0A
    
基础指令：AT+<CMD><{CR}><{LF}>  
查询指令：AT+<CMD>[?]<{CR}><{LF}>  
设置指令：AT+<CMD>[=][data_1,data_2...data_n]<{CR}><{LF}>  
响应消息：AT+<CMD>[:][data_1,data_2...data_n]<{CR}><{LF}><RSP><{CR}><{LF}>

### 1.3 指令命令符
|     `命令`    |    `说明`     |
|:------------|:-----------|
|   NAME    |   模块的名称      |
|   MAC     |   模块的MAC地址   |
|   CIVER   |   模块软件版本号  |
|   UART    |   模块串口波特率  |
|   AUTO++   |  模块连接蓝牙后是否自动进入透传模式|
|   LINK    |   模块蓝牙连接状态|
|   ADVINT   |  模块蓝牙广播间隔|
|   POWER   |   模块发射功率    |
|   UUID    |   模块UUID服务    |
|   DISCONN |   断开当前蓝牙连接|
|   +++     |   模块进入透传模式|
|   Z       |   模块重启        |
|   RESET   |   恢复出厂设置    |
|   FLASH   |   模块记忆参数    |
|   RESE    |   自定义广播数据  |
---

### 1.4 指令说明

#### 1.4.1 `AT+ERR`

所有查询或设置指令出错则响应：AT+ERR{CR}{LF}

#### 1.4.2 `AT+NAME`

功能：查询/设置模块的名称

> 查询模块的名称:AT+NAME?{CR}{LF}  
> 查询响应：AT+NAME:name{CR}{LF}OK{CR}{LF}

> 设置模块的名称：AT+NAME=name{CR}{LF}  
> 设置响应：AT+NAME:name{CR}{LF}OK{CR}{LF}  

> tips:设置后重广播生效

参数name:模块的名称（限制长度1--20字节，ASCII码标志，默认为BLE_UART)

设置举例：将模块名称设置为Lenze_UartBle  
> AT+NAME=Lenze_UartBle{CR}{LF}  

#### 1.4.3 `AT+MAC`

功能：查询/设置模块的MAC地址

> 查询模块的MAC地址：AT+MAC?{CR}{LF}  
> 查询响应：AT+MAC:mac1{CR}{LF}OK{CR}{LF} 

> 设置模块的MAC地址：AT+MAC=mac2{CR}{LF}  
> 设置响应：AT+MAC:mac2{CR}{LF}OK{CR}{LF}  

> tips:设置后掉电记忆，重启生效。

参数mac1:查询模块的MAC地址（限制长度12字节，ASCII码表示）  
参数mac2:设置模块的MAC地址（限制长度12字节，ASCII码表示）

设置举例：将模块MAC地址设置为0xAF 12 34 56 78 90  
> AT+MAC=[0x41 0x46 0x31 0x32 0x33 0x34 0x35 0x36 0x37 0x38 0x39 0x30]{CR}{LF}

#### 1.4.4 `AT+CIVER`

功能：查询软件版本号

>查询模块的软件版本号：AT+CIVER?{CR}{LF}  
>查询响应：AT+CIVER:ver{CR}{LF}OK{CR}{LF}

参数ver:模块软件版本号(ASCII码表示，默认为v1.1.1)

#### 1.4.5 `AT+UART`

功能：查询/设置模块的串口波特率

> 查询模块的串口波特率：AT+UART?{CR}{LF}  
> 查询响应：AT+UART:baudrate{CR}{LF}OK{CR}{LF}  

> 设置模块的串口波特率：AT+UART=baudrate{CR}{LF}  
> 设置响应：AT+UART:baudrate{CR}{LF}OK{CR}{LF}  

> tips:设置后立即生效。

参数baudrate:模块的串口波特率(限制长度4字节，十六进制表示，默认为115200：0x00 01 C2 00)

设置举例：将模块的串口波特率设置为9600  
> AT+UART=[0x00 00 25 80]{CR}{LF}

#### 1.4.6 `AT+AUTO+++`

功能：查询/设置模块蓝牙连接后是否进入透传模式

> 查询模块蓝牙连接后是否进入透传模式：AT+AUTO+++?{CR}{LF}  
> 查询响应：AT+AUTO+++:[Y/N]{CR}{LF}OK{CR}{LF}

> 设置模块蓝牙连接后是否进入透传模式：AT+AUTO+++=[Y/N]{CR}{LF}  
> 设置响应：AT+AUTO+++:[Y/N]{CR}{LF}OK{CR}{LF}

参数[Y/N]:模块蓝牙连接后是否进入透传模式(限制长度1字节，ASCII码表示，Y 蓝牙连接后进入透传模式，N 蓝牙连接后不进入透传模式，默认为Y)

设置举例：模块蓝牙连接后进入透传模式
> AT+AUTO+++=Y{CR}{LF}

#### 1.4.7 `AT+LINK`

功能：查询蓝牙连接状态

> 查询模块的蓝牙连接状态：AT+LINK?{CR}{LF}  
> 查询响应：AT+LINK:[OnLine/OffLine]{CR}{LF}OK{CR}{LF}  

参数：  
      OnLine 蓝牙已连接  
      OffLine 蓝牙未连接

#### 1.4.8 `AT+ADVINT`

功能：查询/设置蓝牙广播包间隔

> 查询蓝牙广播包间隔：AT+ADVINT?{CR}{LF}  
> 查询响应：AT+ADVINT:advint{CR}{LF}OK{CR}{LF} 

> 设置蓝牙广播包间隔：AT+ADVINT=advint{CR}{LF}  
> 设置响应：AT+ADVINT:advint{CR}{LF}OK{CR}{LF} 
 
> tips:设置后重新广播生效。


|  参数((十六进制表示)  |  广播包间隔  |
|:----------|:----------|
|  0x00  |  50ms  |
|  0x01  |  100ms  |
|  0x02(默认)  |  200ms  |
|  0x03  |  500ms  |
|  0x04  |  1000ms  |
|  0x05  |  2000ms  |
---

设置举例：设置蓝牙广播包间隔为200ms
> AT+ADVINT=[0x02]{CR}{LF} 

#### 1.4.9 `AT+POWER`

功能：查询/设置模块的射频功率

> 查询模块的射频功率：AT+POWER?{CR}{LF}  
> 查询响应：AT+POWER:power{CR}{LF}OK{CR}{LF} 

> 设置模块的射频功率：AT+POWER=power{CR}{LF}  
> 设置响应：AT+POWER:power{CR}{LF}OK{CR}{LF}  

> tips:设置后重新广播生效。

|  参数(十六进制表示)  |  射频功率(dB)  |
|:----------|:----------|
|  0x00  |  10  |
|  0x01  |  5  |
|  0x02  |  4  |
|  0x03  |  3  |
|  0x04(默认)  |  0  |
|  0x05  |  -2  |
|  0x06  |  -5  |
|  0x07  |  -10  |
---
设置举例：设置模块的射频功率为0dB
> AT+POWER=[0x04]{CR}{LF}  

#### 1.4.10 `AT+UUID`

功能：查询模块透传服务

> 查询服务UUID:AT+UUID?{CR}{LF}  
> 查询响应：
> AT+bleUart_Server_Uuid:[server_uuid]{CR}{LF}OK{CR}{LF}  
> AT+bleUart_Server_Tx_Uuid:[server_tx_uuid]{CR}{LF}OK{CR}{LF}  
> AT+bleUart_Server_Rx_Uuid:[server_rx_uuid]{CR}{LF}OK{CR}{LF}  

|  参数  |  说明  |  默认值(十六进制)  |
|:------|:------|:-------|
|  server_uuid |  主服务UUID  |  0x55 E4 05 D2 AF 9F A9 8F E5 4A 7D FE 43 53 53 49  |
|  server_tx_uuid |  串口读服务UUID  |  0x16 96 24 47 C6 23 61 BA D9 4B 4D 1E 43 53 53 49  |
|  server_rx_uuid |  串口写服务UUID  |  0xB3 9B 72 34 BE EC D4 A8 F4 43 41 88 43 53 53 49  |
---

#### 1.4.11 `AT+DISCONN`

功能：断开当前蓝牙连接

> 设置断开当前蓝牙连接：AT+DISCONN{CR}{LF}  
> 设置响应：AT+DISCONN{CR}{LF}OK{CR}{LF}  

#### 1.4.12 `AT++++`

功能：设置模块进入透传模式

> 设置模块进入透传模式：AT++++{CR}{LF}  
> 设置响应：AT++++{CR}{LF}OK{CR}{LF}  

#### 1.4.13 `AT+Z`

功能：重启模块

> 设置重启模块：AT+Z{CR}{LF}  
> 设置响应：AT+Z{CR}{LF}OK{CR}{LF}  

#### 1.4.14 `AT+RESET`

功能：恢复出厂设置

> 设置恢复出厂设置：AT+RESET{CR}{LF}  
> 设置响应：AT+RESET{CR}{LF}OK{CR}{LF}

#### 1.4.15 `AT+RESE`

功能：查询/设置蓝牙自定义广播数据

> 查询蓝牙自定义广播数据：AT+RESE?{CR}{LF}  
> 查询响应：AT+RESE:data{CR}{LF}OK{CR}{LF}  

> 设置蓝牙自定义广播数据：AT+RESE=data{CR}{LF}  
> 设置响应：AT+RESE:data{CR}{LF}OK{CR}{LF}  

> tips:设置后重新广播生效。

参数data:蓝牙自定义广播数据（长度限制8字节，十六进制表示）

设置举例：设置蓝牙自定义广播数据0x 11 22 33 44 55 66 77 88 
> AT+RESE=[0x11 22 33 44 55 66 77 88]{CR}{LF}

#### 1.4.16 `AT+FLASH`

功能：控制模块存储设备名、广播包间隔、连接是否自动进入透传模式、串口波特率、射频功率、自定义广播包数据等信息到FLASH

> 设置存储信息到FLASH:AT+FLASH{CR}{LF}  
> 设置响应：AT+FLASH{CR}{LF}OK{CR}{LF}  

