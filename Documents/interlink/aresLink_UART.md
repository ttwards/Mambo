## ARES2025上下位机通信

### 一、执行

#### 1. 发送帧

```mermaid
packet-beta
title 执行帧
0-1: "0xCAFE"
2-3: "ID"
4-7: "Arg1"
8-11: "Arg2"
12-15: "Arg3"
16-17: "Request ID"
```

#### 2. 接收帧

```mermaid
packet-beta
title 返回帧
0-1: "0x1A64"
2-3: "Func ID"
4-7: "Return Value"
8-9: "Value"
```

### 二、数据

#### 1. 接收帧

```mermaid
packet-beta
title 数据帧
0-1: "0xDA2A"
2-3: "Data ID"
4-5: "Num"
6-13: "Block 0"
14-21: "Block 1"
22-29: "Block..."
30-31: "CRC-16"
32-33: "0xDEAD"
```

```mermaid
packet-beta
title 数据Block
0: "0x98"
1-2: "Var Code"
3-6: "Var Data"
7: "0x77"
```

**一帧数据中最多包含12个数据**

### 三、错误

#### 1. 错误帧

```mermaid
packet-beta
title 错误帧
0-1: "0x2BAD"
2-3: "Request ID"
4-5: "Error Code"
6-7: "CRC-8"
8-9: "0xDEAD"
```

#### 2. 错误码

```
0xFAC2	CRC效验错误
0xF411	未知帧头
0x8848	未知帧尾
0x6666	未知变量码
0x2024	未知函数码
0x1024	参数数量错误
0xCAFE	发送超时
0x22DD	下位机忙
0x2401	下位机/上位机收到对方的错误码，但数据已经丢弃无法重发
0xBEEF	重要事件，需要对方立刻处理下一帧数据
0x90DE	下位机重启
```

错误帧中的Request ID可以是执行帧的Request ID，也可以是数据帧的Data ID。
上位机最好保存尽可能多的数据，下位机会至少保留256Bytes

#### 3. 错误处理

尽可能重发，如果重发次数超过3次，上位机应该停止发送数据，等待下位机重启。
