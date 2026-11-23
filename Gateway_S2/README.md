# Gateway_S2：FDCAN1 外部收发（S2 阶段）

STM32H743ZIT6 + FreeRTOS（CMSIS-RTOS2）的 CAN 网关项目第二阶段：把 FDCAN1 从内部回环切到真实总线，完成请求-应答、过滤验收与看门狗。

## 硬件

| 项 | 内容 |
|---|---|
| MCU | STM32H743ZIT6 核心板 |
| CAN 卡 | ZQWL-UCANFD-200U（CAN0，500 kbit/s） |
| 收发器 | 独立 5 V 供电，与板子共地 |
| 终端 | 两端共约 60 Ω |

## 功能

| 模块 | 说明 |
|---|---|
| `S2_CanInit()` | 标准过滤器 `0x123`/掩码 `0x7FF` → Rx FIFO0；未匹配帧全部 reject；启动 FDCAN1 |
| `S2_Transmit()` | 提交 → 等 TXBTO（20 ms）→ 超时取消（20 ms 窗口）→ 挂起保护 |
| `S2_OnFrame()` | 应用层入口：帧合法性判断 + 原样应答 `0x321` |
| `StartCanTask()` | 1 ms 轮询 FIFO0（每轮最多 4 帧）+ 上电启动帧 |
| 默认任务 | 心跳 LED + 栈/堆水位采样（500 ms） |
| IWDG | LSI/32 → 1 kHz，Reload 4000 → 4 s；`task_alive_num` 推进才喂狗 |

## 验收结果

| 用例 | 内容 | 结果 |
|---|---|---|
| T01 | 复位后发出启动帧 `321#5332410000000000` | 通过 |
| T02 | 收到 `123#0102030405060708` 后原样回发 | 通过 |
| T03 | `0x124` 被过滤器拒绝，无回包 | 通过 |
| T04 | **100 Hz × 1000 帧，零丢帧、零超时** | 通过 |
| T05 | 无 ACK 场景：`LEC=3`、超时路径正确 | 通过 |
| T06 | 注入任务停滞：**4.05 s** 后被 IWDG 复位并恢复 | 通过 |

## 资源占用

| 项 | 值 |
|---|---|
| text / bss | 32780 B / 21716 B |
| CanTask 栈 | 2048 B（最小剩余 1712 B） |
| defaultTask 栈 | 2048 B（最小剩余 1880 B） |
| FreeRTOS 堆 | 16384 B（剩余 12056 B） |

## 目录

```
Gateway_S2/
├── Core/                应用与 HAL 配置
│   └── Src/main.c       本阶段全部业务代码（USER CODE 区）
├── Drivers/             STM32H7 HAL 与 CMSIS
├── Middlewares/         FreeRTOS + CMSIS-RTOS2
├── docs/                调试记录与验收报告
└── H743_FreeRTOS_Bringup.ioc
```

## 编译

STM32CubeIDE 1.17.0 导入本目录即可；构建产物（`Debug/`）已在 `.gitignore` 中忽略。

## 说明

- S1 自测代码由 `S1_SELFTEST_ENABLE` 宏整体屏蔽（默认关闭，不参与编译）；
- `S2_REPORT_ENABLE` 默认 0（开启后每秒多发一帧 `0x322` 状态帧，用于调试）；
- `S2_STALL_INJECT` 默认 0（置 1 用于验证看门狗复位，发布前必须为 0）。