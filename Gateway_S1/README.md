# Gateway_S1 · H743 FreeRTOS 单路 FDCAN 基线

本目录保存 S0 + S1 阶段的完整工程（STM32CubeIDE 工程，可直接打开编译）。

## 内容

| 阶段 | 内容 |
|---|---|
| S0 | FreeRTOS 心跳基线；`g_dbg_idcode` / `g_rev_id` / `g_heartbeat` 三个调试观测量 |
| S1 | FDCAN1 内部回环自测（T01 接收 / T02 过滤拒绝 / T03 连续接收），结果写入 `g_s1_*` |
| 代码规范 | 应用层按 MISRA C:2012 定向整改（用户代码区），偏离项在注释中标注 |

## 重要说明：`.ioc` 当前是 S2-A 配置

工程在 2026-11-15 按 S2 任务单第 1.5 节改过 CubeMX，因此本目录的 `.ioc` 已经是：

- `FDCAN1.Mode = FDCAN_MODE_NORMAL`
- `FDCAN1.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64`（4 个元素 × 64 字节）

**若要复现 S1 的内部回环验收**，需在 CubeMX 中改回：

- Mode → `Internal Loopback`
- Rx FIFO0 Elmt Size → `8 bytes`

改回后重新生成、编译、烧录，才能复现 S1 的 T01–T03 结果。

## 编译环境

- STM32CubeIDE 1.17.0
- STM32Cube FW_H7 V1.12.1
- GNU Tools for STM32 12.3.rel1
- FreeRTOS CMSIS-RTOS2，tick 1 kHz

## 时钟与位时序

| 项 | 值 |
|---|---|
| SYSCLK | HSI 64 MHz |
| FDCAN 内核时钟 | HSE 25 MHz |
| 名义位时序 | Prescaler 1 / TSEG1 39 / TSEG2 10 / SJW 1 |
| 位率 | 500 kbit/s，采样点 80% |

## 目录

```
Gateway_S1/
├─ Core/            用户代码与 CubeMX 生成代码
│  ├─ Inc/          main.h、FreeRTOSConfig.h、stm32h7xx_hal_conf.h
│  └─ Src/          main.c（S0/S1 业务代码）、msp、it、时基
├─ Drivers/         STM32H7 HAL 与 CMSIS
├─ Middlewares/     FreeRTOS
├─ H743_FreeRTOS_Bringup.ioc
├─ .cproject / .project / .mxproject
└─ STM32H743ZITX_FLASH.ld
```

## 已知未验证项

- 实物 `g_rev_id` 与 ES0392 勘误的对照结论（S3 前必须完成）
- HSE 25 MHz 的物理频率验证（S2-A 必须完成）
