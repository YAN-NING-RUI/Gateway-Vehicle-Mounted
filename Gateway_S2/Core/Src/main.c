/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "portable.h"   /* xPortGetFreeHeapSize 的原型 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* S2 诊断快照：只放“有人读”的字段；调试器展开 g_s2 即可查看全部 */
typedef struct
{
  uint32_t task_created;   /* 1=CanTask 已进入入口（证明创建成功） */
  uint32_t init_step;      /* 1=配过滤 2=配全局 3=启动 4=全部成功 */
  uint32_t init_ok;        /* 1=过滤+启动全部成功，可以收发 */
  uint32_t tx_submitted_num;   /* 提交成功次数 */
  uint32_t tx_confirmed_num;   /* TXBTO 确认完成次数 */
  uint32_t tx_submit_fail_num; /* 提交失败次数 */
  uint32_t tx_timeout_num;     /* 等待完成超时次数 */
  uint32_t tx_cancel_num;      /* 超时后发起取消的次数 */
  uint32_t tx_slot_mask;       /* 最近一次提交占用的槽位掩码 */
  uint32_t tx_stuck;           /* 1=取消后仍挂起，已停止后续提交 */
  uint32_t rx_read_num;        /* 从 FIFO0 成功取出的帧数 */
  uint32_t rx_valid_num;       /* 通过合法性检查的帧数（在 S2_OnFrame 里累加） */
  uint32_t rx_bad_frame_num;   /* 帧头不符或非目标帧的次数 */
  uint32_t rx_last_id;     /* 最近一帧的标准 ID */
  uint32_t rx_last_dlc;    /* 最近一帧的 DLC 码 */
  uint32_t rx_data0;       /* 最近一帧前 4 字节（小端打包） */
  uint32_t rx_data1;       /* 最近一帧后 4 字节 */
  uint32_t stack_can_min;  /* CanTask 历史最小剩余栈（字节） */
  uint32_t stack_led_min;  /* defaultTask 历史最小剩余栈（字节） */
  uint32_t heap_free;      /* FreeRTOS 堆当前剩余（字节） */
  uint32_t task_alive_num;   /* 任务进度计数，S2-D 看门狗的唯一输入 */
} s2_diag_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* S1 自测总开关：定义即启用（上电跑内部回环自测）；S2 起默认不定义，S1 的参数与代码整段不参与编译 */
/* #define S1_SELFTEST_ENABLE */

#ifdef S1_SELFTEST_ENABLE
/* ---- S1 FDCAN1 内部回环自测参数（只在本文件使用） ---- */
#define S1_ID_ACCEPT        0x123U   /* 过滤器放行、应当收到的标准 ID */
#define S1_ID_REJECT        0x124U   /* 不匹配过滤器，应被全局过滤拒绝 */
#define S1_DATA_BYTES       8U       /* Classic 数据帧字节数 */
#define S1_RX_BUF_BYTES     64U      /* 接收临时缓冲按最大 FD 帧给足 */
#define S1_BUDGET_TICKS     50U      /* 单用例等待上限，1 kHz 下约 50 ms */
#define S1_T02_IDLE_TICKS   2U       /* T02 发送完成后再观察的空闲 tick */

/* g_s1_result 取值 */
#define S1_RESULT_IDLE      0U
#define S1_RESULT_PASS      1U
#define S1_RESULT_FAIL      2U

/* g_s1_fail 取值，0 表示没有失败 */
#define S1_FAIL_NONE           0U
#define S1_FAIL_CFG_FILTER     1U   /* HAL_FDCAN_ConfigFilter 失败 */
#define S1_FAIL_CFG_GLOBAL     2U   /* HAL_FDCAN_ConfigGlobalFilter 失败 */
#define S1_FAIL_START          3U   /* HAL_FDCAN_Start 失败 */
#define S1_FAIL_T01_SUBMIT     4U   /* T01 提交失败 */
#define S1_FAIL_T01_TXBTO      5U   /* T01 发送未完成（TXBTO 超时） */
#define S1_FAIL_T01_NO_RX      6U   /* T01 FIFO0 一直为空 */
#define S1_FAIL_T01_READ       7U   /* T01 HAL_FDCAN_GetRxMessage 失败 */
#define S1_FAIL_T01_HEADER     8U   /* T01 帧头不符 */
#define S1_FAIL_T01_DATA       9U   /* T01 数据不符 */
#define S1_FAIL_T02_FIFO_DIRTY 10U  /* T02 开始前 FIFO0 不空 */
#define S1_FAIL_T02_SUBMIT     11U  /* T02 提交失败 */
#define S1_FAIL_T02_TXBTO      12U  /* T02 发送未完成 */
#define S1_FAIL_T02_RX         13U  /* T02 竟然收到了 0x124 */
#define S1_FAIL_T03_SUBMIT     14U  /* T03 提交失败 */
#define S1_FAIL_T03_TXBTO      15U  /* T03 发送未完成 */
#define S1_FAIL_T03_NO_RX      16U  /* T03 FIFO0 一直为空 */
#define S1_FAIL_T03_READ       17U  /* T03 HAL_FDCAN_GetRxMessage 失败 */
#define S1_FAIL_T03_HEADER     18U  /* T03 帧头不符 */
#define S1_FAIL_T03_DATA       19U  /* T03 数据不符 */
#endif /* S1_SELFTEST_ENABLE */

/* ---- S2 单路外部 CAN 骨架参数 ---- */
#define S2_ID_REQUEST           0x123U /* 应用接收的标准 ID */
#define S2_ID_REPLY             0x321U /* 启动帧与应答使用的标准 ID */
#define S2_STD_ID_MASK          0x7FFU /* 11 位全 1：精确匹配 */
#define S2_DATA_BYTES           8U     /* Classic 帧载荷字节数 */
#define S2_TASK_PERIOD_TICKS    1U     /* 无帧可处理时的轮询周期 */
#define S2_RX_BUF_BYTES         64U    /* 与 CubeMX 的 Rx FIFO0 元素大小一致 */
#define S2_RX_MAX_PER_ROUND     4U     /* 每轮最多取 4 帧，避免长时间占住 CPU */
#define S2_REPORT_ENABLE        0      /* 1=每秒上报一帧状态帧（调试用），验收保持 0 */
#define S2_ID_STATUS            0x322U /* 状态帧 ID，与 0x321 应答区分 */
#define S2_REPORT_PERIOD_MS     1000U  /* 状态上报周期（1 tick ≈ 1 ms） */
#define S2_TX_TIMEOUT_TICKS     20U    /* 发送完成等待预算，1 kHz 下约 20 ms */
#define S2_TX_CANCEL_TICKS      20U    /* 超时后取消请求的观察窗口 */
#define S2_STALL_INJECT         1      /* 0=CanTask 启动时卡住 5 s，用于验证 IWDG 复位（T06） */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

IWDG_HandleTypeDef hiwdg1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanTask */
osThreadId_t CanTaskHandle;
const osThreadAttr_t CanTask_attributes = {
  .name = "CanTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */
/* S0 调试观测：仅由调试器读取，不参与业务逻辑 */
volatile uint32_t g_dbg_idcode    = 0U;   /* DBGMCU->IDCODE 原值 */
volatile uint32_t g_rev_id        = 0U;   /* 由 IDCODE[31:16] 提取的修订号 */
volatile uint32_t g_heartbeat_num = 0U;   /* defaultTask 心跳计数，仅本任务写入 */

#ifdef S1_SELFTEST_ENABLE
/* S1 自测结果：只由 defaultTask 写，调试器只读；
   关闭自测时连同声明一起编译掉，不再依赖 --gc-sections 回收 */
volatile uint32_t g_s1_step     = 0U;  /* 最后进入的步骤 1..7 */
volatile uint32_t g_s1_result   = 0U;  /* 0=未完成 1=全部通过 2=失败 */
volatile uint32_t g_s1_fail     = 0U;  /* 失败原因码，见 S1_FAIL_* */
volatile uint32_t g_s1_tx_sub   = 0U;  /* 成功提交帧数，期望 3 */
volatile uint32_t g_s1_tx_done  = 0U;  /* TXBTO 确认完成数，期望 3 */
volatile uint32_t g_s1_rx_cnt   = 0U;  /* 成功读取帧数，期望 2 */
volatile uint32_t g_s1_rx_id    = 0U;  /* 最后读到的 ID */
volatile uint32_t g_s1_rx_dlc   = 0U;  /* 最后读到的 DLC 码，期望 8 */
volatile uint32_t g_s1_rx_data0 = 0U;  /* 最后 8 字节，前 4 字节按小端打包 */
volatile uint32_t g_s1_rx_data1 = 0U;  /* 后 4 字节 */
volatile HAL_StatusTypeDef g_s1_hal_ret = HAL_OK;  /* 最后一次 HAL 状态码 */
volatile uint32_t g_s1_fdcan_ec = 0U;  /* 最后一次 hfdcan1.ErrorCode */
#endif /* S1_SELFTEST_ENABLE */

/* S2 诊断快照：CanTask 写；读者 = 调试器 / S2-D 看门狗（task_alive_num）/ 后续 UDS 状态 DID */
volatile s2_diag_t g_s2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_IWDG1_Init(void);
void StartDefaultTask(void *argument);
void StartCanTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 调试冻结：内核被调试器 halt 时 IWDG 停止计数，避免断点停放导致复位；
     只在“内核停机”时生效，任务卡死（内核仍在跑）时看门狗照常复位 */
  DBGMCU->APB4FZ1 |= DBGMCU_APB4FZ1_DBG_IWDG1;

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  g_dbg_idcode = DBGMCU->IDCODE;
  g_rev_id = (g_dbg_idcode & DBGMCU_IDCODE_REV_ID_Msk) >> DBGMCU_IDCODE_REV_ID_Pos;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_IWDG1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CanTask */
  CanTaskHandle = osThreadNew(StartCanTask, NULL, &CanTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 39;
  hfdcan1.Init.NominalTimeSeg2 = 10;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 5;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 4;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 4;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */

  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */

  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg1.Init.Window = 4095;
  hiwdg1.Init.Reload = 4000;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */

  /* USER CODE END IWDG1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef S1_SELFTEST_ENABLE

/* ===========================================================================
 * S1 FDCAN1 内部回环自测（MISRA C:2012 定向整改）
 * 范围：仅本段 USER CODE；CubeMX 生成代码与 ST HAL/CMSIS 按“外部库”偏离处理。
 * 接受偏离：15.5（多出口快速返回）、17.7（osDelay 返回值显式丢弃）。
 * 整改要点：8.7/8.13 链接性与 const；9.1 定义即初始化；10.5 状态码保留枚举
 * 类型不再强转；12.1 补括号；14.4 控制表达式为本质布尔。
 * =========================================================================== */

/**
  * @brief  记录失败现场；调用者随后立即返回，不再执行后续用例。
  * @param  code  S1_FAIL_* 之一
  */
static void S1_Fail(uint32_t code)
{
  g_s1_fail     = code;
  g_s1_result   = S1_RESULT_FAIL;
  g_s1_fdcan_ec = hfdcan1.ErrorCode;   /* 快照，避免被后续 HAL 调用累加污染 */
}

/**
  * @brief  在用例预算内等待本次发送槽位的 TXBTO 完成位。
  * @param  bufMask     HAL_FDCAN_GetLatestTxFifoQRequestBuffer() 返回的位掩码
  * @param  startTick   用例开始时刻，发送与接收共用同一预算
  * @param  budgetTicks 用例总预算，单位 tick
  * @retval true=已确认发送完成，false=掩码无效或超时
  * @note   TXBTO 只读；同一槽再次提交时硬件自动清位，因此必须在提交后
  *         立即取得掩码再等待，不能延后。
  */
static bool S1_WaitTxDone(uint32_t bufMask, uint32_t startTick, uint32_t budgetTicks)
{
  bool finished = false;

  if (0U != bufMask)
  {
    while (0U == (hfdcan1.Instance->TXBTO & bufMask))
    {
      /* tick 为无符号计数，溢出回绕后差值仍然正确 */
      if ((osKernelGetTickCount() - startTick) >= budgetTicks)
      {
        break;
      }
      (void)osDelay(1U);
    }

    if (0U != (hfdcan1.Instance->TXBTO & bufMask))
    {
      finished = true;
    }
  }

  return finished;
}

/**
  * @brief  在用例预算内等待 Rx FIFO0 至少出现一帧。
  * @param  startTick   用例开始时刻
  * @param  budgetTicks 用例总预算，单位 tick
  * @retval true=FIFO0 非空，可安全调用 HAL_FDCAN_GetRxMessage()
  */
static bool S1_WaitRxFrame(uint32_t startTick, uint32_t budgetTicks)
{
  bool received = false;

  while (0U == HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
  {
    if ((osKernelGetTickCount() - startTick) >= budgetTicks)
    {
      break;
    }
    (void)osDelay(1U);
  }

  if (0U != HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
  {
    received = true;
  }

  return received;
}

/**
  * @brief  把一帧标准 Classic 数据帧提交到 Tx FIFO/Queue。
  * @param  id   11 位标准 ID
  * @param  data 指向 8 字节载荷，调用期间只读
  * @retval HAL 返回值；本函数不等待发送完成
  * @note   DataLength 填 DLC 码（8 表示 8 字节），不是移位后的值。
  */
static HAL_StatusTypeDef S1_QueueStdFrame(uint32_t id, const uint8_t *data)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  txHeader.Identifier          = id;
  txHeader.IdType              = FDCAN_STANDARD_ID;
  txHeader.TxFrameType         = FDCAN_DATA_FRAME;
  txHeader.DataLength          = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker       = 0U;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

/**
  * @brief  逐项核对接收帧头。
  * @param  rx 只读的接收帧头
  * @param  id 期望的标准 ID
  * @retval true=ID、帧类型、格式、DLC 全部符合
  */
static bool S1_RxHeaderOk(const FDCAN_RxHeaderTypeDef *rx, uint32_t id)
{
  bool ok = true;

  if ((rx->Identifier  != id) ||
      (rx->IdType      != FDCAN_STANDARD_ID) ||
      (rx->RxFrameType != FDCAN_DATA_FRAME) ||
      (rx->FDFormat    != FDCAN_CLASSIC_CAN) ||
      (rx->DataLength  != FDCAN_DLC_BYTES_8))
  {
    ok = false;
  }

  return ok;
}

/**
  * @brief  逐字节比较固定长度载荷。
  * @note   比较长度由 S1_DATA_BYTES 决定，不使用帧头 DLC；调用前必须已
  *         校验 DLC 等于 8，避免用未经核对的长度做比较。
  */
static bool S1_DataMatch(const uint8_t *lhs, const uint8_t *rhs)
{
  bool match = true;
  uint32_t i;

  for (i = 0U; i < S1_DATA_BYTES; i++)
  {
    if (lhs[i] != rhs[i])
    {
      match = false;
      break;
    }
  }

  return match;
}

/**
  * @brief  把最近一次收到的 8 字节打包成两个 32 位字，便于调试器直接读取。
  * @param  data 指向至少 S1_DATA_BYTES 字节的只读缓冲区
  */
static void S1_SaveRxData(const uint8_t *data)
{
  g_s1_rx_data0 = ((uint32_t)data[0])
                | ((uint32_t)data[1] << 8U)
                | ((uint32_t)data[2] << 16U)
                | ((uint32_t)data[3] << 24U);
  g_s1_rx_data1 = ((uint32_t)data[4])
                | ((uint32_t)data[5] << 8U)
                | ((uint32_t)data[6] << 16U)
                | ((uint32_t)data[7] << 24U);
}

/**
  * @brief  S1：FDCAN1 内部回环自测，单任务、轮询、只执行一次。
  * @note   顺序与任务单 S1-T01..T03 一致；任何一步失败只记录失败码并返回，
  *         由 StartDefaultTask 继续进入 LED/心跳循环。
  */
static void S1_FdcanLoopbackTest(void)
{
  FDCAN_FilterTypeDef   filter     = {0};
  FDCAN_RxHeaderTypeDef rxHeader   = {0};
  uint8_t  txData[S1_DATA_BYTES]   = {0x01U, 0x23U, 0x45U, 0x67U, 0x89U, 0xABU, 0xCDU, 0xEFU};
  uint8_t  rxData[S1_RX_BUF_BYTES] = {0};
  uint32_t bufMask                 = 0U;
  uint32_t caseStart               = 0U;
  HAL_StatusTypeDef status         = HAL_OK;

  g_s1_step     = 0U;
  g_s1_result   = S1_RESULT_IDLE;
  g_s1_fail     = S1_FAIL_NONE;
  g_s1_tx_sub   = 0U;
  g_s1_tx_done  = 0U;
  g_s1_rx_cnt   = 0U;
  g_s1_rx_id    = 0U;
  g_s1_rx_dlc   = 0U;
  g_s1_rx_data0 = 0U;
  g_s1_rx_data1 = 0U;
  g_s1_hal_ret  = HAL_OK;
  g_s1_fdcan_ec = 0U;

  /* ---- 1. 标准 ID 掩码过滤器：只有 S1_ID_ACCEPT 能进 FIFO0 ---- */
  g_s1_step = 1U;

  filter.IdType           = FDCAN_STANDARD_ID;
  filter.FilterIndex      = 0U;
  filter.FilterType       = FDCAN_FILTER_MASK;
  filter.FilterConfig     = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1        = S1_ID_ACCEPT;
  filter.FilterID2        = 0x7FFU;   /* 11 位全 1：精确匹配 */
  filter.RxBufferIndex    = 0U;       /* 配置为进 FIFO0 时被忽略 */
  filter.IsCalibrationMsg = 0U;       /* 同上，显式初始化 */

  status = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  g_s1_hal_ret = status;
  if (HAL_OK != status)
  {
    S1_Fail(S1_FAIL_CFG_FILTER);
    return;
  }

  /* ---- 2. 未匹配的标准帧/扩展帧一律拒绝，两类远程帧也拒绝 ---- */
  g_s1_step = 2U;

  status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,
                                        FDCAN_REJECT_REMOTE);
  g_s1_hal_ret = status;
  if (HAL_OK != status)
  {
    S1_Fail(S1_FAIL_CFG_GLOBAL);
    return;
  }

  /* ---- 3. 启动 FDCAN1（内部回环，无需收发器） ---- */
  g_s1_step = 3U;

  status = HAL_FDCAN_Start(&hfdcan1);
  g_s1_hal_ret = status;
  if (HAL_OK != status)
  {
    S1_Fail(S1_FAIL_START);
    return;
  }

  /* ===================== T01：0x123 应被接收 ===================== */
  g_s1_step = 4U;
  caseStart = osKernelGetTickCount();   /* T01 总预算起点 */

  status = S1_QueueStdFrame(S1_ID_ACCEPT, txData);
  g_s1_hal_ret = status;
  if (HAL_OK != status)
  {
    S1_Fail(S1_FAIL_T01_SUBMIT);
    return;
  }
  g_s1_tx_sub++;

  bufMask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
  if (!S1_WaitTxDone(bufMask, caseStart, S1_BUDGET_TICKS))
  {
    S1_Fail(S1_FAIL_T01_TXBTO);
    return;
  }
  g_s1_tx_done++;

  if (!S1_WaitRxFrame(caseStart, S1_BUDGET_TICKS))
  {
    S1_Fail(S1_FAIL_T01_NO_RX);
    return;
  }

  status = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);
  g_s1_hal_ret = status;
  if (HAL_OK != status)
  {
    S1_Fail(S1_FAIL_T01_READ);
    return;
  }
  g_s1_rx_cnt++;
  g_s1_rx_id  = rxHeader.Identifier;
  g_s1_rx_dlc = rxHeader.DataLength;

  if (!S1_RxHeaderOk(&rxHeader, S1_ID_ACCEPT))
  {
    S1_Fail(S1_FAIL_T01_HEADER);
    return;
  }
  S1_SaveRxData(rxData);
  if (!S1_DataMatch(rxData, txData))
  {
    S1_Fail(S1_FAIL_T01_DATA);
    return;
  }

  /* ============ T02：0x124 应被拒绝（发送必须完成） ============ */
  g_s1_step = 5U;
  caseStart = osKernelGetTickCount();   /* T02 总预算起点 */

  if (0U != HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
  {
    S1_Fail(S1_FAIL_T02_FIFO_DIRTY);
    return;
  }

  status = S1_QueueStdFrame(S1_ID_REJECT, txData);
  g_s1_hal_ret = status;
  if (HAL_OK != status)
  {
    S1_Fail(S1_FAIL_T02_SUBMIT);
    return;
  }
  g_s1_tx_sub++;

  bufMask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
  if (!S1_WaitTxDone(bufMask, caseStart, S1_BUDGET_TICKS))
  {
    S1_Fail(S1_FAIL_T02_TXBTO);
    return;
  }
  g_s1_tx_done++;

  (void)osDelay(S1_T02_IDLE_TICKS);   /* 发送确实完成后，再多看两个 tick */

  if (0U != HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
  {
    S1_Fail(S1_FAIL_T02_RX);
    return;
  }

  /* ============ T03：0x123 再来一帧，末字节改为 0xEE ============ */
  g_s1_step = 6U;
  caseStart = osKernelGetTickCount();   /* T03 总预算起点 */
  txData[7] = 0xEEU;

  status = S1_QueueStdFrame(S1_ID_ACCEPT, txData);
  g_s1_hal_ret = status;
  if (HAL_OK != status)
  {
    S1_Fail(S1_FAIL_T03_SUBMIT);
    return;
  }
  g_s1_tx_sub++;

  bufMask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
  if (!S1_WaitTxDone(bufMask, caseStart, S1_BUDGET_TICKS))
  {
    S1_Fail(S1_FAIL_T03_TXBTO);
    return;
  }
  g_s1_tx_done++;

  if (!S1_WaitRxFrame(caseStart, S1_BUDGET_TICKS))
  {
    S1_Fail(S1_FAIL_T03_NO_RX);
    return;
  }

  status = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);
  g_s1_hal_ret = status;
  if (HAL_OK != status)
  {
    S1_Fail(S1_FAIL_T03_READ);
    return;
  }
  g_s1_rx_cnt++;
  g_s1_rx_id  = rxHeader.Identifier;
  g_s1_rx_dlc = rxHeader.DataLength;

  if (!S1_RxHeaderOk(&rxHeader, S1_ID_ACCEPT))
  {
    S1_Fail(S1_FAIL_T03_HEADER);
    return;
  }
  S1_SaveRxData(rxData);
  if (!S1_DataMatch(rxData, txData))
  {
    S1_Fail(S1_FAIL_T03_DATA);
    return;
  }

  /* ---- 7. 三组用例全部通过 ---- */
  g_s1_step   = 7U;
  g_s1_result = S1_RESULT_PASS;
}

#endif /* S1_SELFTEST_ENABLE */

/* ===========================================================================
 * S2 单路外部 CAN：任务与收发实现
 * ---------------------------------------------------------------------------
 * 任务本身由 CubeMX 管理：CanTaskHandle / CanTask_attributes / osThreadNew()
 * 都在生成区，不要手写；这里只填任务入口里的业务代码。
 *
 * 组成：
 *   S2_CanInit()    标准过滤器 + 全局过滤 + 启动
 *   S2_Transmit()   提交 → 等 TXBTO → 超时取消
 *   S2_OnFrame()    合法性判断 + 应答（应用层入口）
 *   StartCanTask()  FIFO0 轮询（每轮最多 4 帧）+ 上电启动帧
 * =========================================================================== */

/**
  * @brief  配置标准过滤器与全局过滤，并启动 FDCAN1，只执行一次。
  * @note   任何一步失败都停在对应 g_s2.init_step，结合 g_s2.init_ok 判断是否可收发。
  */
static void S2_CanInit(void)
{
  FDCAN_FilterTypeDef filter = {0};
  HAL_StatusTypeDef status   = HAL_OK;

  g_s2.init_ok   = 0U;
  g_s2.init_step = 1U;

  /* 1. 标准 ID 掩码过滤：只有 S2_ID_REQUEST 能进 FIFO0 */
  filter.IdType           = FDCAN_STANDARD_ID;
  filter.FilterIndex      = 0U;
  filter.FilterType       = FDCAN_FILTER_MASK;
  filter.FilterConfig     = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1        = S2_ID_REQUEST;
  filter.FilterID2        = S2_STD_ID_MASK;
  filter.RxBufferIndex    = 0U;
  filter.IsCalibrationMsg = 0U;

  status = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  if (HAL_OK != status)
  {
    return;
  }

  /* 2. 未匹配的标准帧、扩展帧与两类远程帧全部拒绝 */
  g_s2.init_step = 2U;
  status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,
                                        FDCAN_REJECT_REMOTE);
  if (HAL_OK != status)
  {
    return;
  }

  /* 3. 启动 FDCAN1，只启动一次 */
  g_s2.init_step = 3U;
  status = HAL_FDCAN_Start(&hfdcan1);
  if (HAL_OK != status)
  {
    return;
  }

  g_s2.init_step = 4U;
  g_s2.init_ok   = 1U;
}

/**
  * @brief  S2 发送函数：提交、等待 TXBTO、超时收尾。
  * @param  id   标准 ID
  * @param  data 指向 S2_DATA_BYTES 字节载荷，调用期间只读
  * @retval HAL_OK      = 已确认发送完成
  *         HAL_TIMEOUT = 超时未完成（取消流程已执行完毕）
  *         HAL_ERROR   = 提交失败、槽位异常，或取消后仍挂起
  * @note   超时后，若帧在取消窗口内才真正发完，会同时保留 g_s2.tx_timeout_num 与
  *         g_s2.tx_confirmed_num 两份证据。
  */
static HAL_StatusTypeDef S2_Transmit(uint32_t id, const uint8_t *data)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};        /* 发送帧头模板 */
  HAL_StatusTypeDef status       = HAL_ERROR;  /* 各步骤的 HAL 返回值 */
  uint32_t slotMask              = 0U;         /* 本次占用的 Tx FIFO 槽位掩码 */
  uint32_t startTick             = 0U;         /* 提交时刻，用于超时预算 */
  uint32_t waitTick              = 0U;         /* 等待已消耗的 tick 数 */

  txHeader.Identifier          = id;
  txHeader.IdType              = FDCAN_STANDARD_ID;
  txHeader.TxFrameType         = FDCAN_DATA_FRAME;
  txHeader.DataLength          = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker       = 0U;

  /* 上一次取消没收干净时，不再提交新帧（保留现场，等排障） */
  if (0U != g_s2.tx_stuck)
  {
    return HAL_ERROR;
  }

  /* ===== 第 1 步：提交 ===== */
  status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
  if (HAL_OK != status)
  {
    g_s2.tx_submit_fail_num++;
    return status;
  }
  g_s2.tx_submitted_num++;

  /* 提交成功后立刻取本次占用的槽位掩码：下一次提交会覆盖它 */
  slotMask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
  g_s2.tx_slot_mask = slotMask;
  if (0U == slotMask)
  {
    g_s2.tx_submit_fail_num++;
    return HAL_ERROR;
  }

  /* ===== 第 2 步：等待 TXBTO，预算 S2_TX_TIMEOUT_TICKS ===== */
  startTick = osKernelGetTickCount();
  while (0U == (hfdcan1.Instance->TXBTO & slotMask))
  {
    if ((uint32_t)(osKernelGetTickCount() - startTick) >= S2_TX_TIMEOUT_TICKS)
    {
      break;   /* 超时，转第 3 步 */
    }
    (void)osDelay(1U);
  }

  if (0U != (hfdcan1.Instance->TXBTO & slotMask))
  {
    g_s2.tx_confirmed_num++;
    return HAL_OK;
  }

  /* ===== 第 3 步：超时收尾 ===== */
  g_s2.tx_timeout_num++;

  /* 3.1 仍挂起才请求取消；取消接口返回 HAL_OK 只代表“请求已受理” */
  if (0U != (hfdcan1.Instance->TXBRP & slotMask))
  {
    status = HAL_FDCAN_AbortTxRequest(&hfdcan1, slotMask);
    if (HAL_OK == status)
    {
      g_s2.tx_cancel_num++;
    }
  }

  /* 3.2 在取消窗口内观察 pending 是否真的撤离（TXBCF 可作为辅助对照） */
  waitTick = osKernelGetTickCount();
  while ((0U != (hfdcan1.Instance->TXBRP & slotMask)) &&
         ((uint32_t)(osKernelGetTickCount() - waitTick) < S2_TX_CANCEL_TICKS))
  {
    (void)osDelay(1U);
  }

  if (0U != (hfdcan1.Instance->TXBRP & slotMask))
  {
    /* 一直清不掉：置卡死标志，阻止后续提交并保留快照 */
    g_s2.tx_stuck = 1U;
    return HAL_ERROR;
  }

  /* 3.3 取消窗口结束后的复核才有意义：这 20 ms 里帧若真的发完，TXBTO 会置位，
         此时“超时”和“最终完成”两份证据同时保留 */
  if (0U != (hfdcan1.Instance->TXBTO & slotMask))
  {
    g_s2.tx_confirmed_num++;
  }

  return HAL_TIMEOUT;
}

/**
  * @brief  应用层收帧入口：合法性判断与应答都在这里完成。
  * @param  rxHeader 接收帧头（只读）
  * @param  data     接收数据，长度按 DLC 计算，缓冲至少 S2_RX_BUF_BYTES
  * @note   本函数由轮询循环调用；以下三步留给你实现：
  *
  *   第 1 步 合法性判断：
  *     标准帧 && 数据帧 && Identifier == S2_ID_REQUEST && DataLength == FDCAN_DLC_BYTES_8
  *     （不合格 -> g_s2.rx_bad_frame_num++; return;）
  *
  *   第 2 步 合格帧处理：
  *     g_s2.rx_valid_num++; 并把 8 字节复制到本地缓冲。
  *
  *   第 3 步 应答：
  *     调用 S2_Transmit(S2_ID_REPLY, 本地缓冲)，返回值可记录到 g_s2.tx_* 计数。
  */
#if (1 == S2_REPORT_ENABLE)
/**
  * @brief  通过 CAN 上报一帧状态：栈余量 + 堆剩余 + rx_valid_num 低字节。
  * @note   由 S2_REPORT_ENABLE 控制，默认关闭，避免干扰 T02–T04 的计数验收。
  */
static void S2_SendStatus(void)
{
  uint8_t buf[S2_DATA_BYTES] = {0};
  uint32_t canFree  = g_s2.stack_can_min;
  uint32_t ledFree  = g_s2.stack_led_min;
  uint32_t heapFree = g_s2.heap_free;

  buf[0] = (uint8_t)(canFree & 0xFFU);
  buf[1] = (uint8_t)((canFree >> 8U) & 0xFFU);
  buf[2] = (uint8_t)(ledFree & 0xFFU);
  buf[3] = (uint8_t)((ledFree >> 8U) & 0xFFU);
  buf[4] = (uint8_t)(heapFree & 0xFFU);
  buf[5] = (uint8_t)((heapFree >> 8U) & 0xFFU);
  buf[6] = (uint8_t)((heapFree >> 16U) & 0xFFU);
  buf[7] = (uint8_t)(g_s2.rx_valid_num & 0xFFU);

  (void)S2_Transmit(S2_ID_STATUS, buf);
}
#endif /* S2_REPORT_ENABLE */

static void S2_OnFrame(const FDCAN_RxHeaderTypeDef *rxHeader, const uint8_t *data)
{
	/*数据合法性判断*/
	if((FDCAN_STANDARD_ID != rxHeader->IdType)||
		(FDCAN_DATA_FRAME != rxHeader->RxFrameType)||
		(S2_ID_REQUEST    != rxHeader->Identifier)||
		(FDCAN_DLC_BYTES_8!= rxHeader->DataLength))
	{
		g_s2.rx_bad_frame_num++;/*记录一次不处理的无效帧*/
		return ;
	}
	/*记录一次需要处理的有效帧*/
	g_s2.rx_valid_num++;
	/*调试阶段先原样返回8字节*/
	(void)S2_Transmit(S2_ID_REPLY, data);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t lastAlive = 0U;    /* 上一轮看到的 task_alive_num，用于判断 CanTask 是否在推进 */
#if (1 == S2_REPORT_ENABLE)
  uint32_t reportTick = 0U;   /* 上报节拍，每 2 个循环（约 1 s）发一帧 */
#endif
#ifdef S1_SELFTEST_ENABLE
  /* S1：FDCAN1 内部回环自测，只执行一次；失败也继续进入下面的心跳循环 */
  S1_FdcanLoopbackTest();
#endif /* S1_SELFTEST_ENABLE */

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  g_heartbeat_num++;
	  /* 水位采样：每 500 ms 一次；栈余量取历史最小值，堆取当前值 */
	  g_s2.stack_can_min = (uint32_t)osThreadGetStackSpace(CanTaskHandle);
	  g_s2.stack_led_min = (uint32_t)osThreadGetStackSpace(defaultTaskHandle);
	  g_s2.heap_free     = (uint32_t)xPortGetFreeHeapSize();

	  /* 喂狗条件：CanTask 的存活计数在推进才喂；卡死则不喂，约 4 s 后硬件复位 */
	  if (g_s2.task_alive_num != lastAlive)
	  {
	    lastAlive = g_s2.task_alive_num;
	    (void)HAL_IWDG_Refresh(&hiwdg1);
	  }

#if (1 == S2_REPORT_ENABLE)
	  /* 每 2 个循环（约 1 s）上报一帧 0x322 */
	  reportTick++;
	  if (reportTick >= (S2_REPORT_PERIOD_MS / 500U))
	  {
	    reportTick = 0U;
	    S2_SendStatus();
	  }
#endif
	  osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the CanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void *argument)
{
  /* USER CODE BEGIN StartCanTask */
  uint8_t txPayload[S2_DATA_BYTES] = {0x53U, 0x32U, 0x41U};    /* 启动帧载荷 "S2A"，其余补 0 */
  FDCAN_RxHeaderTypeDef rxHeader = {0};                        /* 接收帧头：ID/DLC/帧类型 */
  uint8_t rxData[S2_RX_BUF_BYTES] = {0};                       /* 接收缓冲，按 64 字节给足 */
  HAL_StatusTypeDef status = HAL_OK;                           /* 取帧调用的返回值 */
  uint32_t rxRound = 0U;                                       /* 本轮已取帧数，上限 S2_RX_MAX_PER_ROUND */


  (void)argument;

  /* 任务能跑到这里，说明 CubeMX 生成的 osThreadNew 成功了 */
  g_s2.task_created = 1U;

  /* 计数器归零：既是自测基线，也让这些调试变量保留在 ELF 里（否则会被 --gc-sections 回收） */
  g_s2.tx_submitted_num   = 0U;
  g_s2.tx_confirmed_num   = 0U;
  g_s2.tx_submit_fail_num = 0U;
  g_s2.tx_timeout_num     = 0U;
  g_s2.tx_cancel_num      = 0U;
  g_s2.tx_slot_mask   = 0U;
  g_s2.tx_stuck       = 0U;
  g_s2.rx_read_num        = 0U;
  g_s2.rx_valid_num       = 0U;
  g_s2.rx_bad_frame_num   = 0U;
  g_s2.rx_last_id     = 0U;
  g_s2.rx_last_dlc    = 0U;
  g_s2.rx_data0       = 0U;
  g_s2.rx_data1       = 0U;
  g_s2.stack_can_min  = 0U;
  g_s2.stack_led_min  = 0U;
  g_s2.heap_free      = 0U;

  S2_CanInit();



  if (1U == g_s2.init_ok)
  {
    /* 上电只发一帧启动帧，作为“板子已就绪”的标志 */
    (void)S2_Transmit(S2_ID_REPLY, txPayload);
  }

  /* Infinite loop */
  for(;;)
  {
	#if (0 == S2_STALL_INJECT)
	/* T06 默认任务不喂狗，应在 ~4 s 时复位 */
	g_s2.task_alive_num++;
	#endif

    if (1U == g_s2.init_ok)
    {
      /* 轮询 FIFO0：先查填充量，再取帧，每轮最多 S2_RX_MAX_PER_ROUND 帧 */
      rxRound = 0U;
      while ((rxRound < S2_RX_MAX_PER_ROUND) &&
             (0U != HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)))
      {
        status = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);
        if (HAL_OK != status)
        {
          g_s2.rx_bad_frame_num++;
          break;
        }

        g_s2.rx_read_num++;
        g_s2.rx_last_id  = rxHeader.Identifier;
        g_s2.rx_last_dlc = rxHeader.DataLength;
        g_s2.rx_data0 = ((uint32_t)rxData[0])
                      | ((uint32_t)rxData[1] << 8U)
                      | ((uint32_t)rxData[2] << 16U)
                      | ((uint32_t)rxData[3] << 24U);
        g_s2.rx_data1 = ((uint32_t)rxData[4])
                      | ((uint32_t)rxData[5] << 8U)
                      | ((uint32_t)rxData[6] << 16U)
                      | ((uint32_t)rxData[7] << 24U);

        S2_OnFrame(&rxHeader, rxData);
        rxRound++;
      }
    }


    (void)osDelay(S2_TASK_PERIOD_TICKS);
  }
  /* USER CODE END StartCanTask */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
