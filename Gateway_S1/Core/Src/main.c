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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* S0 调试观测：仅由调试器读取，不参与业务逻辑 */
volatile uint32_t g_dbg_idcode = 0U;   /* DBGMCU->IDCODE 原值 */
volatile uint32_t g_rev_id     = 0U;   /* 由 IDCODE[31:16] 提取的修订号 */
volatile uint32_t g_heartbeat  = 0U;   /* defaultTask 心跳计数，仅本任务写入 */

/* S1 自测结果：只由 defaultTask 写，调试器只读 */
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
void StartDefaultTask(void *argument);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  /* S1：FDCAN1 内部回环自测，只执行一次；失败也继续进入下面的心跳循环 */
  S1_FdcanLoopbackTest();

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	  g_heartbeat++;
	  osDelay(500);
  }
  /* USER CODE END 5 */
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
