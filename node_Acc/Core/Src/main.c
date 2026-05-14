/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Nó IMU — MPU6050/MPU9250/MPU9255 via I2C + envio CAN
  *
  * Sistema de Telemetria UFS-01B | Fórmula Route UFSCar
  *
  * Frames CAN transmitidos (Documento de Decisões Técnicas v2.0):
  *
  *   ID 0x201 | DLC 8 | accelX_H, accelX_L,
  *                       accelY_H, accelY_L,
  *                       accelZ_H, accelZ_L,
  *                       gyroX_H,  gyroX_L
  *
  *   ID 0x204 | DLC 4 | gyroY_H, gyroY_L,
  *                       gyroZ_H, gyroZ_L
  *
  * Bit rate CAN : 500 kbps  (PCLK1 = 36 MHz, Prescaler=4, BS1=15TQ, BS2=2TQ)
  * Pinos CAN    : PA11 = CAN_RX  |  PA12 = CAN_TX  (ou PB8/PB9 com remap)
  *
  * Sensor: detecta automaticamente entre MPU6050, MPU6500, MPU9250 e MPU9255
  *   I2C  : PB6 = SCL | PB7 = SDA (Fast Mode 400 kHz)
  *   Acel : ±2 g    → 16384 LSB/g
  *   Gyro : ±250°/s → 131 LSB/(°/s)
  *
  * LED de diagnóstico (PC13 = LED onboard da Blue Pill, ATIVO LOW):
  *   1 piscada  = WHO_AM_I não bate (sensor não suportado)
  *   2 piscadas = Falha de I2C (sem resposta no barramento)
  *   3 piscadas = Falha na inicialização do CAN
  *   4 piscadas = CAN TX timeout (sem ACK no barramento)
  *
  * Taxa de envio: 50 Hz (20 ms)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    ERR_NONE       = 0,
    ERR_MPU_WHOAMI = 1,
    ERR_MPU_I2C    = 2,
    ERR_CAN_INIT   = 3,
    ERR_CAN_TX     = 4,
    ERR_HAL        = 5
} SystemError_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------- MPU ---------- */
#define MPU_ADDR             (0x68 << 1)  /* Endereço 7-bit shifted          */
#define MPU_REG_WHO_AM_I     0x75
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CFG     0x1B
#define MPU_REG_ACCEL_CFG    0x1C
#define MPU_REG_ACCEL_OUT    0x3B   /* Burst de 14 bytes a partir daqui     */

/* WHO_AM_I aceitos */
#define MPU6050_ID           0x68
#define MPU6500_ID           0x70
#define MPU9250_ID           0x71
#define MPU9255_ID           0x73

/* ---------- CAN IDs (Documento v2.0) ---------- */
#define CAN_ID_ACCEL_GYROX   0x201
#define CAN_ID_GYRO_YZ       0x204

/* ---------- Temporização ---------- */
#define IMU_PERIOD_MS        20   /* 50 Hz                                  */
#define CAN_TX_TIMEOUT_MS    5
#define I2C_TIMEOUT_MS       10

/* ---------- LED Heartbeat (PC13 — Blue Pill onboard, ativo LOW) ---------- */
#define LED_PORT             GPIOC
#define LED_PIN              GPIO_PIN_13

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
static uint8_t  g_mpuType      = 0;       /* WHO_AM_I detectado             */
static uint32_t g_canTxErrors  = 0;
static uint32_t g_i2cErrors    = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef MPU_Init(void);
static HAL_StatusTypeDef MPU_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                                     int16_t *gx, int16_t *gy, int16_t *gz);
static HAL_StatusTypeDef CAN_WaitMailbox(uint32_t timeout_ms);
static HAL_StatusTypeDef CAN_SendIMU(int16_t ax, int16_t ay, int16_t az,
                                     int16_t gx, int16_t gy, int16_t gz);
static void              ErrorBlink(SystemError_t code);
static void              LED_On(void);
static void              LED_Off(void);
static void              LED_Toggle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==========================================================================
 * LED helpers — PC13 da Blue Pill é ativo LOW (RESET = LED ON)
 * ========================================================================== */

static void LED_On(void)     { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); }
static void LED_Off(void)    { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   }
static void LED_Toggle(void) { HAL_GPIO_TogglePin(LED_PORT, LED_PIN);                }

/* ==========================================================================
 * MPU — Detecção e inicialização
 *
 * Aceita: 0x68 (MPU6050), 0x70 (MPU6500), 0x71 (MPU9250), 0x73 (MPU9255)
 * ========================================================================== */

static HAL_StatusTypeDef MPU_Init(void)
{
    uint8_t check, data;
    HAL_StatusTypeDef status;

    /* Lê WHO_AM_I — falha de I2C indica sensor desconectado */
    status = HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_REG_WHO_AM_I,
                              1, &check, 1, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        return HAL_ERROR;  /* tratado externamente como ERR_MPU_I2C */
    }

    /* Verifica se o ID retornado é de um sensor suportado */
    if (check != MPU6050_ID && check != MPU6500_ID &&
        check != MPU9250_ID && check != MPU9255_ID) {
        g_mpuType = check;
        return HAL_TIMEOUT;  /* tratado externamente como ERR_MPU_WHOAMI */
    }
    g_mpuType = check;

    /* PWR_MGMT_1: sai de sleep, clock = PLL com referência Gyro X */
    data = 0x01;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, MPU_REG_PWR_MGMT_1,
                          1, &data, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(100);  /* Aguarda PLL estabilizar */

    /* CONFIG: DLPF banda 41 Hz (filtra ruído de vibração) */
    data = 0x03;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, MPU_REG_CONFIG,
                          1, &data, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    /* ACCEL_CONFIG: ±2g (AFS_SEL = 0) → 16384 LSB/g */
    data = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, MPU_REG_ACCEL_CFG,
                          1, &data, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    /* GYRO_CONFIG: ±250°/s (FS_SEL = 0) → 131 LSB/(°/s) */
    data = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, MPU_REG_GYRO_CFG,
                          1, &data, 1, I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Lê accel XYZ + gyro XYZ em um único burst de 14 bytes.
 *        Bytes 6-7 contêm temperatura (descartada).
 */
static HAL_StatusTypeDef MPU_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                                     int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t raw[14];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_REG_ACCEL_OUT,
                              1, raw, 14, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        g_i2cErrors++;
        *ax = *ay = *az = 0;
        *gx = *gy = *gz = 0;
        return status;
    }

    *ax = (int16_t)((raw[0]  << 8) | raw[1]);
    *ay = (int16_t)((raw[2]  << 8) | raw[3]);
    *az = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6..7] = temperatura — ignorado */
    *gx = (int16_t)((raw[8]  << 8) | raw[9]);
    *gy = (int16_t)((raw[10] << 8) | raw[11]);
    *gz = (int16_t)((raw[12] << 8) | raw[13]);

    return HAL_OK;
}

/* ==========================================================================
 * CAN — Transmissão de frames 0x201 e 0x204
 * ========================================================================== */

static HAL_StatusTypeDef CAN_WaitMailbox(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef CAN_SendIMU(int16_t ax, int16_t ay, int16_t az,
                                     int16_t gx, int16_t gy, int16_t gz)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t  txData[8];
    uint32_t txMailbox;

    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.TransmitGlobalTime = DISABLE;
    txHeader.ExtId              = 0;

    /* --- Frame 0x201: Acel XYZ + Gyro X (8 bytes) --- */
    txHeader.StdId = CAN_ID_ACCEL_GYROX;
    txHeader.DLC   = 8;

    txData[0] = (uint8_t)((ax >> 8) & 0xFF);
    txData[1] = (uint8_t)( ax       & 0xFF);
    txData[2] = (uint8_t)((ay >> 8) & 0xFF);
    txData[3] = (uint8_t)( ay       & 0xFF);
    txData[4] = (uint8_t)((az >> 8) & 0xFF);
    txData[5] = (uint8_t)( az       & 0xFF);
    txData[6] = (uint8_t)((gx >> 8) & 0xFF);
    txData[7] = (uint8_t)( gx       & 0xFF);

    if (CAN_WaitMailbox(CAN_TX_TIMEOUT_MS) != HAL_OK) {
        g_canTxErrors++;
        return HAL_TIMEOUT;
    }
    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK) {
        g_canTxErrors++;
        return HAL_ERROR;
    }

    /* --- Frame 0x204: Gyro Y + Z (4 bytes) --- */
    txHeader.StdId = CAN_ID_GYRO_YZ;
    txHeader.DLC   = 4;

    txData[0] = (uint8_t)((gy >> 8) & 0xFF);
    txData[1] = (uint8_t)( gy       & 0xFF);
    txData[2] = (uint8_t)((gz >> 8) & 0xFF);
    txData[3] = (uint8_t)( gz       & 0xFF);

    if (CAN_WaitMailbox(CAN_TX_TIMEOUT_MS) != HAL_OK) {
        g_canTxErrors++;
        return HAL_TIMEOUT;
    }
    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK) {
        g_canTxErrors++;
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ==========================================================================
 * Error Handler com LED — pisca N vezes conforme código de erro
 *
 *   1 piscada  = WHO_AM_I invalido (sensor nao suportado)
 *   2 piscadas = I2C falhou (sensor nao responde / fiacao)
 *   3 piscadas = CAN init falhou
 *   4 piscadas = CAN TX timeout (sem ACK no barramento)
 *   5 piscadas = HAL generico
 *
 *   NUNCA RETORNA — é um trap de erro fatal.
 * ========================================================================== */
static void ErrorBlink(SystemError_t code)
{
    uint8_t n = (uint8_t)code;
    if (n == 0) n = 5;

    while (1) {
        for (uint8_t i = 0; i < n; i++) {
            LED_On();
            HAL_Delay(150);
            LED_Off();
            HAL_Delay(150);
        }
        HAL_Delay(1000);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* LED aceso durante init */
  LED_On();

  /* Aguarda MPU completar boot interno após power-on */
  HAL_Delay(150);

  /* Inicializa MPU — trata os dois modos de falha separadamente */
  HAL_StatusTypeDef mpuStatus = MPU_Init();
  if (mpuStatus == HAL_ERROR) {
      /* I2C respondeu mas escrita/leitura falhou — fiação ou pull-ups */
      ErrorBlink(ERR_MPU_I2C);
  } else if (mpuStatus == HAL_TIMEOUT) {
      /* WHO_AM_I retornou valor desconhecido — sensor não suportado */
      ErrorBlink(ERR_MPU_WHOAMI);
  } else if (mpuStatus != HAL_OK) {
      ErrorBlink(ERR_HAL);
  }

  /* Configura filtro CAN (aceitar tudo — este nó só transmite) */
  CAN_FilterTypeDef canfilter = {0};
  canfilter.FilterBank           = 0;
  canfilter.FilterMode           = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale          = CAN_FILTERSCALE_32BIT;
  canfilter.FilterIdHigh         = 0x0000;
  canfilter.FilterIdLow          = 0x0000;
  canfilter.FilterMaskIdHigh     = 0x0000;
  canfilter.FilterMaskIdLow      = 0x0000;
  canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilter.FilterActivation     = ENABLE;
  canfilter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &canfilter) != HAL_OK) {
      ErrorBlink(ERR_CAN_INIT);
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
      ErrorBlink(ERR_CAN_INIT);
  }

  /* LED apaga — init OK */
  LED_Off();

  /* USER CODE END 2 */

  /* Infinite loop -----------------------------------------------------------*/
  /* USER CODE BEGIN WHILE */

  uint32_t lastTick    = HAL_GetTick();
  uint32_t ledToggleCt = 0;

  while (1)
  {
      uint32_t now = HAL_GetTick();

      /* Período de 20 ms (50 Hz) sem drift acumulado */
      if ((now - lastTick) >= IMU_PERIOD_MS)
      {
          lastTick += IMU_PERIOD_MS;

          int16_t ax, ay, az, gx, gy, gz;

          /* Lê sensores — falha envia zeros mas não trava */
          MPU_ReadAll(&ax, &ay, &az, &gx, &gy, &gz);

          /* Transmite pela CAN */
          CAN_SendIMU(ax, ay, az, gx, gy, gz);

          /* Heartbeat: toggle LED a cada 500 ms (25 ciclos de 20 ms) */
          ledToggleCt++;
          if (ledToggleCt >= 25) {
              ledToggleCt = 0;
              LED_Toggle();
          }
      }

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *
  * NOTA: HSE_BYPASS porque a Nucleo recebe o clock externamente do ST-Link.
  *       Se usar Blue Pill, trocar para RCC_HSE_ON.
  *
  * HSE 8 MHz → PLL ×9 → SYSCLK 72 MHz
  * APB1 = 36 MHz (CAN clock source)
  * APB2 = 72 MHz
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;   /* Nucleo */
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  *
  *  CAN clock = APB1 = 36 MHz
  *  Prescaler = 4 → TQ = 4/36 MHz = 111.1 ns
  *  BS1 = 15 TQ, BS2 = 2 TQ → total = 1 + 15 + 2 = 18 TQ
  *  Bit rate = 36 MHz / (4 × 18) = 500 kbps ✓
  *  Sample point = (1 + 15) / 18 = 88.9%
  *
  *  Mesmo timing usado pelo ESP32, garantindo interoperabilidade.
  */
static void MX_CAN_Init(void)
{
  hcan.Instance                  = CAN1;
  hcan.Init.Prescaler            = 4;             /* 500 kbps */
  hcan.Init.Mode                 = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1             = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2             = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode    = DISABLE;
  hcan.Init.AutoBusOff           = ENABLE;
  hcan.Init.AutoWakeUp           = DISABLE;
  hcan.Init.AutoRetransmission   = DISABLE;
  hcan.Init.ReceiveFifoLocked    = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  *        PB6 = SCL | PB7 = SDA | Fast Mode 400 kHz
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 400000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  *        Configura LED de heartbeat em PC13 (Blue Pill onboard).
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PC13 = LED onboard (ativo LOW). Inicia desligado (HIGH). */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin   = LED_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  *         Pisca LED 5x (ERR_HAL) em loop infinito.
  */
void Error_Handler(void)
{
  __disable_irq();
  ErrorBlink(ERR_HAL);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  Error_Handler();
}
#endif /* USE_FULL_ASSERT */
