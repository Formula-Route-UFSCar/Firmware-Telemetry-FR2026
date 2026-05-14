/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Nó Analógico — Hall Acelerador + Hall Freio
  *
  * Sistema de Telemetria UFS-01B | Fórmula Route UFSCar
  *
  * Frame CAN transmitido (conforme Documento de Decisões Técnicas v2.0):
  *
  *   ID 0x202 | DLC 8 | hall_acel_H,  hall_acel_L,
  *                       hall_freio_H, hall_freio_L,
  *                       0x00, 0x00, 0x00, 0x00       (reservados)
  *
  * Bit rate CAN : 500 kbps  (PCLK1 = 36 MHz, Prescaler=4, BS1=15TQ, BS2=2TQ)
  * Pinos CAN    : PA11 = CAN_RX  |  PA12 = CAN_TX
  *
  * Sensores:
  *   PA0 → Hall Acelerador (0–5V analógico, ADC CH0)
  *   PA1 → Hall Freio      (0–5V analógico, ADC CH1)
  *
  * ADC:
  *   Resolução  : 12 bits (0–4095)
  *   Scan Mode  : ENABLE (leitura sequencial de 2 canais)
  *   Clock ADC  : APB2/6 = 72 MHz / 6 = 12 MHz
  *   Sampling   : 55.5 ciclos → ~4.6 µs por canal
  *
  * Conversão no receptor:
  *   hall_pct = (adc_raw * 99.0) / 4095.0   → 0–99%
  *
  * Taxa de envio : 50 Hz (20 ms) conforme documento
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/** @brief Códigos de erro do sistema para diagnóstico via LED */
typedef enum {
    ERR_NONE     = 0,
    ERR_ADC_INIT = 1,   /* Falha na inicialização do ADC                     */
    ERR_ADC_READ = 2,   /* Timeout ou falha na conversão ADC                 */
    ERR_CAN_INIT = 3,   /* Falha na inicialização do periférico CAN          */
    ERR_CAN_TX   = 4,   /* Timeout ao transmitir frame CAN (sem ACK?)        */
    ERR_HAL      = 5    /* Erro genérico da HAL                              */
} SystemError_t;

/** @brief Leitura raw dos sensores Hall (12 bits, 0–4095) */
typedef struct {
    uint16_t hallAcel;
    uint16_t hallFreio;
} LeituraAnalog_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------- CAN ID (Documento v2.0) ---------- */
#define CAN_ID_PEDAIS         0x202   /* Hall Acel + Freio (DLC 8)            */

/* ---------- Temporização ---------- */
#define ADC_PERIOD_MS         20      /* 50 Hz conforme documento             */
#define CAN_TX_TIMEOUT_MS     5       /* Timeout para esperar mailbox livre   */
#define ADC_POLL_TIMEOUT_MS   10      /* Timeout para conversão ADC           */

/* ---------- ADC ---------- */
#define ADC_NUM_CHANNELS      2       /* CH0 = Acelerador, CH1 = Freio        */

/* ---------- Filtro média móvel (anti-ruído) ---------- */
#define ADC_FILTER_SAMPLES    4       /* Média de 4 leituras por ciclo        */

/* ---------- LED Heartbeat (PC13 na Blue Pill, ativo LOW) ---------- */
#define LED_PORT              GPIOC
#define LED_PIN               GPIO_PIN_13

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
static volatile SystemError_t g_lastError    = ERR_NONE;
static volatile uint32_t      g_canTxErrors  = 0;
static volatile uint32_t      g_adcErrors    = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef ADC_ReadAll(LeituraAnalog_t *out);
static HAL_StatusTypeDef ADC_ReadFiltered(LeituraAnalog_t *out);
static HAL_StatusTypeDef CAN_SendPedais(const LeituraAnalog_t *analog);
static HAL_StatusTypeDef CAN_WaitMailbox(uint32_t timeout_ms);
static void              ErrorBlink(SystemError_t code);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==========================================================================
 * ADC — Leitura sequencial de 2 canais via Scan Mode
 *
 * CORREÇÕES vs. código original:
 *   1. ScanConvMode mudado de DISABLE → ENABLE
 *   2. NbrOfConversion mudado de 1 → 2
 *   3. DMA removido (nunca estava conectado ao ADC, gerava IRQ fantasma)
 *   4. Timeout de 10ms por canal em vez de HAL_MAX_DELAY
 *   5. Adicionada média móvel de 4 amostras para reduzir ruído
 * ========================================================================== */

/**
 * @brief  Lê os 2 canais ADC sequencialmente usando scan mode.
 *
 *         Rank 1 → CH0 → PA0 → Hall Acelerador
 *         Rank 2 → CH1 → PA1 → Hall Freio
 *
 * @param  out  Ponteiro para estrutura de saída com valores raw (0–4095)
 * @retval HAL_OK em sucesso, HAL_ERROR/HAL_TIMEOUT em falha
 */
static HAL_StatusTypeDef ADC_ReadAll(LeituraAnalog_t *out)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    HAL_StatusTypeDef status;

    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    sConfig.Rank         = ADC_REGULAR_RANK_1;

    /* ===== Canal 0: Hall Acelerador (PA0) ===== */
    sConfig.Channel = ADC_CHANNEL_0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        g_adcErrors++;
        memset(out, 0, sizeof(*out));
        return HAL_ERROR;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        g_adcErrors++;
        memset(out, 0, sizeof(*out));
        return HAL_ERROR;
    }

    status = HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT_MS);
    if (status != HAL_OK) {
        g_adcErrors++;
        HAL_ADC_Stop(&hadc1);
        memset(out, 0, sizeof(*out));
        return status;
    }
    out->hallAcel = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* ===== Canal 1: Hall Freio (PA1) ===== */
    sConfig.Channel = ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        g_adcErrors++;
        out->hallFreio = 0;
        return HAL_ERROR;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        g_adcErrors++;
        out->hallFreio = 0;
        return HAL_ERROR;
    }

    status = HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT_MS);
    if (status != HAL_OK) {
        g_adcErrors++;
        HAL_ADC_Stop(&hadc1);
        out->hallFreio = 0;
        return status;
    }
    out->hallFreio = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return HAL_OK;
}

/**
 * @brief  Lê os 2 canais ADC com média móvel de N amostras.
 *         Reduz ruído do sinal analógico sem custo de hardware.
 *
 * @param  out  Ponteiro para estrutura com média dos valores raw
 * @retval HAL_OK se todas as leituras OK, último erro caso contrário
 */
static HAL_StatusTypeDef ADC_ReadFiltered(LeituraAnalog_t *out)
{
    uint32_t sumAcel  = 0;
    uint32_t sumFreio = 0;
    uint8_t  validSamples = 0;
    HAL_StatusTypeDef lastStatus = HAL_OK;

    for (uint8_t i = 0; i < ADC_FILTER_SAMPLES; i++) {
        LeituraAnalog_t raw;
        HAL_StatusTypeDef status = ADC_ReadAll(&raw);

        if (status == HAL_OK) {
            sumAcel  += raw.hallAcel;
            sumFreio += raw.hallFreio;
            validSamples++;
        } else {
            lastStatus = status;
        }
    }

    if (validSamples > 0) {
        out->hallAcel  = (uint16_t)(sumAcel  / validSamples);
        out->hallFreio = (uint16_t)(sumFreio / validSamples);
        return HAL_OK;
    }

    /* Nenhuma leitura válida */
    memset(out, 0, sizeof(*out));
    return lastStatus;
}

/* ==========================================================================
 * CAN — Transmissão conforme Documento de Decisões Técnicas v2.0
 *
 *  Frame 0x202 (DLC 8):
 *    Bytes 0-1: Hall Acelerador (big-endian, 0–4095)
 *    Bytes 2-3: Hall Freio      (big-endian, 0–4095)
 *    Bytes 4-7: Reservados (0x00)
 * ========================================================================== */

/**
 * @brief  Aguarda mailbox CAN livre com timeout.
 * @retval HAL_OK se mailbox disponível, HAL_TIMEOUT se esgotou.
 */
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

/**
 * @brief  Envia frame 0x202 com dados dos pedais.
 * @retval HAL_OK se enviado, HAL_ERROR/HAL_TIMEOUT caso contrário.
 */
static HAL_StatusTypeDef CAN_SendPedais(const LeituraAnalog_t *analog)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox;
    HAL_StatusTypeDef status;

    txHeader.StdId              = CAN_ID_PEDAIS;
    txHeader.DLC                = 8;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.TransmitGlobalTime = DISABLE;
    txHeader.ExtId              = 0;

    /* Bytes 0-1: Hall Acelerador (big-endian) */
    txData[0] = (uint8_t)((analog->hallAcel  >> 8) & 0xFF);
    txData[1] = (uint8_t)( analog->hallAcel        & 0xFF);

    /* Bytes 2-3: Hall Freio (big-endian) */
    txData[2] = (uint8_t)((analog->hallFreio >> 8) & 0xFF);
    txData[3] = (uint8_t)( analog->hallFreio       & 0xFF);

    /* Bytes 4-7: Reservados */
    txData[4] = 0x00;
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;

    if (CAN_WaitMailbox(CAN_TX_TIMEOUT_MS) != HAL_OK) {
        g_canTxErrors++;
        g_lastError = ERR_CAN_TX;
        return HAL_TIMEOUT;
    }

    status = HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
    if (status != HAL_OK) {
        g_canTxErrors++;
        g_lastError = ERR_CAN_TX;
        return status;
    }

    return HAL_OK;
}

/* ==========================================================================
 * Error Handler com LED — pisca N vezes conforme código de erro
 *
 *   1 piscada  = ADC init falhou
 *   2 piscadas = ADC leitura falhou (timeout na conversão)
 *   3 piscadas = CAN init falhou
 *   4 piscadas = CAN TX timeout (sem ACK — outro nó não no barramento?)
 *   5 piscadas = HAL genérico
 *
 *   Padrão: pisca N vezes rápido, pausa 1s, repete.
 *   NUNCA RETORNA — é um trap de erro fatal.
 * ========================================================================== */
static void ErrorBlink(SystemError_t code)
{
    uint8_t n = (uint8_t)code;
    if (n == 0) n = 5;

    /* Garante que o LED está configurado mesmo se GPIO_Init falhou */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);

    while (1) {
        for (uint8_t i = 0; i < n; i++) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); /* LED ON  */
            HAL_Delay(150);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   /* LED OFF */
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
  /* MCU Configuration -------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();

  /* USER CODE BEGIN 2 */

  /* LED de heartbeat: acende durante init */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); /* ON */

  /* Calibração do ADC (recomendado pelo Reference Manual antes do primeiro uso) */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Configura filtro CAN: aceitar tudo (este nó só transmite) */
  CAN_FilterTypeDef canFilter = {0};
  canFilter.FilterBank           = 0;
  canFilter.FilterMode           = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale          = CAN_FILTERSCALE_32BIT;
  canFilter.FilterIdHigh         = 0x0000;
  canFilter.FilterIdLow          = 0x0000;
  canFilter.FilterMaskIdHigh     = 0x0000;
  canFilter.FilterMaskIdLow      = 0x0000;
  canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilter.FilterActivation     = ENABLE;
  canFilter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK) {
      g_lastError = ERR_CAN_INIT;
      ErrorBlink(ERR_CAN_INIT);
  }

  /* Inicia periférico CAN */
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
      g_lastError = ERR_CAN_INIT;
      ErrorBlink(ERR_CAN_INIT);
  }

  /* Teste rápido do ADC: lê uma vez para validar que os canais respondem */
  {
      LeituraAnalog_t teste;
      HAL_StatusTypeDef adcTest = ADC_ReadAll(&teste);
      if (adcTest != HAL_OK) {
          g_lastError = ERR_ADC_READ;
          ErrorBlink(ERR_ADC_READ);
      }
  }

  /* LED apaga — init OK */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); /* OFF */

  /* USER CODE END 2 */

  /* Infinite loop -----------------------------------------------------------*/
  /* USER CODE BEGIN WHILE */

  uint32_t lastTick    = HAL_GetTick();
  uint32_t ledToggleCt = 0;

  while (1)
  {
      uint32_t now = HAL_GetTick();

      /* Controla período de 20 ms (50 Hz) sem drift acumulado */
      if ((now - lastTick) >= ADC_PERIOD_MS)
      {
          lastTick += ADC_PERIOD_MS;

          LeituraAnalog_t analog;

          /* Lê sensores com filtro de média — se falhar, envia zeros */
          ADC_ReadFiltered(&analog);

          /* Transmite pela CAN */
          CAN_SendPedais(&analog);

          /* Heartbeat: toggle LED a cada 500 ms (25 ciclos de 20 ms) */
          ledToggleCt++;
          if (ledToggleCt >= 25) {
              ledToggleCt = 0;
              HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
          }
      }

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  *        HSE 8 MHz → PLL ×9 → SYSCLK 72 MHz
  *        APB1 = 36 MHz (CAN clock source)
  *        APB2 = 72 MHz (ADC clock /6 = 12 MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct       = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct       = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit      = {0};

  RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState        = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState        = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL      = RCC_PLL_MUL9;
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

  /* ADC clock = APB2 / 6 = 72 MHz / 6 = 12 MHz */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
      Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization
  *
  * Scan Mode habilitado com 2 canais sequenciais:
  *   Rank 1: CH0 (PA0) → Hall Acelerador
  *   Rank 2: CH1 (PA1) → Hall Freio
  *
  * Sampling: 55.5 ciclos @ 12 MHz = ~4.6 µs por canal
  *
  * NOTA: NÃO usar DMA. A leitura é feita por polling com
  *       HAL_ADC_PollForConversion() — simples e confiável.
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance                   = ADC1;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;     /* Single channel    */
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;     /* 2 canais          */
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Rank 1: PA0 → Hall Acelerador (CH0) */
  sConfig.Channel      = ADC_CHANNEL_0;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
      Error_Handler();
  }

  /* Rank 2: PA1 → Hall Freio (CH1) */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank    = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
  *  NOTA: Timing idêntico ao Nó IMU para garantir interoperabilidade.
  */
static void MX_CAN_Init(void)
{
  hcan.Instance                  = CAN1;
  hcan.Init.Prescaler            = 4;
  hcan.Init.Mode                 = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1             = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2             = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode    = DISABLE;
  hcan.Init.AutoBusOff           = ENABLE;        /* Recupera automaticamente      */
  hcan.Init.AutoWakeUp           = DISABLE;
  hcan.Init.AutoRetransmission   = DISABLE;        /* DISABLE para bancada          */
  hcan.Init.ReceiveFifoLocked    = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
      Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  *        Habilita clocks e configura LED heartbeat (PC13)
  *
  *        PA0 e PA1 são configurados automaticamente pelo HAL_ADC_Init
  *        como entradas analógicas — não precisa configurar aqui.
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Habilita clocks dos portos utilizados */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* LED Heartbeat — PC13 (Blue Pill onboard, ativo LOW) */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); /* OFF inicialmente */

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
  *         Pisca LED 5× (ERR_HAL) em loop infinito.
  * @retval None
  */
void Error_Handler(void)
{
  g_lastError = ERR_HAL;
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
