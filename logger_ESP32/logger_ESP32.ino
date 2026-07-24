/**
 * ============================================================================
 * logger_ESP32.ino
 *
 * Sistema de Telemetria UFS-01B | Fórmula Route UFSCar
 * Logger principal — MCP2515 (SPI) + SD + LEDs + Display + Barra RPM
 *
 * Arquitetura:
 *   Core 1 (task_can)  : Lê MCP2515 via SPI, decodifica frames, enfileira.
 *                        Atualiza LEDs, barra RPM e display TM1637.
 *   Core 0 (task_sd)   : Consome fila e grava CSV no SD.
 *                        Flush automático a cada 1s.
 *   Loop (main)        : Trata botão de lap (debounce).
 *
 * Frames CAN recebidos:
 *   0x201 | DLC 8 | Acel XYZ + Gyro X  (nó IMU)
 *   0x204 | DLC 4 | Gyro YZ             (nó IMU)
 *   0x202 | DLC 8 | Hall Acel + Freio + Pressão Freio Diant/Tras (nó Analog)
 *   0x1E0 | DLC 8 | ECU ProTune PR440   (manual oficial 03/2024 R1)
 *
 * ============================================================================
 * PINOUT
 * ============================================================================
 *
 * MCP2515 (conforme Documento de Decisões Técnicas v2.0):
 *   GPIO  5 → CS
 *   GPIO 23 → MOSI
 *   GPIO 19 → MISO
 *   GPIO 18 → SCK
 *   GPIO 17 → INT
 *
 * SD Card (SPI próprio):
 *   GPIO 15 → CS
 *   GPIO 13 → MOSI (HSPI)
 *   GPIO 12 → MISO (HSPI)
 *   GPIO 14 → SCK  (HSPI)
 *
 *   NOTA: MCP2515 usa VSPI (default 18/19/23) e SD usa HSPI (14/12/13)
 *         para evitar conflito — dois barramentos SPI separados.
 *
 * Volante:
 *   GPIO 27 → Barra RPM WS2812B (8 LEDs)
 *   GPIO 26 → Display TM1637 DIO
 *   GPIO 25 → Display TM1637 CLK
 *   GPIO 16 → Botão Lap
 *
 * LEDs de aviso:
 *   GPIO 21 → LED 1 (Bateria)
 *   GPIO 22 → LED 2 (Temp Água)
 *   GPIO 32 → LED 3 (Pressão Óleo)
 *   GPIO 33 → LED 4 (Status Gravação)
 *
 * Bibliotecas necessárias:
 *   - mcp_can (coryjfowler)
 *   - FastLED
 *   - TM1637 (avishorp/TM1637 — TM1637Display)
 *
 * Taxa de log: 50 Hz | Serial: 115200 bps
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <SD.h>
#include <FastLED.h>
#include <TM1637Display.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

/* ============================================================================
 * PINOUT
 * ============================================================================ */

/* MCP2515 no VSPI (default SPI do ESP32) */
#define MCP2515_CS_PIN 5
#define MCP2515_INT_PIN 17
#define VSPI_MOSI 23
#define VSPI_MISO 19
#define VSPI_SCK 18

/* SD Card no HSPI */
#define SD_CS_PIN 15
#define HSPI_MOSI 13
#define HSPI_MISO 12
#define HSPI_SCK 14

/* Volante */
#define RPM_LED_PIN 27
#define DISPLAY_DIO_PIN 26
#define DISPLAY_CLK_PIN 25
#define BTN_LAP_PIN 16
#define BTN_PAGE_PIN 2

/* LEDs de aviso */
#define LED_VOLT_BAT 4
#define LED_TEMP_AGUA 3
#define LED_COMB 32
#define LED_PRESS_OLEO 33

/* ============================================================================
 * LIMITES DE ALERTA (THRESHOLDS)
 * ============================================================================ */
#define LIMIT_BAT_MIN 11.5f   // Volts
#define LIMIT_BAT_MAX 14.8f   // Volts (Alerta de sobrecarga)
#define LIMIT_TEMP_MAX 95.0f  // Graus Celsius
#define LIMIT_OIL_MIN 0.1f    // Bar
#define LIMIT_FUEL_MIN 0.1f   // Bar

#define LED_STATUS_REC 34

/* ============================================================================
 * BARRA WS2812B DE RPM
 * ============================================================================ */
#define RPM_LED_COUNT 8
#define RPM_MIN 1000
#define RPM_MAX 11000
#define RPM_SHIFT_WARN 9000
#define RPM_SHIFT_CRIT 10000
#define RPM_LED_BRIGHTNESS 80

/* ============================================================================
 * CAN IDs
 * ============================================================================ */
#define CAN_ID_IMU_ACCEL 0x201
#define CAN_ID_IMU_GYRO_YZ 0x204
#define CAN_ID_PEDAIS 0x202
#define CAN_ID_PROTUNE 0x1E0

#define FREIO_PSI_MAX 1100.0f  // PSI máximo a 4,5V
#define FREIO_ADC_MIN 430.0f   // Leitura crua do ADC com o pedal SOLTO (0 PSI)
#define FREIO_ADC_MAX 4095.0f  // Leitura crua máxima do ADC

/* ============================================================================
 * MATEMÁTICA DE TRANSMISSÃO E MARCHA INSTANTÂNEA (Setup 2025)
 * ============================================================================ */
#define RAIO_PNEU_M 0.2438f
#define REDUCAO_IP 1.955f
#define REDUCAO_IC 3.642857f
#define REDLINE_RPM 10700    // Corte do motor real
#define V_MIN_KMH 3.0f       // Velocidade mínima para a física fazer sentido
#define RPM_MIN_CALC 800.0f  // Evita cálculos com motor afogando

const float GEAR_RATIOS[6] = { 2.846f, 1.947f, 1.556f, 1.333f, 1.190f, 1.083f };

/* ============================================================================
 * COMUNICAÇÃO 4G LTE (SimCom A7670SA)
 * ============================================================================ */
#define TINY_GSM_MODEM_SIM7600
#define MODEM_TX_PIN 16  // Pino TX do ESP32
#define MODEM_RX_PIN 17  // Pino RX do ESP32
#define MODEM_PWR_PIN 4  // Pino K (Key)

/* Credenciais da Operadora e Servidor */
const char apn[] = "zap.vivo.com.br";
const char gprsUser[] = "vivo";
const char gprsPass[] = "vivo";

const char serverAddress[] = "ec2-34-220-119-142.us-west-2.compute.amazonaws.com";
const int serverPort = 8080;

/* Instâncias de Comunicação */
HardwareSerial SerialModem(1);
TinyGsm modem(SerialModem);
TinyGsmClient client(modem);
HttpClient http(client, serverAddress, serverPort);

static char g_currentLapName[64] = "Lap_Boot";

/* ============================================================================
 * ESTRUTURAS DE DADOS
 * ============================================================================ */

typedef struct {
  uint32_t timestamp_ms;

  /* IMU */
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;

  /* Pedais e Freio */
  float hall_acel_pct;
  float hall_freio_pct;

  /* Pressão de freio em PSI */
  float press_freio_diant_psi;
  float press_freio_tras_psi;

  /* Pressão de freio (raw ADC 0–4095; sem calibração bar definida) */
  uint16_t press_freio_diant_raw;
  uint16_t press_freio_tras_raw;

  /* ECU ProTune */
  uint16_t rpm;
  float ign_angle_deg;
  int8_t gear;
  float map_kpa;
  float iat_c;
  float engine_temp_c;
  float throttle_pct;
  float battery_v;
  float lambda1;
  float oil_press_bar;
  float fuel_press_bar;
  float vehicle_speed_kph;
  float fuel_level_l;
} LogSample_t;

typedef struct {
  uint16_t engine_rpm;
  int16_t ign_angle;
  int16_t gear_position;
  int16_t map;
  int16_t iat;
  int16_t engine_temp;
  int16_t throttle_pos;
  int16_t battery_voltage;
  int16_t lambda1;
  int16_t vehicle_speed;
  int16_t oil_pressure;
  int16_t fuel_pressure;
  uint16_t fuel_level;
} ECU_Raw_t;

typedef struct {
  int16_t accel_x, accel_y, accel_z;
  int16_t gyro_x, gyro_y, gyro_z;
} IMU_Raw_t;

typedef struct {
  uint16_t hall_acel;
  uint16_t hall_freio;
  uint16_t press_freio_diant;
  uint16_t press_freio_tras;
} Pedals_Raw_t;

typedef enum {
  CAR_STOPPED,
  CAR_MOVING
} CarState_t;

/* ============================================================================
 * CONSTANTES DE CONVERSÃO
 * ============================================================================ */

/* MPU6050: ±2g / ±250°/s */
#define ACCEL_SCALE (1.0f / 16384.0f)
#define GYRO_SCALE (1.0f / 131.0f)

/* Hall 49E */
#define HALL_ADC_MAX 4095.0f
#define HALL_PCT_MAX 99.0f

/* ECU ProTune — escalas conforme manual */
#define PT_IGN_SCALE (1.0f / 10.0f)
#define PT_MAP_SCALE (1.0f / 10.0f)
#define PT_IAT_SCALE (1.0f / 10.0f)
#define PT_TEMP_SCALE (1.0f / 10.0f)
#define PT_TP_SCALE (1.0f / 10.0f)
#define PT_BAT_SCALE (1.0f / 10.0f)
#define PT_LAMBDA_SCALE (1.0f / 1000.0f)
#define PT_OIL_SCALE (1.0f / 100.0f)
#define PT_FUEL_SCALE (1.0f / 100.0f)
#define PT_VSPD_SCALE (1.0f / 10.0f)
#define PT_FUEL_LVL_SCALE (1.0f / 100.0f)

/* ============================================================================
 * TEMPORIZAÇÃO
 * ============================================================================ */
#define SAMPLE_PERIOD_MS 20
#define SD_FLUSH_INTERVAL_MS 1000
#define LED_BLINK_INTERVAL_MS 250
#define DISPLAY_UPDATE_MS 50
#define LED_UPDATE_MS 50

#define LOG_QUEUE_SIZE 100
#define TASK_CAN_STACK 8192
#define TASK_SD_STACK 8192

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C  // Endereço 7-bit padrão (no seu módulo marca 0x78 em 8-bit, o que equivale a 0x3C)

static Adafruit_SSD1306 g_oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ============================================================================
 * VARIÁVEIS GLOBAIS
 * ============================================================================ */
static ECU_Raw_t g_ecu = { 0 };
static IMU_Raw_t g_imu = { 0 };
static Pedals_Raw_t g_pedals = { 0 };

static QueueHandle_t g_logQueue = NULL;
static SemaphoreHandle_t g_dataMutex = NULL;

static volatile bool g_recording = false;
static volatile uint32_t g_lapStartTime = 0;

static volatile uint32_t g_protuneCrcErrors = 0;
static volatile uint32_t g_protuneCrcOk = 0;

static bool g_sdAvailable = false;  //Bypass SD

static uint8_t g_displayPage = 0;
#define MAX_DISPLAY_PAGES 5

/* Instância MCP2515 (usa VSPI default do ESP32) */
static MCP_CAN CAN0(MCP2515_CS_PIN);

/* Segundo barramento SPI dedicado ao SD (HSPI) */
static SPIClass spiSD(HSPI);

/* Periféricos do volante */
static CRGB g_rpmLeds[RPM_LED_COUNT];
static TM1637Display g_display(DISPLAY_CLK_PIN, DISPLAY_DIO_PIN);

static CarState_t g_carState = CAR_STOPPED;

/* ============================================================================
 * DECODIFICAÇÃO ProTune PR440 — manual oficial 03/2024 R1
 *
 * Formato do pacote:
 *   Byte 0   : packet number
 *   Byte 1   : (dataset << 5) | (checksum & 0x1F)
 *   Bytes 2-7: 3 canais de 2 bytes (little-endian)
 *
 * Algoritmo de checksum (do próprio datasheet):
 *   1. packet   = rx[0]
 *   2. dataset  = rx[1] >> 5
 *   3. checksum = rx[1] & 0x1F
 *   4. rx[1] &= ~0x1F          ← MÁSCARA OBRIGATÓRIA
 *   5. calc = CheckSumCAN(rx) & 0x1F
 *   6. se checksum != calc, ignora
 * ============================================================================ */

static uint8_t CheckSumCAN(const uint8_t *ss) {
  uint8_t size = 8;
  uint8_t checksum = 0xFF;
  while (size--) {
    checksum += *ss++;
  }
  return checksum;
}

static int calcularBarraRPM(float rpm_atual, float rpm_max, int n, float rpm_min) {
  if (rpm_max <= rpm_min) return 0;

  float frac = (rpm_atual - rpm_min) / (rpm_max - rpm_min);

  /* Limita a fração entre 0.0 e 1.0 (Clamp) */
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;

  return round(frac * n);
}

static int8_t calcularMarchaInstantanea(float v_kmh, uint16_t rpm_motor) {
  if (v_kmh < V_MIN_KMH || rpm_motor < RPM_MIN_CALC) {
    return 0;  // Indeterminado (parado ou debreando no grid)
  }

  /* RPM da roda: v(km/h) / 3.6 = v(m/s). 
     * rpm_roda = v(m/s) * 60 / (2 * PI * RAIO) */
  float rpm_roda = (v_kmh / 3.6f) * 60.0f / (2.0f * PI * RAIO_PNEU_M);

  if (rpm_roda <= 0.1f) return 0;

  float ir_medido = (float)rpm_motor / rpm_roda;

  int8_t melhor_marcha = 0;
  float menor_erro = 999.0f;
  const float tolerancia = 0.15f;  // 15% de margem (cobre variação de pneu e pequeno wheelspin)

  for (int i = 0; i < 6; i++) {
    float ir_teorico = REDUCAO_IP * GEAR_RATIOS[i] * REDUCAO_IC;
    float erro = abs(ir_teorico - ir_medido) / ir_teorico;

    if (erro < menor_erro) {
      menor_erro = erro;
      melhor_marcha = i + 1;  // Marchas de 1 a 6
    }
  }

  if (menor_erro <= tolerancia) {
    return melhor_marcha;
  }

  /* Se o erro for > 15%, o pneu está patinando muito forte ou travado no freio */
  return 0;
}

/* ============================================================================
 * EVENTOS (GATILHOS)
 * ============================================================================ */

static void onCarStartMoving() {
  Serial.println("[MOVIMENTO] Carro entrou em MOVIMENTO!");

  // Gera um nome único para a Lap baseado no tempo de boot (Já que não temos bateria de RTC)
  // Exemplo: "Lap_Route_14502"
  snprintf(g_currentLapName, sizeof(g_currentLapName), "Lap_Route_%lu", millis());

  Serial.printf("[4G] Nova Lap criada: %s\n", g_currentLapName);
}

/* Gatilho disparado UMA VEZ quando o carro para */
static void onCarStopMoving() {
  Serial.println("[MOVIMENTO] Carro PAROU!");
}

/* ============================================================================
 * EVENTOS DE ALERTA (GATILHOS)
 * ============================================================================ */

/* --- BATERIA --- */
static void onBatWarningEnter() {
  Serial.println("[ALERTA] Bateria fora do ideal! Ativando LED.");
  digitalWrite(LED_VOLT_BAT, HIGH);
  // Aqui você também poderia mandar um JSON via 4G avisando os boxes
}
static void onBatWarningExit() {
  Serial.println("[ALERTA] Bateria Normalizada. Desativando LED.");
  digitalWrite(LED_VOLT_BAT, LOW);
}

/* --- TEMPERATURA DA ÁGUA --- */
static void onTempWarningEnter() {
  Serial.println("[ALERTA] Superaquecimento da Água! Ativando LED.");
  digitalWrite(LED_TEMP_AGUA, HIGH);
}
static void onTempWarningExit() {
  Serial.println("[ALERTA] Temperatura Normalizada. Desativando LED.");
  digitalWrite(LED_TEMP_AGUA, LOW);
}

/* --- PRESSÃO DE ÓLEO --- */
static void onOilWarningEnter() {
  Serial.println("[ALERTA CRÍTICO] Baixa Pressão de Óleo! Ativando LED.");
  digitalWrite(LED_PRESS_OLEO, HIGH);
}
static void onOilWarningExit() {
  Serial.println("[ALERTA] Pressão de Óleo Normalizada. Desativando LED.");
  digitalWrite(LED_PRESS_OLEO, LOW);
}

/* --- COMBUSTÍVEL --- */
static void onFuelWarningEnter() {
  Serial.println("[ALERTA] Combustível na Reserva! Ativando LED.");
  digitalWrite(LED_COMB, HIGH);
}
static void onFuelWarningExit() {
  Serial.println("[ALERTA] Combustível Normalizado (Reabastecido). Desativando LED.");
  digitalWrite(LED_COMB, LOW);
}

/* ============================================================================
 * CORREÇÃO: Funções ajustadas para ler BIG-ENDIAN (MSB first)
 * ============================================================================ */
static inline int16_t pt_i16(const uint8_t *data, uint8_t offset) {
  /* Desloca o primeiro byte (MSB) 8 vezes e soma com o segundo (LSB) */
  return (int16_t)((data[offset] << 8) | data[offset + 1]);
}

static inline uint16_t pt_u16(const uint8_t *data, uint8_t offset) {
  return (uint16_t)((data[offset] << 8) | data[offset + 1]);
}

/* ============================================================================
 * DECODIFICAÇÃO PROTUNE (Conforme Manual Oficial em C/C++)
 * ============================================================================ */
static void decodeProTunePacket(const uint8_t *data_in) {
  uint8_t rx[8];
  memcpy(rx, data_in, 8);

  uint8_t packet = rx[0];
  uint8_t dataset = rx[1] >> 5;  // usa somente os 3 primeiros bits (dataset)

  if (dataset > 4) return;  // qualquer valor acima ou igual a 4 é ignorado

  uint8_t checksum = rx[1] & 0x1F;  // usa apenas os 5 últimos bits (checkSum)

  rx[1] &= ~0x1F;  // Mascara o checksum do pacote atual

  uint8_t checkCalc = CheckSumCAN(rx) & 0x1F;  // Calcula checksum e testa se está oK

  if (checksum != checkCalc) {
    g_protuneCrcErrors++;
    return;
  }
  g_protuneCrcOk++;

  /* O Switch vai direto no Packet, sem restrição de Dataset */
  switch (packet) {
    case 2:
      g_ecu.engine_rpm = pt_u16(rx, 2);
      g_ecu.ign_angle = pt_i16(rx, 4);
      break;

    case 4:
      g_ecu.gear_position = pt_i16(rx, 4);
      break;

    case 5:
      g_ecu.map = pt_i16(rx, 2);
      g_ecu.iat = pt_i16(rx, 4);
      g_ecu.engine_temp = pt_i16(rx, 6);
      break;

    case 6:
      g_ecu.throttle_pos = pt_i16(rx, 2);
      g_ecu.battery_voltage = pt_i16(rx, 4);
      g_ecu.lambda1 = pt_i16(rx, 6);
      break;

    case 8:
      g_ecu.vehicle_speed = pt_i16(rx, 4);
      break;

    case 15:
      g_ecu.oil_pressure = pt_i16(rx, 2);
      g_ecu.fuel_pressure = pt_i16(rx, 4);
      break;

    case 79:
      g_ecu.fuel_level = pt_u16(rx, 6);
      break;

    default:
      break;
  }
}

/* ============================================================================
 * PROCESSAMENTO DE FRAMES CAN
 * ============================================================================ */

static void processCANMessage(uint32_t id, const uint8_t *data, uint8_t len) {
  xSemaphoreTake(g_dataMutex, portMAX_DELAY);

  switch (id) {
    case CAN_ID_IMU_ACCEL: /* Acel XYZ + Gyro X */
      if (len >= 8) {
        g_imu.accel_x = (int16_t)((data[0] << 8) | data[1]);
        g_imu.accel_y = (int16_t)((data[2] << 8) | data[3]);
        g_imu.accel_z = (int16_t)((data[4] << 8) | data[5]);
        g_imu.gyro_x = (int16_t)((data[6] << 8) | data[7]);
      }
      break;

    case CAN_ID_IMU_GYRO_YZ:
      if (len >= 4) {
        g_imu.gyro_y = (int16_t)((data[0] << 8) | data[1]);
        g_imu.gyro_z = (int16_t)((data[2] << 8) | data[3]);
      }
      break;

    case CAN_ID_PEDAIS:
      if (len >= 8) {
        g_pedals.hall_acel = (uint16_t)((data[0] << 8) | data[1]);
        g_pedals.hall_freio = (uint16_t)((data[2] << 8) | data[3]);
        g_pedals.press_freio_diant = (uint16_t)((data[4] << 8) | data[5]);
        g_pedals.press_freio_tras = (uint16_t)((data[6] << 8) | data[7]);
      }
      break;

    case CAN_ID_PROTUNE:
      if (len >= 8) {
        decodeProTunePacket(data);
      }
      break;
  }

  xSemaphoreGive(g_dataMutex);
}

static void buildSnapshot(LogSample_t *s) {
  xSemaphoreTake(g_dataMutex, portMAX_DELAY);

  s->timestamp_ms = millis() - g_lapStartTime;

  s->ax_g = g_imu.accel_x * ACCEL_SCALE;
  s->ay_g = g_imu.accel_y * ACCEL_SCALE;
  s->az_g = g_imu.accel_z * ACCEL_SCALE;
  s->gx_dps = g_imu.gyro_x * GYRO_SCALE;
  s->gy_dps = g_imu.gyro_y * GYRO_SCALE;
  s->gz_dps = g_imu.gyro_z * GYRO_SCALE;

  s->hall_acel_pct = (g_pedals.hall_acel * HALL_PCT_MAX) / HALL_ADC_MAX;
  s->hall_freio_pct = (g_pedals.hall_freio * HALL_PCT_MAX) / HALL_ADC_MAX;

  s->press_freio_diant_raw = g_pedals.press_freio_diant;
  s->press_freio_tras_raw = g_pedals.press_freio_tras;

  /* Leitura bruta e trava em zero (evita PSI negativo se o ADC flutuar) */
  float raw_diant = (float)g_pedals.press_freio_diant;
  float raw_tras = (float)g_pedals.press_freio_tras;

  if (raw_diant < FREIO_ADC_MIN) raw_diant = FREIO_ADC_MIN;
  if (raw_tras < FREIO_ADC_MIN) raw_tras = FREIO_ADC_MIN;

  /* Conversão final para PSI */
  s->press_freio_diant_psi = ((raw_diant - FREIO_ADC_MIN) / (FREIO_ADC_MAX - FREIO_ADC_MIN)) * FREIO_PSI_MAX;
  s->press_freio_tras_psi = ((raw_tras - FREIO_ADC_MIN) / (FREIO_ADC_MAX - FREIO_ADC_MIN)) * FREIO_PSI_MAX;

  s->rpm = g_ecu.engine_rpm;
  s->ign_angle_deg = g_ecu.ign_angle * PT_IGN_SCALE;
  s->gear = (int8_t)g_ecu.gear_position;
  s->map_kpa = g_ecu.map * PT_MAP_SCALE;
  s->iat_c = g_ecu.iat * PT_IAT_SCALE;
  s->engine_temp_c = g_ecu.engine_temp * PT_TEMP_SCALE;
  s->throttle_pct = g_ecu.throttle_pos * PT_TP_SCALE;
  s->battery_v = g_ecu.battery_voltage * PT_BAT_SCALE;
  s->lambda1 = g_ecu.lambda1 * PT_LAMBDA_SCALE;
  s->oil_press_bar = g_ecu.oil_pressure * PT_OIL_SCALE;
  s->fuel_press_bar = g_ecu.fuel_pressure * PT_FUEL_SCALE;
  s->vehicle_speed_kph = g_ecu.vehicle_speed * PT_VSPD_SCALE;
  s->fuel_level_l = g_ecu.fuel_level * PT_FUEL_LVL_SCALE;

  xSemaphoreGive(g_dataMutex);
}

/* ============================================================================
 * INICIALIZAÇÃO DO MCP2515
 *
 * Assume cristal de 8 MHz no módulo. Se for 16 MHz, trocar MCP_8MHZ por
 * MCP_16MHZ. Verifique o cristal soldado no módulo físico.
 * ============================================================================ */

static bool setup_mcp2515(void) {
  /* Inicializa VSPI explicitamente com pinos configuráveis.
     * O MCP_CAN usa SPI global por default, então precisa estar iniciado. */
  SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, MCP2515_CS_PIN);

  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);

  const uint8_t maxRetries = 5;
  for (uint8_t attempt = 1; attempt <= maxRetries; attempt++) {
    Serial.printf("[MCP2515] Tentativa %d/%d... ", attempt, maxRetries);

    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
      Serial.println("OK");

      /* Modo normal — necessário para enviar ACK aos transmissores */
      CAN0.setMode(MCP_NORMAL);
      return true;
    }

    Serial.println("falhou");
    delay(200);
  }

  Serial.println("[MCP2515] ERRO FATAL — verifique fiação SPI e cristal");
  return false;
}

/* ============================================================================
 * LEDS DE ALERTA
 * ============================================================================ */

static void updateAlertLEDs(void) {
  static uint32_t lastBlink[3] = { 0, 0, 0 };
  const uint32_t now = millis();

  float bat_v, temp_c, oil_bar;

  xSemaphoreTake(g_dataMutex, portMAX_DELAY);
  bat_v = g_ecu.battery_voltage * PT_BAT_SCALE;
  temp_c = g_ecu.engine_temp * PT_TEMP_SCALE;
  oil_bar = g_ecu.oil_pressure * PT_OIL_SCALE;
  xSemaphoreGive(g_dataMutex);

  if (bat_v > 14.0f) {
    if (now - lastBlink[0] >= LED_BLINK_INTERVAL_MS) {
      digitalWrite(LED_VOLT_BAT, !digitalRead(LED_VOLT_BAT));
      lastBlink[0] = now;
    }
  } else if (bat_v < 10.0f) {
    digitalWrite(LED_VOLT_BAT, HIGH);
  } else {
    digitalWrite(LED_VOLT_BAT, LOW);
  }

  if (temp_c > 95.0f) {
    if (now - lastBlink[1] >= LED_BLINK_INTERVAL_MS) {
      digitalWrite(LED_TEMP_AGUA, !digitalRead(LED_TEMP_AGUA));
      lastBlink[1] = now;
    }
  } else {
    digitalWrite(LED_TEMP_AGUA, LOW);
  }

  if (oil_bar < 2.0f) {
    if (now - lastBlink[2] >= LED_BLINK_INTERVAL_MS) {
      digitalWrite(LED_PRESS_OLEO, !digitalRead(LED_PRESS_OLEO));
      lastBlink[2] = now;
    }
  } else if (oil_bar < 3.0f) {
    digitalWrite(LED_PRESS_OLEO, HIGH);
  } else {
    digitalWrite(LED_PRESS_OLEO, LOW);
  }
}

/* ============================================================================
 * BARRA WS2812B DE RPM
 * ============================================================================ */

static void updateRPMBar(uint16_t rpm, int8_t gear) {
  static uint32_t lastBlink = 0;
  static bool blinkState = false;
  const uint32_t now = millis();

  /* Limites baseados no Shift Light (Corte em 10700 RPM) */
  uint16_t shift_rpm = REDLINE_RPM;
  uint16_t crit_rpm = shift_rpm - 300;
  uint16_t min_rpm = shift_rpm * 0.3f;

  /* Alerta Crítico de Troca (Shift Light estourando) */
  if (rpm >= crit_rpm) {
    if (now - lastBlink >= 60) {
      blinkState = !blinkState;
      lastBlink = now;
    }
    /* Pisca tudo em Azul e Vermelho alternado para chamar muita atenção */
    CRGB color = blinkState ? CRGB::Blue : CRGB::Red;
    for (int i = 0; i < RPM_LED_COUNT; i++) g_rpmLeds[i] = color;
    FastLED.show();
    return;
  }

  /* Quantos LEDs acender com base na matemática? */
  int ledsOn = calcularBarraRPM((float)rpm, (float)shift_rpm, RPM_LED_COUNT, (float)min_rpm);

  for (int i = 0; i < RPM_LED_COUNT; i++) {
    if (i < ledsOn) {
      /* Mapeia a cor: LED 0 = Verde (96) -----> LED Final = Vermelho (0) 
             * Fica progressivamente mais vermelho quanto mais próximo do limite */
      uint8_t hue = map(i, 0, RPM_LED_COUNT - 1, 96, 0);

      /* Seta a cor usando o padrão HSV (Cor, Saturação máxima, Brilho máximo) */
      g_rpmLeds[i] = CHSV(hue, 255, 255);
    } else {
      g_rpmLeds[i] = CRGB::Black;
    }
  }
  FastLED.show();
}

/* ============================================================================
 * DISPLAY TM1637 — 4 dígitos 7-segmentos
 *
 * Lógica:
 *   - Se RPM >= 1000: mostra RPM (4 dígitos)
 *   - Se RPM < 1000 mas > 0: mostra RPM com zeros à esquerda
 *   - Se motor parado (RPM = 0): mostra marcha atual (ex: "G 1", "G n")
 * ============================================================================ */

static void updateDisplay(uint16_t rpm, int8_t gear, float temp_c, float bat_v, float lambda) {
  /* Segmentos customizados úteis para o TM1637 (a=0x01, b=0x02, ..., g=0x40) */
  const uint8_t CHAR_G = 0x7D;  // 'G' (Marcha)
  const uint8_t CHAR_C = 0x39;  // 'C' (Celsius)
  const uint8_t CHAR_U = 0x3E;  // 'U' (Volts)
  const uint8_t CHAR_L = 0x38;  // 'L' (Lambda)
  const uint8_t CHAR_N = 0x54;  // 'n' (Neutro)
  const uint8_t CHAR_R = 0x50;  // 'r' (Re)

  uint8_t segs[4] = { 0, 0, 0, 0 };

  switch (g_displayPage) {
    case 0: /* Página 0: Automática (RPM ou Marcha) */
      if (rpm >= 1000) {
        g_display.showNumberDec(rpm, false);
      } else if (rpm > 0) {
        g_display.showNumberDec(rpm, true);
      } else {
        /* Motor parado: exibe a marcha atual */
        segs[2] = 0;  // Garante que o dígito da dezena está apagado

        if (gear == 0) {
          segs[3] = CHAR_N;  // Ponto morto: mostra apenas 'n'
        } else if (gear > 0 && gear <= 9) {
          segs[3] = g_display.encodeDigit(gear);  // Mostra o número da marcha (1 a 6)
        } else if (gear < 0) {
          segs[3] = CHAR_R;  // Ré: mostra 'r'
        }

        g_display.setSegments(segs);
      }
      break;

    case 1: /* Página 1: Temperatura (C XX) */
      segs[0] = CHAR_C;
      segs[1] = 0;                                        // Espaço
      g_display.setSegments(segs, 3, 0);                  // Desenha os 2 primeiros
      g_display.showNumberDec((int)temp_c, false, 3, 1);  // Desenha os 2 últimos
      break;

    case 2: /* Página 2: Bateria (U XX.X) */
      {
        segs[0] = CHAR_U;
        g_display.setSegments(segs, 1, 0);

        // Ex: 13.5V -> 135. Mostramos 3 dígitos.
        int bat_display = (int)(bat_v * 10.0f);
        // O true habilita zeros à esquerda se a bateria cair para < 10V.
        // O 0x40 (ou 0x80 dependendo do seu módulo) liga os dois pontos do display.
        g_display.showNumberDecEx(bat_display, 0x40, true, 3, 1);
        break;
      }

    case 3: /* Página 3: Lambda (L .XXX) */
      {
        segs[0] = CHAR_L;
        g_display.setSegments(segs, 1, 0);
        int lam_display = (int)(lambda * 1000.0f);
        g_display.showNumberDec(lam_display, true, 3, 1);
        break;
      }

    case 4: /* Página 4: Marcha Fixa */
      {
        segs[0] = 0;  // Apagado
        segs[1] = 0;  // Apagado
        segs[2] = 0;  // <-- Aqui estava o CHAR_G que parecia um '6'. Agora fica apagado!

        if (gear == 0) {
          segs[3] = CHAR_N;  // 'n' para ponto morto
        } else if (gear > 0 && gear <= 9) {
          segs[3] = g_display.encodeDigit(gear);
        } else if (gear < 0) {
          segs[3] = CHAR_R;  // 'r' para ré
        }

        g_display.setSegments(segs);
        break;
      }
  }
}

static void updateBrakeBiasDisplay() {
  static float last_diant = -999.0f;
  static float last_tras = -999.0f;
  const float NOISE_DEADBAND = 2.0f;  // Atualiza a tela só se a pressão variar > 2 PSI

  uint16_t raw_diant, raw_tras;

  /* Coleta segura das variáveis brutas */
  xSemaphoreTake(g_dataMutex, portMAX_DELAY);
  raw_diant = g_pedals.press_freio_diant;
  raw_tras = g_pedals.press_freio_tras;
  xSemaphoreGive(g_dataMutex);

  /* Tratamento do limite mínimo do ADC */
  float f_diant = (float)raw_diant;
  float f_tras = (float)raw_tras;
  if (f_diant < FREIO_ADC_MIN) f_diant = FREIO_ADC_MIN;
  if (f_tras < FREIO_ADC_MIN) f_tras = FREIO_ADC_MIN;

  /* Conversão final para PSI */
  float psi_diant = ((f_diant - FREIO_ADC_MIN) / (FREIO_ADC_MAX - FREIO_ADC_MIN)) * FREIO_PSI_MAX;
  float psi_tras = ((f_tras - FREIO_ADC_MIN) / (FREIO_ADC_MAX - FREIO_ADC_MIN)) * FREIO_PSI_MAX;

  /* Verifica se houve mudança real (histerese) para poupar processamento I2C */
  if (abs(psi_diant - last_diant) > NOISE_DEADBAND || abs(psi_tras - last_tras) > NOISE_DEADBAND) {
    last_diant = psi_diant;
    last_tras = psi_tras;

    float total_psi = psi_diant + psi_tras;
    float pct_diant = 0.0f;
    float pct_tras = 0.0f;

    /* Evita divisão por zero e exibe porcentagem real apenas se houver pressão mínima no sistema */
    if (total_psi > 10.0f) {
      pct_diant = (psi_diant / total_psi) * 100.0f;
      pct_tras = (psi_tras / total_psi) * 100.0f;
    } else {
      pct_diant = 50.0f;  // Valor neutro quando os pedais estão soltos
      pct_tras = 50.0f;
    }

    /* Rotina de desenho na tela OLED */
    g_oled.clearDisplay();

    g_oled.setTextSize(1);
    g_oled.setCursor(0, 0);
    g_oled.println("BRAKE BIAS (BALANCO)");

    g_oled.setTextSize(2);  // Texto grande para facilitar leitura com o carro em movimento
    g_oled.setCursor(0, 20);
    g_oled.printf("DIANT: %.0f%%\n", pct_diant);

    g_oled.setCursor(0, 45);
    g_oled.printf("TRAS : %.0f%%\n", pct_tras);

    g_oled.display();
  }
}

/* ============================================================================
 * SD CARD
 * ============================================================================ */

static File g_dataFile;
static uint32_t g_lastFlushTime = 0;

static void createNewLogFile(void) {
  if (!g_sdAvailable) return;

  char filename[32];
  int fileNum = 0;
  do {
    snprintf(filename, sizeof(filename), "/lap_%03d.csv", fileNum++);
  } while (SD.exists(filename) && fileNum < 1000);

  g_dataFile = SD.open(filename, FILE_WRITE);

  if (!g_dataFile) {
    Serial.println("[SD] ERRO ao criar arquivo");
    return;
  }

  g_dataFile.println(
    "timestamp_ms,"
    "accel_x_g,accel_y_g,accel_z_g,"
    "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
    "hall_acel_pct,hall_freio_pct,"
    "press_freio_diant_raw,press_freio_tras_raw,"
    "rpm,ign_angle_deg,gear,"
    "map_kpa,iat_c,engine_temp_c,throttle_pct,"
    "battery_v,lambda1,"
    "oil_press_bar,fuel_press_bar,"
    "vehicle_speed_kph,fuel_level_l");
  g_dataFile.flush();
  g_lastFlushTime = millis();

  Serial.printf("[SD] Arquivo criado: %s\n", filename);
}

static void writeSampleToSD(const LogSample_t *s) {
  if (!g_sdAvailable || !g_dataFile) return;

  char buf[352];
  int n = snprintf(buf, sizeof(buf),
                   "%lu,"
                   "%.4f,%.4f,%.4f,"
                   "%.2f,%.2f,%.2f,"
                   "%.1f,%.1f,"
                   "%.1f,%.1f,"  // <-- Aqui as variaveis float de PSI
                   "%u,%.1f,%d,"
                   "%.1f,%.1f,%.1f,%.1f,"
                   "%.1f,%.3f,"
                   "%.2f,%.2f,"
                   "%.1f,%.2f\n",
                   (unsigned long)s->timestamp_ms,
                   s->ax_g, s->ay_g, s->az_g,
                   s->gx_dps, s->gy_dps, s->gz_dps,
                   s->hall_acel_pct, s->hall_freio_pct,
                   s->press_freio_diant_psi, s->press_freio_tras_psi,  // <-- Trocado os RAWs velhos por PSI
                   s->rpm, s->ign_angle_deg, s->gear,
                   s->map_kpa, s->iat_c, s->engine_temp_c, s->throttle_pct,
                   s->battery_v, s->lambda1,
                   s->oil_press_bar, s->fuel_press_bar,
                   s->vehicle_speed_kph, s->fuel_level_l);

  if (n > 0) {
    g_dataFile.write((const uint8_t *)buf, n);
  }

  uint32_t now = millis();
  if (now - g_lastFlushTime >= SD_FLUSH_INTERVAL_MS) {
    g_dataFile.flush();
    g_lastFlushTime = now;
  }
}

static void handlePageButton(void) {
  static bool lastState = HIGH;
  static uint32_t lastChange = 0;
  const uint32_t DEBOUNCE_MS = 50;

  bool current = digitalRead(BTN_PAGE_PIN);
  uint32_t now = millis();

  if (current != lastState && (now - lastChange) >= DEBOUNCE_MS) {
    lastChange = now;

    if (current == LOW) {  // Botão pressionado
      g_displayPage++;
      if (g_displayPage >= MAX_DISPLAY_PAGES) {
        g_displayPage = 0;  // Volta para a primeira página
      }
      Serial.printf("[DISPLAY] Pagina alterada para: %d\n", g_displayPage);
    }
    lastState = current;
  }
}

/* ============================================================================
 * BOTÃO DE LAP
 * ============================================================================ */

static void handleLapButton(void) {
  static bool lastState = HIGH;
  static uint32_t lastChange = 0;
  const uint32_t DEBOUNCE_MS = 50;

  bool current = digitalRead(BTN_LAP_PIN);
  uint32_t now = millis();

  if (current != lastState && (now - lastChange) >= DEBOUNCE_MS) {
    lastChange = now;
    Serial.println("Botao <<<");
    if (current == LOW) {
      if (!g_recording) {
        g_lapStartTime = millis();

        if (g_sdAvailable) {
          createNewLogFile();
        } else {
          Serial.println("[REC] Gravacao simulada (Bypass do SD)");
        }

        g_recording = true;
        digitalWrite(LED_STATUS_REC, HIGH);
        Serial.println("[REC] >>> GRAVACAO INICIADA <<<");
      } else {
        g_recording = false;
        digitalWrite(LED_STATUS_REC, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));

        if (g_sdAvailable && g_dataFile) {  // Proteção extra no fechamento
          g_dataFile.flush();
          g_dataFile.close();
        }
        Serial.println("[REC] >>> GRAVACAO PARADA <<<");
      }
    }
    lastState = current;
  }
}

/* Função que varre a velocidade em tempo real com Histerese */
static void handleMotionState() {
  static uint32_t timeAboveStartThresh = 0;
  static uint32_t timeBelowStopThresh = 0;

  /* Configurações de Histerese */
  const float SPEED_START_KMH = 5.0f;    // Acima de 5km/h para considerar andando
  const float SPEED_STOP_KMH = 2.0f;     // Abaixo de 2km/h para considerar parado
  const uint32_t TIME_TO_CONFIRM = 500;  // 500ms de confirmação constante

  /* Leitura segura da velocidade do mutex global */
  float v_kmh;
  xSemaphoreTake(g_dataMutex, portMAX_DELAY);
  v_kmh = g_ecu.vehicle_speed * PT_VSPD_SCALE;
  xSemaphoreGive(g_dataMutex);

  uint32_t now = millis();

  if (g_carState == CAR_STOPPED) {
    if (v_kmh >= SPEED_START_KMH) {
      if (timeAboveStartThresh == 0) {
        timeAboveStartThresh = now;  // Inicia o cronômetro
      } else if (now - timeAboveStartThresh >= TIME_TO_CONFIRM) {
        g_carState = CAR_MOVING;  // Confirma o movimento
        onCarStartMoving();       // Dispara o evento
        timeBelowStopThresh = 0;  // Reseta o cronômetro de parada
      }
    } else {
      timeAboveStartThresh = 0;  // Reseta se a velocidade cair antes do tempo
    }
  } else if (g_carState == CAR_MOVING) {
    if (v_kmh <= SPEED_STOP_KMH) {
      if (timeBelowStopThresh == 0) {
        timeBelowStopThresh = now;  // Inicia o cronômetro de parada
      } else if (now - timeBelowStopThresh >= TIME_TO_CONFIRM) {
        g_carState = CAR_STOPPED;  // Confirma a parada
        onCarStopMoving();         // Dispara o evento
        timeAboveStartThresh = 0;  // Reseta o cronômetro de movimento
      }
    } else {
      timeBelowStopThresh = 0;  // Reseta se o carro voltar a acelerar antes do tempo
    }
  }
}

/* ============================================================================
 * MÉTODOS AUXILIARES 4G
 * ============================================================================ */

/* Sequência de Boot Física do A7670SA (Pino K) */
static void powerOnModem() {
  pinMode(MODEM_PWR_PIN, OUTPUT);
  Serial.println("[4G] Ligando modem...");

  // Sequência exata do Datasheet para acordar o A7670
  digitalWrite(MODEM_PWR_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(MODEM_PWR_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(1000));
  digitalWrite(MODEM_PWR_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(6000));  // Tempo para o OS do modem iniciar
}

/* Conecta na Rede Móvel e APN */
static bool initGSM() {
  Serial.println("[4G] Iniciando comunicacao AT...");
  SerialModem.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  if (!modem.restart()) {
    Serial.println("[4G] ERRO: Modem nao respondeu aos comandos AT.");
    return false;
  }

  Serial.print("[4G] Aguardando sinal de rede...");
  if (!modem.waitForNetwork(60000L)) {
    Serial.println(" FALHOU!");
    return false;
  }
  Serial.println(" OK!");

  Serial.print("[4G] Conectando na APN (PDP Context)...");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" FALHOU!");
    return false;
  }
  Serial.println(" CONECTADO COM SUCESSO!");
  return true;
}

/* Dispara o HTTP POST com o JSON formatado */
static void sendTelemetryPost(const char *jsonPayload) {
  if (!modem.isGprsConnected()) {
    Serial.println("[4G] Sem conexao GPRS. Reconectando...");
    modem.gprsConnect(apn, gprsUser, gprsPass);
    return;
  }

  Serial.println("[4G] Enviando pacote ao Servidor Spring...");

  http.beginRequest();
  http.post("/main/parameter/register");  // Endpoint de destino
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", strlen(jsonPayload));
  http.beginBody();
  http.print(jsonPayload);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  String response = http.responseBody();

  Serial.printf("[4G] Resposta HTTP: %d\n", statusCode);
}

/* ============================================================================
 * TASK CORE 1 — MCP2515 e LOG
 * ============================================================================ */
static void task_can(void *arg) {
  (void)arg;
  Serial.printf("[TASK] task_can no core %d\n", xPortGetCoreID());

  uint32_t lastSample = 0;
  uint32_t lastStatsTime = millis();
  static uint32_t lastDebugTime = 0;

  while (1) {
    /* 1. Drena o buffer do MCP2515 checando diretamente o chip via SPI (ignora o pino INT) */
    unsigned long canId = 0;
    uint8_t len = 0;
    uint8_t buf[8];

    // Tenta ler o buffer 0 ou buffer 1 diretamente
    while (CAN0.readMsgBuf(&canId, &len, buf) == CAN_OK) {
      if (!(canId & 0x80000000UL) && !(canId & 0x40000000UL)) {
        uint32_t id = canId & 0x7FF;
        processCANMessage(id, buf, len);
      }
    }

    uint32_t now = millis();

    /* 2. Enfileira sample a 50 Hz para o SD */
    if (g_recording && (now - lastSample >= SAMPLE_PERIOD_MS)) {
      lastSample = now;
      LogSample_t sample;
      buildSnapshot(&sample);
      if (xQueueSend(g_logQueue, &sample, 0) != pdTRUE) {
        // Buffer cheio
      }
    }

    /* 3. Imprime a Telemetria a cada 1s */
    if (now - lastDebugTime >= 1000) {
      lastDebugTime = now;

      xSemaphoreTake(g_dataMutex, portMAX_DELAY);
      int16_t raw_ax = g_imu.accel_x;
      int16_t raw_ay = g_imu.accel_y;
      int16_t raw_az = g_imu.accel_z;
      int16_t raw_gx = g_imu.gyro_x;
      int16_t raw_gy = g_imu.gyro_y;
      int16_t raw_gz = g_imu.gyro_z;
      uint16_t p_freio_diant = g_pedals.press_freio_diant;
      uint16_t p_freio_tras = g_pedals.press_freio_tras;
      uint16_t hall_freio = g_pedals.hall_freio;

      uint16_t ecu_rpm = g_ecu.engine_rpm;
      int16_t ecu_temp = g_ecu.engine_temp;
      int16_t ecu_tps = g_ecu.throttle_pos;
      int16_t ecu_bat = g_ecu.battery_voltage;
      int16_t ecu_oil = g_ecu.oil_pressure;
      int16_t ecu_fuel = g_ecu.fuel_pressure;
      int16_t ecu_lambda = g_ecu.lambda1;
      int16_t ecu_vspd = g_ecu.vehicle_speed;
      int8_t ecu_gear = (int8_t)g_ecu.gear_position;
      xSemaphoreGive(g_dataMutex);

      /* Conversões do Chassi */
      float ax_g = raw_ax * ACCEL_SCALE;
      float ay_g = raw_ay * ACCEL_SCALE;
      float az_g = raw_az * ACCEL_SCALE;
      float gx = raw_gx * GYRO_SCALE;
      float gy = raw_gy * GYRO_SCALE;
      float gz = raw_gz * GYRO_SCALE;
      float hall_freio_pct = (hall_freio * HALL_PCT_MAX) / HALL_ADC_MAX;

      /* Freio em PSI */
      float raw_diant = (float)p_freio_diant;
      float raw_tras = (float)p_freio_tras;
      if (raw_diant < FREIO_ADC_MIN) raw_diant = FREIO_ADC_MIN;
      if (raw_tras < FREIO_ADC_MIN) raw_tras = FREIO_ADC_MIN;

      float psi_diant = ((raw_diant - FREIO_ADC_MIN) / (FREIO_ADC_MAX - FREIO_ADC_MIN)) * FREIO_PSI_MAX;
      float psi_tras = ((raw_tras - FREIO_ADC_MIN) / (FREIO_ADC_MAX - FREIO_ADC_MIN)) * FREIO_PSI_MAX;

      /* Conversões do Motor */
      float temp_c = ecu_temp * PT_TEMP_SCALE;
      float tps_pct = ecu_tps * PT_TP_SCALE;
      float bat_v = ecu_bat * PT_BAT_SCALE;
      float oil_bar = ecu_oil * PT_OIL_SCALE;
      float fuel = ecu_fuel * PT_FUEL_SCALE;
      float lambda = ecu_lambda * PT_LAMBDA_SCALE;
      float v_kmh = ecu_vspd * PT_VSPD_SCALE;

      int8_t fast_gear = calcularMarchaInstantanea(v_kmh, ecu_rpm);
      int8_t calc_gear = (fast_gear != 0) ? fast_gear : ecu_gear;

      /* Print */
      Serial.printf("[TELEMETRIA-CHASSI] ACC(g): X=%.2f Y=%.2f Z=%.2f | GYRO: X=%.2f Y=%.2f Z=%.2f | FREIO(psi): D=%.1f T=%.1f | FREIO(raw): D=%u T=%u | HALL_FR: %.1f%%\n",
                    ax_g, ay_g, az_g, gx, gy, gz, psi_diant, psi_tras, p_freio_diant, p_freio_tras, hall_freio_pct);

      // Adicionado "MARCHA: X (ECU: Y)" para você ver a marcha calculada e a marcha real que a ProTune mandou no pacote
      Serial.printf("[TELEMETRIA-MOTOR]  RPM: %u | MARCHA: %d (ECU:%d) | Vm: %.1fKmh | TEMP: %.1fC | TPS: %.1f%% | BAT: %.1fV | OLEO: %.2fbar | COMB: %.2fbar | LAMBDA: %.3f\n",
                    ecu_rpm, calc_gear, ecu_gear, v_kmh, temp_c, tps_pct, bat_v, oil_bar, fuel, lambda);
    }

    /* 4. Estatísticas de CRC a cada 5s */
    if (now - lastStatsTime >= 5000) {
      lastStatsTime = now;
      if (g_protuneCrcOk + g_protuneCrcErrors > 0) {
        // Serial.printf("[ProTune] OK=%lu  CRCerr=%lu\n",
        //               g_protuneCrcOk, g_protuneCrcErrors);
      }
    }

    /* Yield ultracurto para não estourar Watchdog */
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* ============================================================================
 * TASK CORE 0 — Escrita no SD
 * ============================================================================ */

static void task_sd(void *arg) {
  (void)arg;
  Serial.printf("[TASK] task_sd no core %d\n", xPortGetCoreID());

  LogSample_t sample;
  while (1) {
    if (xQueueReceive(g_logQueue, &sample, portMAX_DELAY) == pdTRUE) {
      if (g_recording) {
        writeSampleToSD(&sample);
      }
    }
  }
}

/* ============================================================================
 * TASK CORE 0 — Interface (Painel, LEDs e Display)
 * ============================================================================ */
static void task_hmi(void *arg) {
  (void)arg;
  Serial.printf("[TASK] task_hmi no core %d\n", xPortGetCoreID());

  uint32_t lastLedUpdate = 0;
  uint32_t lastDispUpdate = 0;

  int8_t last_gear_seen = -99;

  while (1) {
    uint32_t now = millis();

    /* 1. Coleta super rápida via Mutex */
    uint16_t rpm;
    int8_t ecu_gear;
    float v_kmh;
    float temp_c;
    float bat_v;
    float lambda_val;

    xSemaphoreTake(g_dataMutex, portMAX_DELAY);
    rpm = g_ecu.engine_rpm;
    ecu_gear = (int8_t)g_ecu.gear_position;
    v_kmh = g_ecu.vehicle_speed * PT_VSPD_SCALE;
    temp_c = g_ecu.engine_temp * PT_TEMP_SCALE;
    bat_v = g_ecu.battery_voltage * PT_BAT_SCALE;
    lambda_val = g_ecu.lambda1 * PT_LAMBDA_SCALE;
    xSemaphoreGive(g_dataMutex);

    /* ========================================================
         * 2. O CÉREBRO DA TRANSMISSÃO (Cálculo Instantâneo)
         * ======================================================== */
    int8_t fast_gear = calcularMarchaInstantanea(v_kmh, rpm);

    // Se a matemática disser "0", usamos a marcha da ProTune.
    // Isso resolve o problema de mostrar a 1ª marcha no grid de largada.
    int8_t current_gear = (fast_gear != 0) ? fast_gear : ecu_gear;

    if (now - lastLedUpdate >= LED_UPDATE_MS) {
      lastLedUpdate = now;
      updateAlertLEDs();
      updateRPMBar(rpm, current_gear);
    }

    /* Força tela instantânea caso o número da marcha mude */
    bool forceDisplayUpdate = false;
    if (current_gear != last_gear_seen) {
      forceDisplayUpdate = true;
      last_gear_seen = current_gear;
    }

    /* 4. Atualiza o LCD do Volante */
    if ((now - lastDispUpdate >= DISPLAY_UPDATE_MS) || forceDisplayUpdate) {
      lastDispUpdate = now;
      updateDisplay(rpm, current_gear, temp_c, bat_v, lambda_val);
    }

    /* 4. Atualiza o LCD do Volante */
    if ((now - lastDispUpdate >= DISPLAY_UPDATE_MS) || forceDisplayUpdate) {
      lastDispUpdate = now;
      updateDisplay(rpm, current_gear, temp_c, bat_v, lambda_val);

      /* 5. Atualiza o OLED de Freio se a pressão mudar */
      updateBrakeBiasDisplay();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ============================================================================
 * TASK DE ANÁLISE E ALERTAS (CORE 1)
 * ============================================================================ */
static void task_alerts(void *arg) {
  (void)arg;
  Serial.printf("[TASK] task_alerts no core %d\n", xPortGetCoreID());

  /* Variáveis de estado para garantir que o gatilho só dispare UMA VEZ na mudança */
  bool stateBat = false;
  bool stateTemp = false;
  bool stateOil = false;
  bool stateFuel = false;

  while (1) {
    /* 1. Coleta os dados de forma thread-safe */
    float bat_v, temp_c, oil_bar, fuel_ps;
    uint16_t rpm;

    xSemaphoreTake(g_dataMutex, portMAX_DELAY);
    bat_v = g_ecu.battery_voltage * PT_BAT_SCALE;
    temp_c = g_ecu.engine_temp * PT_TEMP_SCALE;
    oil_bar = g_ecu.oil_pressure * PT_OIL_SCALE;
    fuel_ps = g_ecu.fuel_pressure * PT_FUEL_SCALE;
    rpm = g_ecu.engine_rpm;
    xSemaphoreGive(g_dataMutex);

    /* 2. Análise da Bateria (Abaixo de 11.5V ou Acima de 14.8V) */
    if (bat_v <= LIMIT_BAT_MIN || bat_v >= LIMIT_BAT_MAX) {
      if (!stateBat) {
        stateBat = true;
        onBatWarningEnter();
      }
    } else {
      if (stateBat) {
        stateBat = false;
        onBatWarningExit();
      }
    }

    /* 3. Análise da Temperatura */
    if (temp_c >= LIMIT_TEMP_MAX) {
      if (!stateTemp) {
        stateTemp = true;
        onTempWarningEnter();
      }
    } else {
      // Usa 90 graus para evitar que o LED fique piscando se a temp ficar em 94.9 e 95.0 (Histerese)
      if (stateTemp && temp_c < (LIMIT_TEMP_MAX - 5.0f)) {
        stateTemp = false;
        onTempWarningExit();
      }
    }

    /* 4. Análise do Óleo (Ignora se o motor estiver desligado) */
    if (rpm > 800) {
      if (oil_bar <= LIMIT_OIL_MIN) {
        if (!stateOil) {
          stateOil = true;
          onOilWarningEnter();
        }
      } else {
        if (stateOil) {
          stateOil = false;
          onOilWarningExit();
        }
      }
    } else {
      /* Se o motor apagou, não queremos acender o alerta de óleo */
      if (stateOil) {
        stateOil = false;
        onOilWarningExit();
      }
    }

    /* 5. Análise do Combustível */
    if (rpm > 800)
      if (fuel_ps <= LIMIT_FUEL_MIN) {
        if (!stateFuel) {
          stateFuel = true;
          onFuelWarningEnter();
        }
      } else {
        if (stateFuel) {
          stateFuel = false;
          onFuelWarningExit();
        }
      }

    /* Varre a cada 100ms (10 Hz é mais que suficiente para alertas visuais) */
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* ============================================================================
 * TASK CORE 0 — TELEMETRIA 4G
 * ============================================================================ */
static void task_4g(void *arg) {
    (void)arg;
    Serial.printf("[TASK] task_4g no core %d\n", xPortGetCoreID());

    /* 1. Liga e Conecta (Pode demorar, mas como é uma Task, não trava o CAN) */
    powerOnModem();
    while (!initGSM()) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Tenta de novo a cada 5 segundos se falhar
    }

    /* 2. Loop Principal de Envio */
    while (1) {
        if (g_carState == CAR_MOVING) {
            /* Variáveis para receber os dados crus e processados */
            float temp_c, v_kmh, oil_bar;
            uint16_t rpm, raw_diant, raw_tras;
            int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
            
            /* Coleta super rápida e segura do Mutex global */
            xSemaphoreTake(g_dataMutex, portMAX_DELAY);
            temp_c    = g_ecu.engine_temp * PT_TEMP_SCALE;
            v_kmh     = g_ecu.vehicle_speed * PT_VSPD_SCALE;
            oil_bar   = g_ecu.oil_pressure * PT_OIL_SCALE;
            rpm       = g_ecu.engine_rpm;
            
            raw_diant = g_pedals.press_freio_diant;
            raw_tras  = g_pedals.press_freio_tras;
            
            raw_ax    = g_imu.accel_x;
            raw_ay    = g_imu.accel_y;
            raw_az    = g_imu.accel_z;
            raw_gx    = g_imu.gyro_x;
            raw_gy    = g_imu.gyro_y;
            raw_gz    = g_imu.gyro_z;
            xSemaphoreGive(g_dataMutex);
            
            // 1. Freios para PSI
            float f_diant = (float)raw_diant;
            float f_tras  = (float)raw_tras;
            if (f_diant < FREIO_ADC_MIN) f_diant = FREIO_ADC_MIN;
            if (f_tras  < FREIO_ADC_MIN) f_tras  = FREIO_ADC_MIN;
            float psi_diant = ((f_diant - FREIO_ADC_MIN) / (FREIO_ADC_MAX - FREIO_ADC_MIN)) * FREIO_PSI_MAX;
            float psi_tras  = ((f_tras  - FREIO_ADC_MIN) / (FREIO_ADC_MAX - FREIO_ADC_MIN)) * FREIO_PSI_MAX;

            // 2. IMU para G's e Graus por Segundo (dps)
            float ax_g = raw_ax * ACCEL_SCALE;
            float ay_g = raw_ay * ACCEL_SCALE;
            float az_g = raw_az * ACCEL_SCALE;
            float gx_dps = raw_gx * GYRO_SCALE;
            float gy_dps = raw_gy * GYRO_SCALE;
            float gz_dps = raw_gz * GYRO_SCALE;

            /* --- Monta o JSON --- 
               Tamanho aumentado para 512 bytes para caber o novo payload completo */
            char jsonBuffer[512];
            snprintf(jsonBuffer, sizeof(jsonBuffer),
                "{"
                "\"lap-name\": \"%s\","
                "\"temperature\": \"%.1f\","
                "\"speed\": \"%.1f\","
                "\"pressure\": \"%.2f\","
                "\"rpm\": \"%u\","
                "\"brake_front\": \"%.1f\","
                "\"brake_rear\": \"%.1f\","
                "\"accel_x\": \"%.2f\","
                "\"accel_y\": \"%.2f\","
                "\"accel_z\": \"%.2f\","
                "\"gyro_x\": \"%.2f\","
                "\"gyro_y\": \"%.2f\","
                "\"gyro_z\": \"%.2f\""
                "}", 
                g_currentLapName, temp_c, v_kmh, oil_bar, rpm,
                psi_diant, psi_tras,
                ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps
            );

            /* Envia pro servidor (bloqueia apenas a Task 4G, liberando o resto do carro) */
            sendTelemetryPost(jsonBuffer);
            
            /* Taxa de envio: a cada 2 segundos enquanto estiver andando */
            vTaskDelay(pdMS_TO_TICKS(2000)); 
        } else {
            /* Carro parado = dorme para não processar coisas à toa */
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

/* ============================================================================
 * SETUP
 * ============================================================================ */

void setup() {
  Serial.begin(115200);
  delay(500);

  /* Inicializa I2C (Pinos padrão ESP32: SDA=21, SCL=22) */
  Wire.begin(21, 22);

  if (!g_oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("[OLED] Falha ao inicializar o SSD1306");
  } else {
    g_oled.clearDisplay();
    g_oled.setTextColor(SSD1306_WHITE);
    g_oled.setTextSize(1);
    g_oled.setCursor(0, 0);
    g_oled.println("SISTEMA ROUTE");
    g_oled.println("Aguardando Freio...");
    g_oled.display();
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println(" Logger UFS-01B - Telemetria");
  Serial.println(" MCP2515 (SPI) + SD + TM1637 + WS2812");
  Serial.println("========================================");

  /* GPIO básicos */
  pinMode(LED_VOLT_BAT, OUTPUT);
  pinMode(LED_TEMP_AGUA, OUTPUT);
  pinMode(LED_PRESS_OLEO, OUTPUT);
  pinMode(LED_COMB, OUTPUT);
  pinMode(BTN_LAP_PIN, INPUT_PULLDOWN);
  pinMode(BTN_PAGE_PIN, INPUT_PULLDOWN);

  digitalWrite(LED_VOLT_BAT, LOW);
  digitalWrite(LED_TEMP_AGUA, LOW);
  digitalWrite(LED_PRESS_OLEO, LOW);
  digitalWrite(LED_STATUS_REC, LOW);

  /* Barra WS2812 */
  FastLED.addLeds<WS2812B, RPM_LED_PIN, GRB>(g_rpmLeds, RPM_LED_COUNT);
  FastLED.setBrightness(RPM_LED_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  /* Display TM1637 */
  g_display.setBrightness(5);
  g_display.clear();

  /* Inicializa HSPI para o SD (não conflita com VSPI do MCP2515) */
  spiSD.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN, spiSD)) {
    Serial.println("[SD] AVISO - Cartao nao inicializou. Modo Bypass ativado.");
    g_sdAvailable = false;
  } else {
    Serial.println("[SD] OK (HSPI)");
    g_sdAvailable = true;
  }

  /* Inicializa MCP2515 no VSPI */
  if (!setup_mcp2515()) {
    Serial.println("[CAN] ERRO FATAL");
    while (1) {
      digitalWrite(LED_VOLT_BAT, HIGH);
      delay(100);
      digitalWrite(LED_VOLT_BAT, LOW);
      delay(100);
    }
  }

  /* ================================== */
  /* Animação de boot (SEMPRE PRIMEIRO) */
  /* ================================== */
  for (int rep = 0; rep < 2; rep++) {
    for (int i = 0; i < RPM_LED_COUNT; i++) {
      g_rpmLeds[i] = CRGB::Blue;
      FastLED.show();
      delay(50);
    }
    for (int i = 0; i < RPM_LED_COUNT; i++) {
      g_rpmLeds[i] = CRGB::Black;
      FastLED.show();
      delay(30);
    }
  }

  /* Mutex e fila */
  g_dataMutex = xSemaphoreCreateMutex();
  g_logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogSample_t));
  if (!g_dataMutex || !g_logQueue) {
    Serial.println("[RTOS] ERRO ao criar mutex/queue");
    while (1) { delay(1000); }
  }

  /* ================================== */
  /* Cria as Tasks APENAS NO FINAL      */
  /* ================================== */
  xTaskCreatePinnedToCore(task_can, "task_can", TASK_CAN_STACK, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(task_sd, "task_sd", TASK_SD_STACK, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task_hmi, "task_hmi", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task_alerts, "task_alerts", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_4g, "task_4g", 8192, NULL, 1, NULL, 0);

  Serial.println("[SYS] Pronto. Aguardando botao de lap...");
}

/* ============================================================================
 * LOOP — apenas botão
 * ============================================================================ */

void loop() {
  handleLapButton();
  handlePageButton();
  handleMotionState();

  vTaskDelay(pdMS_TO_TICKS(10));
}
