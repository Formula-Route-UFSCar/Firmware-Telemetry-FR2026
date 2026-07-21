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

/* ============================================================================
 * PINOUT
 * ============================================================================ */

/* MCP2515 no VSPI (default SPI do ESP32) */
#define MCP2515_CS_PIN      5
#define MCP2515_INT_PIN     17
#define VSPI_MOSI           23
#define VSPI_MISO           19
#define VSPI_SCK            18

/* SD Card no HSPI */
#define SD_CS_PIN           15
#define HSPI_MOSI           13
#define HSPI_MISO           12
#define HSPI_SCK            14

/* Volante */
#define RPM_LED_PIN         27
#define DISPLAY_DIO_PIN     26
#define DISPLAY_CLK_PIN     25
#define BTN_LAP_PIN         16
#define BTN_PAGE_PIN        2

/* LEDs de aviso */
#define LED_VOLT_BAT        21
#define LED_TEMP_AGUA       22
#define LED_PRESS_OLEO      32
#define LED_STATUS_REC      33

/* ============================================================================
 * BARRA WS2812B DE RPM
 * ============================================================================ */
#define RPM_LED_COUNT       8
#define RPM_MIN             3000
#define RPM_MAX             13000
#define RPM_SHIFT_WARN      11500
#define RPM_SHIFT_CRIT      12500
#define RPM_LED_BRIGHTNESS  80

/* ============================================================================
 * CAN IDs
 * ============================================================================ */
#define CAN_ID_IMU_ACCEL    0x201
#define CAN_ID_IMU_GYRO_YZ  0x204
#define CAN_ID_PEDAIS       0x202
#define CAN_ID_PROTUNE      0x1E0

/* ============================================================================
 * ESTRUTURAS DE DADOS
 * ============================================================================ */

typedef struct {
    uint32_t timestamp_ms;

    /* IMU */
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;

    /* Pedais */
    float hall_acel_pct;
    float hall_freio_pct;

    /* Pressão de freio (raw ADC 0–4095; sem calibração bar definida) */
    uint16_t press_freio_diant_raw;
    uint16_t press_freio_tras_raw;

    /* ECU ProTune */
    uint16_t rpm;
    float    ign_angle_deg;
    int8_t   gear;
    float    map_kpa;
    float    iat_c;
    float    engine_temp_c;
    float    throttle_pct;
    float    battery_v;
    float    lambda1;
    float    oil_press_bar;
    float    fuel_press_bar;
    float    vehicle_speed_kph;
    float    fuel_level_l;
} LogSample_t;

typedef struct {
    uint16_t engine_rpm;
    int16_t  ign_angle;
    int16_t  gear_position;
    int16_t  map;
    int16_t  iat;
    int16_t  engine_temp;
    int16_t  throttle_pos;
    int16_t  battery_voltage;
    int16_t  lambda1;
    int16_t  vehicle_speed;
    int16_t  oil_pressure;
    int16_t  fuel_pressure;
    uint16_t fuel_level;
} ECU_Raw_t;

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
} IMU_Raw_t;

typedef struct {
    uint16_t hall_acel;
    uint16_t hall_freio;
    uint16_t press_freio_diant;
    uint16_t press_freio_tras;
} Pedals_Raw_t;

/* ============================================================================
 * CONSTANTES DE CONVERSÃO
 * ============================================================================ */

/* MPU6050: ±2g / ±250°/s */
#define ACCEL_SCALE         (1.0f / 16384.0f)
#define GYRO_SCALE          (1.0f / 131.0f)

/* Hall 49E */
#define HALL_ADC_MAX        4095.0f
#define HALL_PCT_MAX        99.0f

/* ECU ProTune — escalas conforme manual */
#define PT_IGN_SCALE        (1.0f / 10.0f)
#define PT_MAP_SCALE        (1.0f / 10.0f)
#define PT_IAT_SCALE        (1.0f / 10.0f)
#define PT_TEMP_SCALE       (1.0f / 10.0f)
#define PT_TP_SCALE         (1.0f / 10.0f)
#define PT_BAT_SCALE        (1.0f / 10.0f)
#define PT_LAMBDA_SCALE     (1.0f / 1000.0f)
#define PT_OIL_SCALE        (1.0f / 100.0f)
#define PT_FUEL_SCALE       (1.0f / 100.0f)
#define PT_VSPD_SCALE       (1.0f / 10.0f)
#define PT_FUEL_LVL_SCALE   (1.0f / 100.0f)

/* ============================================================================
 * TEMPORIZAÇÃO
 * ============================================================================ */
#define SAMPLE_PERIOD_MS        20
#define SD_FLUSH_INTERVAL_MS    1000
#define LED_BLINK_INTERVAL_MS   250
#define DISPLAY_UPDATE_MS       100
#define LED_UPDATE_MS           50

#define LOG_QUEUE_SIZE          100
#define TASK_CAN_STACK          8192
#define TASK_SD_STACK           8192

/* ============================================================================
 * VARIÁVEIS GLOBAIS
 * ============================================================================ */
static ECU_Raw_t     g_ecu     = {0};
static IMU_Raw_t     g_imu     = {0};
static Pedals_Raw_t  g_pedals  = {0};

static QueueHandle_t     g_logQueue  = NULL;
static SemaphoreHandle_t g_dataMutex = NULL;

static volatile bool     g_recording    = false;
static volatile uint32_t g_lapStartTime = 0;

static volatile uint32_t g_protuneCrcErrors = 0;
static volatile uint32_t g_protuneCrcOk     = 0;

static bool g_sdAvailable = false; //Bypass SD

static uint8_t g_displayPage = 0; 
#define MAX_DISPLAY_PAGES    4

/* Instância MCP2515 (usa VSPI default do ESP32) */
static MCP_CAN       CAN0(MCP2515_CS_PIN);

/* Segundo barramento SPI dedicado ao SD (HSPI) */
static SPIClass      spiSD(HSPI);

/* Periféricos do volante */
static CRGB          g_rpmLeds[RPM_LED_COUNT];
static TM1637Display g_display(DISPLAY_CLK_PIN, DISPLAY_DIO_PIN);

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

static uint8_t CheckSumCAN(const uint8_t *ss)
{
    uint8_t size     = 8;
    uint8_t checksum = 0xFF;
    while (size--) {
        checksum += *ss++;
    }
    return checksum;
}

/* ============================================================================
 * CORREÇÃO: Funções ajustadas para ler BIG-ENDIAN (MSB first)
 * ============================================================================ */
static inline int16_t pt_i16(const uint8_t *data, uint8_t offset)
{
    /* Desloca o primeiro byte (MSB) 8 vezes e soma com o segundo (LSB) */
    return (int16_t)((data[offset] << 8) | data[offset + 1]);
}

static inline uint16_t pt_u16(const uint8_t *data, uint8_t offset)
{
    return (uint16_t)((data[offset] << 8) | data[offset + 1]);
}

/* ============================================================================
 * DECODIFICAÇÃO PROTUNE (Conforme Manual Oficial em C/C++)
 * ============================================================================ */
static void decodeProTunePacket(const uint8_t *data_in)
{
    uint8_t rx[8];
    memcpy(rx, data_in, 8);

    uint8_t packet  = rx[0];
    uint8_t dataset = rx[1] >> 5;  // usa somente os 3 primeiros bits (dataset)

    if (dataset > 4) return;       // qualquer valor acima ou igual a 4 é ignorado

    uint8_t checksum = rx[1] & 0x1F; // usa apenas os 5 últimos bits (checkSum)

    rx[1] &= ~0x1F; // Mascara o checksum do pacote atual

    uint8_t checkCalc = CheckSumCAN(rx) & 0x1F; // Calcula checksum e testa se está oK

    if (checksum != checkCalc) {
        g_protuneCrcErrors++;
        return;
    }
    g_protuneCrcOk++;

    /* O Switch vai direto no Packet, sem restrição de Dataset */
    switch (packet) {
        case 2:
            g_ecu.engine_rpm = pt_u16(rx, 2);
            g_ecu.ign_angle  = pt_i16(rx, 4);
            break;

        case 4:
            g_ecu.gear_position = pt_i16(rx, 4);
            break;

        case 5:
            g_ecu.map         = pt_i16(rx, 2);
            g_ecu.iat         = pt_i16(rx, 4);
            g_ecu.engine_temp = pt_i16(rx, 6);
            break;

        case 6:
            g_ecu.throttle_pos    = pt_i16(rx, 2);
            g_ecu.battery_voltage = pt_i16(rx, 4);
            g_ecu.lambda1         = pt_i16(rx, 6);
            break;

        case 8:
            g_ecu.vehicle_speed = pt_i16(rx, 4);
            break;

        case 15:
            g_ecu.oil_pressure  = pt_i16(rx, 2);
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

static void processCANMessage(uint32_t id, const uint8_t *data, uint8_t len)
{
    xSemaphoreTake(g_dataMutex, portMAX_DELAY);

    switch (id) {
        case CAN_ID_IMU_ACCEL:  /* Acel XYZ + Gyro X */
            if (len >= 8) {
                g_imu.accel_x = (int16_t)((data[0] << 8) | data[1]);
                g_imu.accel_y = (int16_t)((data[2] << 8) | data[3]);
                g_imu.accel_z = (int16_t)((data[4] << 8) | data[5]);
                g_imu.gyro_x  = (int16_t)((data[6] << 8) | data[7]);
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
                g_pedals.hall_acel         = (uint16_t)((data[0] << 8) | data[1]);
                g_pedals.hall_freio        = (uint16_t)((data[2] << 8) | data[3]);
                g_pedals.press_freio_diant = (uint16_t)((data[4] << 8) | data[5]);
                g_pedals.press_freio_tras  = (uint16_t)((data[6] << 8) | data[7]);
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

static void buildSnapshot(LogSample_t *s)
{
    xSemaphoreTake(g_dataMutex, portMAX_DELAY);

    s->timestamp_ms = millis() - g_lapStartTime;

    s->ax_g   = g_imu.accel_x * ACCEL_SCALE;
    s->ay_g   = g_imu.accel_y * ACCEL_SCALE;
    s->az_g   = g_imu.accel_z * ACCEL_SCALE;
    s->gx_dps = g_imu.gyro_x  * GYRO_SCALE;
    s->gy_dps = g_imu.gyro_y  * GYRO_SCALE;
    s->gz_dps = g_imu.gyro_z  * GYRO_SCALE;

    s->hall_acel_pct  = (g_pedals.hall_acel  * HALL_PCT_MAX) / HALL_ADC_MAX;
    s->hall_freio_pct = (g_pedals.hall_freio * HALL_PCT_MAX) / HALL_ADC_MAX;

    s->press_freio_diant_raw = g_pedals.press_freio_diant;
    s->press_freio_tras_raw  = g_pedals.press_freio_tras;

    s->rpm               = g_ecu.engine_rpm;
    s->ign_angle_deg     = g_ecu.ign_angle       * PT_IGN_SCALE;
    s->gear              = (int8_t)g_ecu.gear_position;
    s->map_kpa           = g_ecu.map             * PT_MAP_SCALE;
    s->iat_c             = g_ecu.iat             * PT_IAT_SCALE;
    s->engine_temp_c     = g_ecu.engine_temp     * PT_TEMP_SCALE;
    s->throttle_pct      = g_ecu.throttle_pos    * PT_TP_SCALE;
    s->battery_v         = g_ecu.battery_voltage * PT_BAT_SCALE;
    s->lambda1           = g_ecu.lambda1         * PT_LAMBDA_SCALE;
    s->oil_press_bar     = g_ecu.oil_pressure    * PT_OIL_SCALE;
    s->fuel_press_bar    = g_ecu.fuel_pressure   * PT_FUEL_SCALE;
    s->vehicle_speed_kph = g_ecu.vehicle_speed   * PT_VSPD_SCALE;
    s->fuel_level_l      = g_ecu.fuel_level      * PT_FUEL_LVL_SCALE;

    xSemaphoreGive(g_dataMutex);
}

/* ============================================================================
 * INICIALIZAÇÃO DO MCP2515
 *
 * Assume cristal de 8 MHz no módulo. Se for 16 MHz, trocar MCP_8MHZ por
 * MCP_16MHZ. Verifique o cristal soldado no módulo físico.
 * ============================================================================ */

static bool setup_mcp2515(void)
{
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

static void updateAlertLEDs(void)
{
    static uint32_t lastBlink[3] = {0, 0, 0};
    const uint32_t now = millis();

    float bat_v, temp_c, oil_bar;

    xSemaphoreTake(g_dataMutex, portMAX_DELAY);
    bat_v   = g_ecu.battery_voltage * PT_BAT_SCALE;
    temp_c  = g_ecu.engine_temp     * PT_TEMP_SCALE;
    oil_bar = g_ecu.oil_pressure    * PT_OIL_SCALE;
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

static void updateRPMBar(uint16_t rpm)
{
    static uint32_t lastBlink  = 0;
    static bool     blinkState = false;
    const uint32_t now = millis();

    if (rpm >= RPM_SHIFT_CRIT) {
        if (now - lastBlink >= 60) {
            blinkState = !blinkState;
            lastBlink  = now;
        }
        CRGB color = blinkState ? CRGB::Blue : CRGB::Red;
        for (int i = 0; i < RPM_LED_COUNT; i++) g_rpmLeds[i] = color;
        FastLED.show();
        return;
    }

    if (rpm >= RPM_SHIFT_WARN) {
        if (now - lastBlink >= 120) {
            blinkState = !blinkState;
            lastBlink  = now;
        }
        CRGB color = blinkState ? CRGB::Red : CRGB::Black;
        for (int i = 0; i < RPM_LED_COUNT; i++) g_rpmLeds[i] = color;
        FastLED.show();
        return;
    }

    int ledsOn = 0;
    if (rpm > RPM_MIN) {
        ledsOn = map(rpm, RPM_MIN, RPM_MAX, 0, RPM_LED_COUNT);
        if (ledsOn > RPM_LED_COUNT) ledsOn = RPM_LED_COUNT;
    }

    for (int i = 0; i < RPM_LED_COUNT; i++) {
        if (i < ledsOn) {
            if      (i < 3) g_rpmLeds[i] = CRGB::Green;
            else if (i < 6) g_rpmLeds[i] = CRGB::Yellow;
            else            g_rpmLeds[i] = CRGB::Red;
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

static void updateDisplay(uint16_t rpm, int8_t gear, float temp_c, float bat_v, float lambda)
{
    /* Segmentos customizados úteis para o TM1637 (a=0x01, b=0x02, ..., g=0x40) */
    const uint8_t CHAR_G = 0x7D; // 'G' (Marcha)
    const uint8_t CHAR_C = 0x39; // 'C' (Celsius)
    const uint8_t CHAR_U = 0x3E; // 'U' (Volts)
    const uint8_t CHAR_L = 0x38; // 'L' (Lambda)
    const uint8_t CHAR_N = 0x54; // 'n' (Neutro)
    const uint8_t CHAR_R = 0x50; // 'r' (Re)

    uint8_t segs[4] = {0, 0, 0, 0};

    switch (g_displayPage) {
        case 0: /* Página 0: Automática (RPM ou Marcha) */
            if (rpm >= 1000) {
                g_display.showNumberDec(rpm, false);
            } else if (rpm > 0) {
                g_display.showNumberDec(rpm, true);
            } else {
                /* Motor parado: exibe a marcha atual */
                segs[2] = 0; // Garante que o dígito da dezena está apagado
                
                if (gear == 0) {
                    segs[3] = CHAR_N; // Ponto morto: mostra apenas 'n'
                } 
                else if (gear > 0 && gear <= 9) {
                    segs[3] = g_display.encodeDigit(gear); // Mostra o número da marcha (1 a 6)
                } 
                else if (gear < 0) {
                    segs[3] = CHAR_R; // Ré: mostra 'r'
                }
                
                g_display.setSegments(segs);
            }
            break;

        case 1: /* Página 1: Temperatura (C XX) */
            segs[0] = CHAR_C;
            segs[1] = 0; // Espaço
            g_display.setSegments(segs, 2, 0); // Desenha os 2 primeiros
            g_display.showNumberDec((int)temp_c, false, 2, 2); // Desenha os 2 últimos
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
            
            // Ex: Lambda 0.980 -> mostramos "980" nos 3 últimos dígitos.
            int lam_display = (int)(lambda * 1000.0f);
            g_display.showNumberDec(lam_display, true, 3, 1); 
            break;
        }
    }
}

/* ============================================================================
 * SD CARD
 * ============================================================================ */

static File     g_dataFile;
static uint32_t g_lastFlushTime = 0;

static void createNewLogFile(void)
{
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
        "vehicle_speed_kph,fuel_level_l"
    );
    g_dataFile.flush();
    g_lastFlushTime = millis();

    Serial.printf("[SD] Arquivo criado: %s\n", filename);
}

static void writeSampleToSD(const LogSample_t *s)
{
    if (!g_sdAvailable || !g_dataFile) return;

    char buf[352];
    int n = snprintf(buf, sizeof(buf),
        "%lu,"
        "%.4f,%.4f,%.4f,"
        "%.2f,%.2f,%.2f,"
        "%.1f,%.1f,"
        "%u,%u,"
        "%u,%.1f,%d,"
        "%.1f,%.1f,%.1f,%.1f,"
        "%.1f,%.3f,"
        "%.2f,%.2f,"
        "%.1f,%.2f\n",
        (unsigned long)s->timestamp_ms,
        s->ax_g, s->ay_g, s->az_g,
        s->gx_dps, s->gy_dps, s->gz_dps,
        s->hall_acel_pct, s->hall_freio_pct,
        s->press_freio_diant_raw, s->press_freio_tras_raw,
        s->rpm, s->ign_angle_deg, s->gear,
        s->map_kpa, s->iat_c, s->engine_temp_c, s->throttle_pct,
        s->battery_v, s->lambda1,
        s->oil_press_bar, s->fuel_press_bar,
        s->vehicle_speed_kph, s->fuel_level_l
    );

    if (n > 0) {
        g_dataFile.write((const uint8_t*)buf, n);
    }

    uint32_t now = millis();
    if (now - g_lastFlushTime >= SD_FLUSH_INTERVAL_MS) {
        g_dataFile.flush();
        g_lastFlushTime = now;
    }
}

/* ============================================================================
 * TASK CORE 1 — MCP2515 + LEDs + display
 * ============================================================================ */

static void task_can(void *arg)
{
    (void)arg;
    Serial.printf("[TASK] task_can no core %d\n", xPortGetCoreID());

    uint32_t lastSample     = 0;
    uint32_t lastLedUpdate  = 0;
    uint32_t lastDispUpdate = 0;
    uint32_t lastStatsTime  = millis();

    while (1) {
        /* Lê mensagens enquanto o pino INT estiver baixo.
         * O MCP2515 pode ter até 2 mensagens em buffer (RXB0, RXB1),
         * então drenamos até esvaziar. */
        while (digitalRead(MCP2515_INT_PIN) == LOW) {
            unsigned long canId = 0;
            uint8_t       len   = 0;
            uint8_t       buf[8];

            if (CAN0.readMsgBuf(&canId, &len, buf) == CAN_OK) {
                /* Ignora extended e RTR */
                if (!(canId & 0x80000000UL) && !(canId & 0x40000000UL)) {
                    uint32_t id = canId & 0x7FF;
                    processCANMessage(id, buf, len);
                }
            } else {
                break;  /* Evita loop infinito se leitura falhar */
            }
        }

        uint32_t now = millis();

        /* Enfileira sample a 50 Hz */
        if (g_recording && (now - lastSample >= SAMPLE_PERIOD_MS)) {
            lastSample = now;
            LogSample_t sample;
            buildSnapshot(&sample);
            if (xQueueSend(g_logQueue, &sample, 0) != pdTRUE) {
                static uint32_t dropCount = 0;
                dropCount++;
                if (dropCount % 50 == 0) {
                    Serial.printf("[WARN] Fila cheia, %lu descartados\n", dropCount);
                }
            }
        }

        /* LEDs + barra RPM a cada 50 ms */
        if (now - lastLedUpdate >= LED_UPDATE_MS) {
            lastLedUpdate = now;
            updateAlertLEDs();

            uint16_t rpm;
            xSemaphoreTake(g_dataMutex, portMAX_DELAY);
            rpm = g_ecu.engine_rpm;
            xSemaphoreGive(g_dataMutex);

            updateRPMBar(rpm);
        }

        /* Display a cada 100 ms */
/* Display a cada 100 ms */
        if (now - lastDispUpdate >= DISPLAY_UPDATE_MS) {
            lastDispUpdate = now;
            
            uint16_t rpm;
            int8_t   gear;
            float    temp_c;
            float    bat_v;
            float    lambda_val;

            xSemaphoreTake(g_dataMutex, portMAX_DELAY);
            rpm        = g_ecu.engine_rpm;
            gear       = (int8_t)g_ecu.gear_position;
            temp_c     = g_ecu.engine_temp * PT_TEMP_SCALE;
            bat_v      = g_ecu.battery_voltage * PT_BAT_SCALE;
            lambda_val = g_ecu.lambda1 * PT_LAMBDA_SCALE;
            xSemaphoreGive(g_dataMutex);
            
            updateDisplay(rpm, gear, temp_c, bat_v, lambda_val);
        }

        /* Estatísticas a cada 5s */
        if (now - lastStatsTime >= 5000) {
            lastStatsTime = now;
            if (g_protuneCrcOk + g_protuneCrcErrors > 0) {
                Serial.printf("[ProTune] OK=%lu  CRCerr=%lu\n",
                              g_protuneCrcOk, g_protuneCrcErrors);
            }
        }

static uint32_t lastDebugTime = 0;
        if (now - lastDebugTime >= 1000) { // Atualiza a cada 1 segundo
            lastDebugTime = now;

            /* 1. Captura rápida de todos os dados protegidos pelo Mutex */
            xSemaphoreTake(g_dataMutex, portMAX_DELAY);
            
            // Dados Chassi
            int16_t raw_ax = g_imu.accel_x;
            int16_t raw_ay = g_imu.accel_y;
            int16_t raw_az = g_imu.accel_z;
            int16_t raw_gx = g_imu.gyro_x;
            int16_t raw_gy = g_imu.gyro_y;
            int16_t raw_gz = g_imu.gyro_z;
            uint16_t p_freio_diant = g_pedals.press_freio_diant;
            uint16_t p_freio_tras  = g_pedals.press_freio_tras;
            uint16_t hall_freio    = g_pedals.hall_freio;

            // Dados Motor (ProTune)
            uint16_t ecu_rpm    = g_ecu.engine_rpm;
            int16_t  ecu_temp   = g_ecu.engine_temp;
            int16_t  ecu_tps    = g_ecu.throttle_pos;
            int16_t  ecu_bat    = g_ecu.battery_voltage;
            int16_t  ecu_oil    = g_ecu.oil_pressure;
            int16_t  ecu_lambda = g_ecu.lambda1;
            
            xSemaphoreGive(g_dataMutex);

            /* 2. Conversões (Chassi) */
            float ax_g = raw_ax * ACCEL_SCALE;
            float ay_g = raw_ay * ACCEL_SCALE;
            float az_g = raw_az * ACCEL_SCALE;
            float gx = raw_gx * GYRO_SCALE;
            float gy = raw_gy * GYRO_SCALE;
            float gz = raw_gz * GYRO_SCALE;
            float hall_freio_pct = (hall_freio * HALL_PCT_MAX) / HALL_ADC_MAX;

            /* 3. Conversões (Motor) */
            float temp_c  = ecu_temp * PT_TEMP_SCALE;
            float tps_pct = ecu_tps  * PT_TP_SCALE;
            float bat_v   = ecu_bat  * PT_BAT_SCALE;
            float oil_bar = ecu_oil  * PT_OIL_SCALE;
            float lambda  = ecu_lambda * PT_LAMBDA_SCALE;

            /* 4. Impressão formatada */
            Serial.printf("[TELEMETRIA-CHASSI] ACC(g): X=%.2f Y=%.2f Z=%.2f | GYRO: X=%.2f Y=%.2f Z=%.2f | FREIO(raw): D=%u T=%u | HALL_FR: %.1f%%\n",
                          ax_g, ay_g, az_g, gx, gy, gz, p_freio_diant, p_freio_tras, hall_freio_pct);
                          
            Serial.printf("[TELEMETRIA-MOTOR]  RPM: %u | TEMP: %.1fC | TPS: %.1f%% | BAT: %.1fV | OLEO: %.2fbar | LAMBDA: %.3f\n",
                          ecu_rpm, temp_c, tps_pct, bat_v, oil_bar, lambda);
        }
        /* ------------------------------------------------------------- */

        /* Yield curto para não monopolizar o core */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ============================================================================
 * TASK CORE 0 — Escrita no SD
 * ============================================================================ */

static void task_sd(void *arg)
{
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
 * BOTÃO DE LAP
 * ============================================================================ */

static void handleLapButton(void)
{
    static bool     lastState   = HIGH;
    static uint32_t lastChange  = 0;
    const  uint32_t DEBOUNCE_MS = 50;

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
                
                if (g_sdAvailable && g_dataFile) { // Proteção extra no fechamento
                    g_dataFile.flush();
                    g_dataFile.close();
                }
                Serial.println("[REC] >>> GRAVACAO PARADA <<<");
            }
        }
        lastState = current;
    }
}

static void handlePageButton(void)
{
    static bool     lastState   = HIGH;
    static uint32_t lastChange  = 0;
    const  uint32_t DEBOUNCE_MS = 50;

    bool current = digitalRead(BTN_PAGE_PIN);
    uint32_t now = millis();

    if (current != lastState && (now - lastChange) >= DEBOUNCE_MS) {
        lastChange = now;

        if (current == LOW) { // Botão pressionado
            g_displayPage++;
            if (g_displayPage >= MAX_DISPLAY_PAGES) {
                g_displayPage = 0; // Volta para a primeira página
            }
            Serial.printf("[DISPLAY] Pagina alterada para: %d\n", g_displayPage);
        }
        lastState = current;
    }
}

/* ============================================================================
 * SETUP
 * ============================================================================ */

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" Logger UFS-01B - Telemetria");
    Serial.println(" MCP2515 (SPI) + SD + TM1637 + WS2812");
    Serial.println("========================================");

    /* GPIO básicos */
    pinMode(LED_VOLT_BAT,   OUTPUT);
    pinMode(LED_TEMP_AGUA,  OUTPUT);
    pinMode(LED_PRESS_OLEO, OUTPUT);
    pinMode(LED_STATUS_REC, OUTPUT);
    pinMode(BTN_LAP_PIN,    INPUT_PULLDOWN);
    pinMode(BTN_PAGE_PIN,   INPUT_PULLDOWN);

    digitalWrite(LED_VOLT_BAT,   LOW);
    digitalWrite(LED_TEMP_AGUA,  LOW);
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
        /* O código não trava mais aqui. Ele segue a vida sem o SD. */
    } else {
        Serial.println("[SD] OK (HSPI)");
        g_sdAvailable = true;
    }

    /* Inicializa MCP2515 no VSPI */
    if (!setup_mcp2515()) {
        Serial.println("[CAN] ERRO FATAL");
        while (1) {
            digitalWrite(LED_VOLT_BAT, HIGH); delay(100);
            digitalWrite(LED_VOLT_BAT, LOW);  delay(100);
        }
    }

    /* Mutex e fila */
    g_dataMutex = xSemaphoreCreateMutex();
    g_logQueue  = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogSample_t));
    if (!g_dataMutex || !g_logQueue) {
        Serial.println("[RTOS] ERRO ao criar mutex/queue");
        while (1) { delay(1000); }
    }

    /* Tasks */
    xTaskCreatePinnedToCore(task_can, "task_can", TASK_CAN_STACK, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(task_sd,  "task_sd",  TASK_SD_STACK,  NULL, 1, NULL, 0);

    /* Animação de boot */
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

    Serial.println("[SYS] Pronto. Aguardando botao de lap...");
}

/* ============================================================================
 * LOOP — apenas botão
 * ============================================================================ */

void loop()
{
    handleLapButton();
    handlePageButton(); /* NOVO BOTAO AQUI */
    delay(10);
}
