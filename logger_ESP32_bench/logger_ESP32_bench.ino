/**
 * ============================================================================
 * logger_ESP32_bench.ino
 *
 * Sistema de Telemetria UFS-01B | Fórmula Route UFSCar
 * VERSÃO DE BANCADA — MCP2515 + SD + decodificação dos nós STM32
 *
 * Esta versão é uma simplificação do logger principal, focada em validar
 * a integração CAN + SD com os dois nós STM32 antes de adicionar a ECU,
 * o display, a barra de RPM e os LEDs de alerta.
 *
 * Arquitetura:
 *   Core 1 (task_can) : Lê MCP2515 via SPI, decodifica frames, enfileira.
 *   Core 0 (task_sd)  : Consome fila e grava CSV no SD com flush a 1Hz.
 *   Loop (main)       : Trata botão de lap (start/stop gravação).
 *
 * Frames CAN recebidos:
 *   0x201 | DLC 8 | Acel XYZ + Gyro X  (nó IMU)
 *   0x204 | DLC 4 | Gyro YZ             (nó IMU)
 *   0x202 | DLC 8 | Hall Acel + Freio + Pressão Freio Diant/Tras (nó Analog)
 *
 * ============================================================================
 * PINOUT
 * ============================================================================
 *
 * MCP2515 (VSPI):
 *   GPIO  5 → CS
 *   GPIO 23 → MOSI
 *   GPIO 19 → MISO
 *   GPIO 18 → SCK
 *   GPIO 17 → INT
 *
 * SD Card (HSPI dedicado):
 *   GPIO 15 → CS
 *   GPIO 13 → MOSI
 *   GPIO 12 → MISO
 *   GPIO 14 → SCK
 *
 * Botão Lap:
 *   GPIO 16 → INPUT_PULLUP (pressionar para iniciar/parar gravação)
 *
 * LED de status:
 *   GPIO 33 → indica gravação ativa
 *
 * Bibliotecas necessárias:
 *   - mcp_can (coryjfowler)
 *
 * Taxa de log: 50 Hz | Serial: 115200 bps
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <SD.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* ============================================================================
 * PINOUT
 * ============================================================================ */

/* MCP2515 no VSPI */
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

/* Botão e LED de status */
#define BTN_LAP_PIN         16
#define LED_STATUS_REC      33

/* ============================================================================
 * CAN IDs (Documento de Decisões Técnicas v2.0)
 * ============================================================================ */
#define CAN_ID_IMU_ACCEL    0x201   /* Acel XYZ + Gyro X (DLC 8) */
#define CAN_ID_IMU_GYRO_YZ  0x204   /* Gyro YZ           (DLC 4) */
#define CAN_ID_PEDAIS       0x202   /* Hall Acel + Freio (DLC 8) */

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
} LogSample_t;

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

/* MPU6050: ±2g / ±250°/s conforme firmware do nó IMU */
#define ACCEL_SCALE         (1.0f / 16384.0f)
#define GYRO_SCALE          (1.0f / 131.0f)

/* Hall 49E: 0-99% */
#define HALL_ADC_MAX        4095.0f
#define HALL_PCT_MAX        99.0f

/* ============================================================================
 * TEMPORIZAÇÃO
 * ============================================================================ */
#define SAMPLE_PERIOD_MS        20      /* 50 Hz             */
#define SD_FLUSH_INTERVAL_MS    1000    /* Flush a cada 1s   */

#define LOG_QUEUE_SIZE          100
#define TASK_CAN_STACK          8192
#define TASK_SD_STACK           8192

/* ============================================================================
 * VARIÁVEIS GLOBAIS
 * ============================================================================ */
static IMU_Raw_t     g_imu     = {0};
static Pedals_Raw_t  g_pedals  = {0};

static QueueHandle_t     g_logQueue  = NULL;
static SemaphoreHandle_t g_dataMutex = NULL;

static volatile bool     g_recording    = false;
static volatile uint32_t g_lapStartTime = 0;

/* Diagnóstico */
static volatile uint32_t g_frameCount_201 = 0;
static volatile uint32_t g_frameCount_204 = 0;
static volatile uint32_t g_frameCount_202 = 0;
static volatile uint32_t g_frameCount_unknown = 0;

/* MCP2515 (usa VSPI default) */
static MCP_CAN  CAN0(MCP2515_CS_PIN);

/* HSPI dedicado ao SD */
static SPIClass spiSD(HSPI);

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
                g_frameCount_201++;

                if (g_frameCount_201 % 50 == 0){
                    Serial.printf("[0x201] Ax=%+7.3fg Ay=%+7.3fg Az=%+7.3fg | Gx=%+8.2f dps\n",
                        g_imu.accel_x * ACCEL_SCALE,
                        g_imu.accel_y * ACCEL_SCALE,
                        g_imu.accel_z * ACCEL_SCALE,
                        g_imu.gyro_x  * GYRO_SCALE);
                }
            }
            break;

        case CAN_ID_IMU_GYRO_YZ:
            if (len >= 4) {
                g_imu.gyro_y = (int16_t)((data[0] << 8) | data[1]);
                g_imu.gyro_z = (int16_t)((data[2] << 8) | data[3]);
                g_frameCount_204++;

                if (g_frameCount_204 % 50 == 0){
                    Serial.printf("[0x204] Gy=%+8.2f dps  Gz=%+8.2f dps\n", g_imu.gyro_y * GYRO_SCALE, g_imu.gyro_z * GYRO_SCALE);
                }
            }
            break;

        case CAN_ID_PEDAIS:
            if (len >= 8) {
                g_pedals.hall_acel         = (uint16_t)((data[0] << 8) | data[1]);
                g_pedals.hall_freio        = (uint16_t)((data[2] << 8) | data[3]);
                g_pedals.press_freio_diant = (uint16_t)((data[4] << 8) | data[5]);
                g_pedals.press_freio_tras  = (uint16_t)((data[6] << 8) | data[7]);
                g_frameCount_202++;

                if(g_frameCount_202 % 50 == 0){
                    Serial.printf("[0x202] Acel=%5.1f%% (raw=%4u) | Freio=%5.1f%% (raw=%4u) | "
                                  "PFrDiant=%4u | PFrTras=%4u\n",
                        (g_pedals.hall_acel  * HALL_PCT_MAX) / HALL_ADC_MAX, g_pedals.hall_acel,
                        (g_pedals.hall_freio * HALL_PCT_MAX) / HALL_ADC_MAX, g_pedals.hall_freio,
                        g_pedals.press_freio_diant, g_pedals.press_freio_tras);
                }
            }
            break;

        default:
            g_frameCount_unknown++;
            break;
    }

    xSemaphoreGive(g_dataMutex);
}

static void buildSnapshot(LogSample_t *s)
{
    xSemaphoreTake(g_dataMutex, portMAX_DELAY);

    s->timestamp_ms = millis() - g_lapStartTime;

    /* IMU */
    s->ax_g   = g_imu.accel_x * ACCEL_SCALE;
    s->ay_g   = g_imu.accel_y * ACCEL_SCALE;
    s->az_g   = g_imu.accel_z * ACCEL_SCALE;
    s->gx_dps = g_imu.gyro_x  * GYRO_SCALE;
    s->gy_dps = g_imu.gyro_y  * GYRO_SCALE;
    s->gz_dps = g_imu.gyro_z  * GYRO_SCALE;

    /* Pedais */
    s->hall_acel_pct  = (g_pedals.hall_acel  * HALL_PCT_MAX) / HALL_ADC_MAX;
    s->hall_freio_pct = (g_pedals.hall_freio * HALL_PCT_MAX) / HALL_ADC_MAX;

    /* Pressão de freio (raw) */
    s->press_freio_diant_raw = g_pedals.press_freio_diant;
    s->press_freio_tras_raw  = g_pedals.press_freio_tras;

    xSemaphoreGive(g_dataMutex);
}

/* ============================================================================
 * INICIALIZAÇÃO DO MCP2515
 *
 * Assume cristal de 8 MHz no módulo. Se for 16 MHz, troque MCP_8MHZ por
 * MCP_16MHZ. Verifique o cristal soldado no módulo físico.
 * ============================================================================ */

static bool setup_mcp2515(void)
{
    SPI.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, MCP2515_CS_PIN);
    pinMode(MCP2515_INT_PIN, INPUT_PULLUP);

    const uint8_t maxRetries = 5;
    for (uint8_t attempt = 1; attempt <= maxRetries; attempt++) {
        Serial.printf("[MCP2515] Tentativa %d/%d... ", attempt, maxRetries);

        if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
            Serial.println("OK");
            CAN0.setMode(MCP_NORMAL);
            return true;
        }

        Serial.println("falhou");
        delay(200);
    }

    Serial.println("[MCP2515] ERRO FATAL");
    Serial.println("  - Verifique fiacao SPI (MOSI=23, MISO=19, SCK=18, CS=5)");
    Serial.println("  - Verifique alimentacao 5V do modulo");
    Serial.println("  - Verifique cristal (8 MHz esperado)");
    return false;
}

/* ============================================================================
 * SD CARD
 * ============================================================================ */

static File     g_dataFile;
static uint32_t g_lastFlushTime = 0;

static void createNewLogFile(void)
{
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
        "press_freio_diant_raw,press_freio_tras_raw"
    );
    g_dataFile.flush();
    g_lastFlushTime = millis();

    Serial.printf("[SD] Arquivo criado: %s\n", filename);
}

static void writeSampleToSD(const LogSample_t *s)
{
    if (!g_dataFile) return;

    /* Monta a linha em buffer local — single write é mais rápido que
     * múltiplas chamadas a print(). */
    char buf[224];
    int n = snprintf(buf, sizeof(buf),
        "%lu,"
        "%.4f,%.4f,%.4f,"
        "%.2f,%.2f,%.2f,"
        "%.1f,%.1f,"
        "%u,%u\n",
        (unsigned long)s->timestamp_ms,
        s->ax_g, s->ay_g, s->az_g,
        s->gx_dps, s->gy_dps, s->gz_dps,
        s->hall_acel_pct, s->hall_freio_pct,
        s->press_freio_diant_raw, s->press_freio_tras_raw
    );

    if (n > 0) {
        g_dataFile.write((const uint8_t*)buf, n);
    }

    /* Flush periódico — não a cada linha! */
    uint32_t now = millis();
    if (now - g_lastFlushTime >= SD_FLUSH_INTERVAL_MS) {
        g_dataFile.flush();
        g_lastFlushTime = now;
    }
}

/* ============================================================================
 * TASK CORE 1 — Recepção CAN
 * ============================================================================ */

static void task_can(void *arg)
{
    (void)arg;
    Serial.printf("[TASK] task_can no core %d\n", xPortGetCoreID());

    uint32_t lastSample    = 0;
    uint32_t lastStatsTime = millis();

    while (1) {
        /* Drena todas as mensagens pendentes do MCP2515 (até 2 buffers) */
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
                break;
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

        /* Estatísticas a cada 5s — útil para diagnóstico de bancada */
        if (now - lastStatsTime >= 5000) {
            uint32_t dt = now - lastStatsTime;
            lastStatsTime = now;

            uint32_t c201, c204, c202, cunk;
            xSemaphoreTake(g_dataMutex, portMAX_DELAY);
            c201 = g_frameCount_201; g_frameCount_201 = 0;
            c204 = g_frameCount_204; g_frameCount_204 = 0;
            c202 = g_frameCount_202; g_frameCount_202 = 0;
            cunk = g_frameCount_unknown; g_frameCount_unknown = 0;
            xSemaphoreGive(g_dataMutex);

            Serial.printf("[STATS] 0x201=%lu (%.1fHz) | 0x204=%lu (%.1fHz) | "
                          "0x202=%lu (%.1fHz) | unk=%lu | rec=%s\n",
                          c201, c201 * 1000.0f / dt,
                          c204, c204 * 1000.0f / dt,
                          c202, c202 * 1000.0f / dt,
                          cunk,
                          g_recording ? "SIM" : "NAO");
        }

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

        if (current == LOW) {
            if (!g_recording) {
                g_lapStartTime = millis();
                createNewLogFile();
                g_recording = true;
                digitalWrite(LED_STATUS_REC, HIGH);
                Serial.println("[REC] >>> GRAVACAO INICIADA <<<");
            } else {
                g_recording = false;
                digitalWrite(LED_STATUS_REC, LOW);
                vTaskDelay(pdMS_TO_TICKS(100));  /* drena fila */
                if (g_dataFile) {
                    g_dataFile.flush();
                    g_dataFile.close();
                }
                Serial.println("[REC] >>> GRAVACAO PARADA <<<");
            }
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
    Serial.println(" Logger BANCADA - UFS-01B");
    Serial.println(" MCP2515 + SD | Nos IMU + Analog");
    Serial.println("========================================");

    pinMode(LED_STATUS_REC, OUTPUT);
    pinMode(BTN_LAP_PIN,    INPUT_PULLUP);
    digitalWrite(LED_STATUS_REC, LOW);

    /* HSPI dedicado ao SD */
    spiSD.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, SD_CS_PIN);

    if (!SD.begin(SD_CS_PIN, spiSD)) {
        Serial.println("[SD] ERRO FATAL - cartao nao inicializou");
        while (1) {
            digitalWrite(LED_STATUS_REC, HIGH); delay(200);
            digitalWrite(LED_STATUS_REC, LOW);  delay(200);
        }
    }
    Serial.println("[SD] OK (HSPI)");

    /* MCP2515 no VSPI */
    if (!setup_mcp2515()) {
        while (1) {
            digitalWrite(LED_STATUS_REC, HIGH); delay(100);
            digitalWrite(LED_STATUS_REC, LOW);  delay(100);
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

    Serial.println();
    Serial.println("[SYS] Pronto.");
    Serial.println("      Aguardando frames CAN (0x201, 0x204, 0x202)...");
    Serial.println("      Pressione o botao GPIO 16 para iniciar gravacao.");
    Serial.println();
}

/* ============================================================================
 * LOOP
 * ============================================================================ */

void loop()
{
    handleLapButton();
    delay(10);
}
