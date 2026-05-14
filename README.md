# Firmware Telemetria FR2026

Sistema de telemetria embarcada para carro de Fórmula SAE. Rede CAN com dois nós STM32, ECU ProTune PR440 e logger central em ESP32.

---

## Visão Geral da Arquitetura

```
┌────────────────────────────────────────────────────────────┐
│                        Barramento CAN (500 kbps)           │
│                                                            │
│  ┌──────────────┐   ┌──────────────┐   ┌────────────────┐  │
│  │  node_Acc    │   │ node_Analog  │   │  ECU PR440     │  │
│  │  STM32F103   │   │  STM32F103   │   │  (ProTune)     │  │
│  │              │   │              │   │                │  │
│  │  MPU-6050    │   │  Hall Acel   │   │  RPM, Temp     │  │
│  │  Acelerôm.   │   │  Hall Freio  │   │  Pressão, etc. │  │
│  │  Giroscópio  │   │  (ADC 12bit) │   │                │  │
│  │              │   │              │   │                │  │
│  │  0x201/0x204 │   │    0x202     │   │     0x1E0      │  │
│  └──────────────┘   └──────────────┘   └────────────────┘  │
│          │                  │                  │           │
└──────────┴──────────────────┴──────────────────┴───────────┘
                                                      │
                                          ┌───────────▼──────────┐
                                          │   Logger Central     │
                                          │   ESP32 + MCP2515    │
                                          │                      │
                                          │  • Decodifica CAN    │
                                          │  • Grava CSV (SD)    │
                                          │  • Display 7-seg     │
                                          │  • Barra LEDs RPM    │
                                          │  • LEDs de alerta    │
                                          └──────────────────────┘
```

**Taxa de amostragem:** 50 Hz (ciclo de 20 ms) em toda a rede.

---

## Estrutura de Pastas

```
Firmware-Telemetry-FR2026/
├── node_Acc/               # Projeto STM32CubeIDE — nó IMU (acelerômetro/giroscópio)
├── node_Analog/            # Projeto STM32CubeIDE — nó pedais (Hall acelerador + freio)
├── logger_ESP32/           # Firmware principal do logger (com ECU + display + LEDs)
└── logger_ESP32_bench/     # Firmware de bancada (só nós Acc e Analog, sem ECU/UI)
```

---

## Protocolo CAN

| ID CAN | DLC | Origem      | Conteúdo                                                               |
|--------|-----|-------------|------------------------------------------------------------------------|
| `0x201` | 8  | node_Acc    | `[AcelX_H, AcelX_L, AcelY_H, AcelY_L, AcelZ_H, AcelZ_L, GiroX_H, GiroX_L]` |
| `0x204` | 4  | node_Acc    | `[GiroY_H, GiroY_L, GiroZ_H, GiroZ_L]`                               |
| `0x202` | 8  | node_Analog | `[HallAcel_H, HallAcel_L, HallFreio_H, HallFreio_L, 0x00 × 4]`       |
| `0x1E0` | 8  | ECU PR440   | Parâmetros ECU (ver Manual R1 03/2024 — mapeamento interno)            |

**Configuração CAN (STM32):** PCLK1 = 36 MHz, Prescaler = 4, BS1 = 15 TQ, BS2 = 2 TQ → 500 kbps.

**Conversão de dados (raw → engenharia):**
- Aceleração: valor raw é inteiro com sinal de 16 bits, dividido por 16384 → unidade em *g* (fundo de escala ±2 g).
- Velocidade angular: valor raw de 16 bits com sinal, dividido por 131 → unidade em °/s (fundo de escala ±250 °/s).
- Pedais Hall: `porcentagem = (raw × 99.0) / 4095.0` → 0 a 99%.

---

## node_Acc — Nó Inercial (IMU)

**Hardware:** STM32F103RB + MPU-9250 (compatível com MPU-6500/9250/9255 via detecção automática por `WHO_AM_I`).

**Periféricos:**
- I²C Fast Mode 400 kHz: SCL = PB6, SDA = PB7
- CAN 500 kbps: RX = PA11, TX = PA12
- LED diagnóstico: PC13 (ativo em nível baixo)

**Fluxo de operação:**
1. Inicializa HAL, I²C e CAN.
2. Lê registrador `WHO_AM_I` do sensor; verifica identidade.
3. Configura sensor: acelerômetro ±2 g, giroscópio ±250 °/s.
4. Loop a 50 Hz:
   - Leitura burst I²C a partir do registrador `0x3B` (14 bytes: 6 acel + 2 temp + 6 giro).
   - Monta frame `0x201` (DLC 8) com acel X/Y/Z + giro X.
   - Monta frame `0x204` (DLC 4) com giro Y/Z.
   - Transmite ambos pelo CAN.

**Diagnóstico por flashes no LED PC13:**

| Flashes | Causa                          |
|---------|--------------------------------|
| 1       | `WHO_AM_I` não reconhecido     |
| 2       | Falha de comunicação I²C       |
| 3       | Falha na inicialização CAN     |
| 4       | Timeout na transmissão CAN     |

**Projeto:** `node_Acc/teste_IMU.ioc` (abrir com STM32CubeIDE).

---

## node_Analog — Nó de Pedais

**Hardware:** STM32F103RB + dois sensores Hall de posição (0–5 V, nível lógico tolerado pelo ADC do STM32).

**Periféricos:**
- ADC 12 bits, modo scan dual-channel: CH0 = PA0 (acelerador), CH1 = PA1 (freio)
- Clock ADC: APB2/6 = 12 MHz; amostragem: 55,5 ciclos ≈ 4,6 µs por canal
- Filtro anti-ruído: média móvel de 4 amostras por ciclo
- CAN 500 kbps: RX = PA11, TX = PA12
- LED diagnóstico: PC13 (ativo em nível baixo)

**Fluxo de operação:**
1. Inicializa HAL, ADC e CAN.
2. Loop a 50 Hz:
   - Dispara conversão ADC sequencial (CH0 → CH1).
   - Calcula média de 4 leituras para cada canal.
   - Converte para porcentagem: `pct = (raw × 99.0) / 4095.0`.
   - Monta frame `0x202` (DLC 8): bytes 0-1 Hall Acel, bytes 2-3 Hall Freio, bytes 4-7 reservados (0x00).
   - Transmite pelo CAN.

**Diagnóstico por flashes no LED PC13:**

| Flashes | Causa                          |
|---------|--------------------------------|
| 1       | Falha na inicialização ADC     |
| 2       | Timeout/falha na conversão ADC |
| 3       | Falha na inicialização CAN     |
| 4       | Timeout na transmissão CAN     |

**Projeto:** `node_Analog/teste_analog.ioc` (abrir com STM32CubeIDE).

---

## logger_ESP32 — Logger Principal

Firmware de uso em pista. Inclui interface com ECU, display, barra de RPM e LEDs de alerta.

**Hardware:**

| Componente           | Interface | Pinos ESP32                              |
|----------------------|-----------|------------------------------------------|
| MCP2515 (CAN)        | SPI (VSPI)| CS=5, MOSI=23, MISO=19, SCK=18, INT=17  |
| SD Card              | SPI (HSPI)| CS=15, MOSI=13, MISO=12, SCK=14         |
| WS2812B (barra RPM)  | GPIO      | DIO=27                                   |
| TM1637 (display 7-seg)| GPIO     | DIO=26, CLK=25                           |
| LED Tensão Bateria   | GPIO      | 21                                       |
| LED Temperatura Água | GPIO      | 22                                       |
| LED Pressão Óleo     | GPIO      | 32                                       |
| LED Gravando (SD)    | GPIO      | 33                                       |
| Botão de Volta       | GPIO      | 16                                       |

**Arquitetura FreeRTOS (dual-core):**

```
Core 1 — task_can()                   Core 0 — task_sd()
─────────────────────────────          ──────────────────────────
Loop a 50 Hz:                          Loop contínuo:
 • Lê frame MCP2515                     • Consome fila de dados
 • Decodifica por ID (0x201/204/        • Formata linha CSV
   202/1E0)                             • Escreve no SD
 • Atualiza variáveis globais           • Flush forçado a 1 Hz
 • Atualiza barra LEDs (RPM)           
 • Atualiza display TM1637             Main loop (Core 1):
 • Enfileira dados para SD              • Debounce botão de volta
```

**Formato do arquivo CSV:** uma linha por ciclo a 50 Hz contendo todos os sinais decodificados. Novo arquivo criado a cada pressão do botão de volta (marcação de voltas).

**Arquivo:** `logger_ESP32/logger_ESP32.ino` (abrir com Arduino IDE ou PlatformIO).

---

## logger_ESP32_bench — Logger de Bancada

Versão simplificada para validação dos nós Acc e Analog **sem ECU, sem display e sem barra de RPM**. Útil para testes de bancada isolados do sistema completo.

**Diferenças em relação ao logger principal:**

| Recurso                  | logger_ESP32 | logger_ESP32_bench |
|--------------------------|--------------|--------------------|
| ECU PR440 (`0x1E0`)      | ✓            | ✗                  |
| Display TM1637           | ✓            | ✗                  |
| Barra WS2812B (RPM)      | ✓            | ✗                  |
| LEDs de alerta (4×)      | ✓            | ✗                  |
| LED status único (GPIO 33)| ✗           | ✓                  |
| Gravação CSV no SD       | ✓            | ✓                  |
| Nós Acc + Analog         | ✓            | ✓                  |
| Botão de volta           | ✓            | ✓                  |

**Pinagem SPI (MCP2515 e SD):** idêntica ao logger principal.

**Arquivo:** `logger_ESP32_bench/logger_ESP32_bench.ino`.

---

## Ferramentas e Dependências

### STM32 (node_Acc e node_Analog)
- **IDE:** STM32CubeIDE (recomendado 1.13+)
- **HAL:** STM32F1xx HAL Driver (incluído nos projetos)
- **Configuração:** editar via arquivo `.ioc` no CubeMX integrado
- **Gravação:** ST-Link V2 via SWD (SWDIO, SWDCLK, GND, 3V3)

### ESP32 (logger)
- **IDE:** Arduino IDE 2.x ou PlatformIO (VS Code)
- **Board package:** `esp32` by Espressif (≥ 2.0)
- **Bibliotecas Arduino necessárias:**
  - `mcp_can` — comunicação com MCP2515
  - `FastLED` — controle WS2812B
  - `TM1637Display` — display 7 segmentos
  - `SD` (built-in ESP32) — cartão microSD
  - `FreeRTOS` (built-in ESP32)
- **Upload:** USB-UART (cabo USB-C ou adaptador CH340/CP2102), baud 115200

---

## Como Começar

1. **Clonar o repositório:**
   ```bash
   git clone https://github.com/<org>/Firmware-Telemetry-FR2026.git
   ```

2. **Abrir nó STM32:**
   - Abrir STM32CubeIDE → `File > Open Projects from File System` → selecionar pasta `node_Acc` ou `node_Analog`.
   - Conectar ST-Link → `Run > Debug` para gravar e depurar.

3. **Abrir logger ESP32:**
   - Instalar board package ESP32 no Arduino IDE.
   - Instalar bibliotecas listadas acima via `Sketch > Include Library > Manage Libraries`.
   - Abrir `logger_ESP32/logger_ESP32.ino` (ou `_bench` para bancada).
   - Selecionar board `ESP32 Dev Module`, porta COM correta → `Upload`.

4. **Teste de bancada (sem ECU):**
   - Usar `logger_ESP32_bench` para validar apenas os nós Acc e Analog.
   - Monitor serial a 115200 bps mostra dados decodificados em tempo real.

---

## Notas de Hardware

- Todos os nós STM32 operam a **5 V**. O nível lógico CAN é fornecido pelo transceiver (ex.: TJA1050 ou SN65HVD230) conectado aos pinos PA11/PA12.
- O MCP2515 no logger ESP32 opera a **3,3 V** (usar módulo com regulador ou divisor de nível caso o módulo seja de 5 V).
- Garantir terminação de **120 Ω** em ambas as extremidades do barramento CAN.
- O SD card deve ser formatado em **FAT32**.
- Os sensores Hall dos pedais fornecem 0–5 V; verificar divisor resistivo ou proteção de nível antes de conectar ao STM32 (ADC tolerante a 3,3 V máximo em PA0/PA1).
