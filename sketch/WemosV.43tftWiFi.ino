// ======================= WemosV.43tftWiFi LIN + BSD Integration (fixed) =========by DYNASTARTER==============
// Полная версия с исправленной многопоточностью Wi-Fi/AP (taskWifiControl на Core0)
// и устойчивым приёмом LIN. Готово к компиляции.

// ---- Feature toggles ----
#define USE_DISPLAY 1
#define USE_WEB 1

#ifdef min
#undef min
#endif
#include <algorithm>
using std::min;

#include <Arduino.h>
#include "esp_wifi.h"  // esp_wifi_set_ps
#include <SPI.h>
#include <TFT_eSPI.h>

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "Free_Fonts.h"


#if USE_WEB
extern const char MAIN_HTML[] PROGMEM;
#endif

// =========================== BSD INTEGRATION ===========================
#define BSD_TX_PIN 17
#define BSD_RX_PIN 16

const unsigned long BSD_FREQ_HZ = 1300;
const unsigned long BSD_periodUs = 1000000UL / BSD_FREQ_HZ;
const unsigned long BSD_SYNC_PULSE = 140;
const unsigned long BSD_ZERO_PULSE = 270;  //280
const unsigned long BSD_ONE_PULSE = 550;   //560
const unsigned long BSD_DELAY_BETWEEN_REG_US = 80000;
const unsigned long BSD_DELAY_BETWEEN_GROUP_MS = 177;

const int BSD_SYNC_MIN = 85, BSD_SYNC_MAX = 180;   // короткий импульс
const int BSD_BIT0_MIN = 200, BSD_BIT0_MAX = 420;  // LOW-фаза как 0
const int BSD_BIT1_MIN = 480, BSD_BIT1_MAX = 900;  // HIGH-фаза как 1

struct BSDPulse {
  uint16_t low_us;
  uint16_t s2s_us;
};
#define BSD_RING_SZ 256
volatile BSDPulse bsd_ring[BSD_RING_SZ];
volatile uint16_t bsd_widx = 0;
volatile uint32_t bsd_lastFall = 0;
volatile uint32_t bsd_prevFall = 0;

// Кадры BSD (19 бит)
const uint8_t bsd_frame_reg7[19] = { 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t bsd_frame_reg6[19] = { 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t bsd_frame_reg2[19] = { 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t bsd_header_reg0_write[9] = { 0, 1, 1, 0, 1, 0, 0, 0, 1 };

volatile uint8_t bsd_writeData = 0x18;
uint8_t bsd_reg0_value = 0;
uint8_t bsd_reg2_value = 0;
uint8_t bsd_reg6_value = 0;
bool bsd_deviceFound = false;

int bsd_missCount = 0;
const int BSD_MAX_MISSES = 1;

// PATCH BEGIN: protocol hysteresis
const uint8_t PROTO_CONFIDENCE_UP = 2;    // сколько подряд «успехов» для входа в режим
const uint8_t PROTO_CONFIDENCE_DOWN = 2;  // сколько подряд «промахов» для выхода
const uint32_t PROTO_MIN_HOLD_MS = 600;   // минимум удержания режима

static uint8_t protoOkStreak = 0;
static uint8_t protoMissStreak = 0;
static uint32_t protoModeSinceMs = 0;
// PATCH END

// === Классификация импульсов BSD без String ===
enum BsdClass : uint8_t { BSD_BAD = 0,
                          BSD_SYNC = 1,
                          BSD_BIT0 = 2,
                          BSD_BIT1 = 3 };

// =========================== PROTOCOL MGMT ===========================
enum ProtocolMode { PROTOCOL_SEARCHING,
                    PROTOCOL_LIN,
                    PROTOCOL_BSD };
volatile ProtocolMode currentProtocol = PROTOCOL_SEARCHING;
volatile bool searchingLIN = true;

// =========================== WiFi / Web ===========================
const char *ssid = "LIN";
const char *password = "12345678";
AsyncWebServer server(80);

// ЕДИНАЯ модель управления через очередь
enum WifiCmd : uint8_t { WIFI_CMD_START = 1,
                         WIFI_CMD_STOP = 2 };
static QueueHandle_t wifiQ = nullptr;
static volatile bool wifiActive = false;  // текущее состояние AP
static volatile bool wifiBusy = false;    // сервис что-то делает (для защиты от дребезга)
static uint32_t wifiLastToggleMs = 0;
const uint32_t WIFI_TOGGLE_GUARD_MS = 1000;

// Прототипы
static void wifi_task(void *pv);
enum UiCmd : uint8_t { UI_VOLT_UP,
                       UI_VOLT_DOWN,
                       UI_START };
static QueueHandle_t uiQ = nullptr;

// JSON буфер для /data
static char g_jsonBuf[4096];
static portMUX_TYPE jsonMux = portMUX_INITIALIZER_UNLOCKED;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite devListSprite = TFT_eSprite(&tft);
TFT_eSprite messageSprite = TFT_eSprite(&tft);
TFT_eSprite voltageSprite = TFT_eSprite(&tft);

uint16_t colorL = TFT_VIOLET;
uint16_t colorM = TFT_CYAN;
uint16_t colorH = TFT_MAGENTA;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

const int headerHeight = 60;
const int voltHeight = 60;
const int msgHeight = 30;
const int listYStart = 150;

int headerX = 0;
int voltX = 0;

// =========================== Encoder & Buttons ===========================
#define CLK 14
#define DT 27
#define SW 22
#define WIFI_BTN_PIN 21

volatile bool buttonHeld = false;

static bool wifiBtnIdleLevel = HIGH;
struct Btn {
  uint8_t pin;
  bool last = true;
  bool level = true;
  uint32_t tEdgeMs = 0;
  bool clicked = false;
  bool held = false;
};

inline void pollButton(Btn &b) {
  bool raw = digitalRead(b.pin);
  uint32_t now = millis();
  if (raw != b.level) {
    b.level = raw;
    b.tEdgeMs = now;
  }
  const uint32_t DEBOUNCE = 25, HOLD_MS = 2000;
  if (now - b.tEdgeMs >= DEBOUNCE) {
    if (b.last && !b.level) {
      b.clicked = false;
      b.held = false;
      b.last = b.level;
      b.tEdgeMs = now;
    }
    if (!b.last && !b.level && !b.held && (now - b.tEdgeMs >= HOLD_MS)) b.held = true;
    if (!b.last && b.level) {
      if (!b.held) b.clicked = true;
      b.last = b.level;
    }
  }
}

// Квадратурный энкодер
volatile uint8_t encState = 0;
volatile int16_t encDelta = 0;
#define ENC_STEPS_PER_NOTCH 4
static const int8_t encLut[16] = {
  0, -1, +1, 0, +1, 0, 0, -1, -1, 0, 0, +1, 0, +1, -1, 0
};
void IRAM_ATTR enc_isr_any() {
  static uint32_t lastUs = 0;
  uint32_t now = micros();
  if ((uint32_t)(now - lastUs) < 120) return;
  lastUs = now;
  uint8_t s = (digitalRead(CLK) << 1) | digitalRead(DT);
  uint8_t idx = (encState << 2) | s;
  encDelta += encLut[idx];
  encState = s;
}

volatile bool startFlag = false;
volatile bool webStartRequested = false;

// =========================== LIN PORT ===========================
//#define USE_SERIAL2
#define HARDWARE_SERIAL Serial2
#define TX_PIN 17
#define RX_PIN 16

// PATCH BEGIN: line arbiter for shared pin
enum LineMode : uint8_t { LINE_HI_Z,
                          LINE_LIN,
                          LINE_BSD };
static SemaphoreHandle_t lineMtx = nullptr;
static volatile LineMode g_lineMode = LINE_HI_Z;
static volatile long g_linLastBaud = 9600;

static inline void line_init() {
  lineMtx = xSemaphoreCreateMutex();
  pinMode(TX_PIN, INPUT);         // Hi-Z по умолчанию
  pinMode(RX_PIN, INPUT_PULLUP);  // RX подтянут
}

static void line_acquire(LineMode m, long baudIfLin = 9600) {
  xSemaphoreTake(lineMtx, portMAX_DELAY);

  // Небольшая «тишина» перед переключением
  delayMicroseconds(300);

  // Гасим предыдущий режим
  if (g_lineMode == LINE_LIN) {
    HARDWARE_SERIAL.flush();
    HARDWARE_SERIAL.end();
  } else if (g_lineMode == LINE_BSD) {
    bsd_end_rx();
    bsd_disableTransmitter();  // TX -> INPUT
  }
  pinMode(TX_PIN, INPUT);  // гарантированный Hi-Z
  delayMicroseconds(50);

  // Включаем нужный
  if (m == LINE_LIN) {
    g_lineMode = LINE_LIN;
  } else if (m == LINE_BSD) {
    bsd_disableTransmitter();
    bsd_begin_rx();                  // ISR на вход
    pinMode(BSD_TX_PIN, OUTPUT);     // общий TX=17 в OUTPUT
    digitalWrite(BSD_TX_PIN, HIGH);  // «единица» по умолчанию
    g_lineMode = LINE_BSD;
  } else {
    g_lineMode = LINE_HI_Z;
  }

  delayMicroseconds(80);  // время на стабилизацию
}

static inline void line_release() {
  xSemaphoreGive(lineMtx);
}
// Виртуальные напряжения
int voltLIN1[] = {
  0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
  0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C
};
int voltLIN2[] = {
  0x3C, 0x40, 0x44, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C, 0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 0x78,
  0x7C, 0x80, 0x84, 0x88, 0x8C, 0x90, 0x94, 0x98, 0x9C, 0xA0, 0xA4, 0xA8, 0xAC, 0xB0
};
const char *setVOLT[] = {
  "12.1", "12.2", "12.3", "12.4", "12.5", "12.6", "12.7", "12.8", "12.9", "13.0",
  "13.1", "13.2", "13.3", "13.4", "13.5", "13.6", "13.7", "13.8", "13.9", "14.0",
  "14.1", "14.2", "14.3", "14.4", "14.5", "14.6", "14.7", "14.8", "14.9", "15.0"
};
volatile int valueLIN = 9;

byte myDataE9[4] = { 0x00, 0x00, 0x00, 0x00 };
byte myDataEC[5] = { 0x00, 0xC7, 0xA8, 0x8C, 0x13 };

struct Device {
  byte pid;
  long baudRate;
  bool enhanced;
  int dataLength;
  byte data[8];
  byte checksum;
  bool responseL;
  bool responseM;
  bool responseH;
};
#define MAX_DEVICES 10
Device devices[MAX_DEVICES];
int deviceCount = 0;

bool deviceExists(byte pid) {
  for (int i = 0; i < deviceCount; i++)
    if (devices[i].pid == pid) return true;
  return false;
}

volatile bool searchMode = true;
int missCount = 0;
const int MAX_MISSES = 2;

// ===== Тестовый PID (отключён 0x00) =====
byte TestPID = 0x00;
bool TestPID_UseE9 = true;

const byte PID_ARRAY[] = {
  0x11, 0x14, 0x55, 0x61, 0x73, 0x80,
  0x92, 0x97, 0xA8, 0xD3, 0xD6, 0xD8,
  0xE7, 0xE9, 0xEC, 0xF0
};

const long BAUD_RATES[] = { 2800, 9600, 19200 };
volatile long detectedBaudRate = 9600;

byte linPID = 0;
byte linData[8];
volatile byte linChecksum = 0;
volatile int linLength = 0;

volatile bool isClassicValid = false;
volatile bool isEnhancedValid = false;

volatile bool responseL = false;
volatile bool responseM = false;
volatile bool responseH = false;

volatile bool forceHeaderReset = false;

byte linBuffer[11];

long baudRateE9 = 9600;
bool enhancedChecksumE9 = false;
long baudRateEC = 9600;
bool enhancedChecksumEC = false;

volatile bool sendMode = true;

char messageLine3[64] = "";

// защитный спинлок для JSON снапшота исходящих LIN
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
//static SemaphoreHandle_t uartMtx = nullptr;
portMUX_TYPE linOutMux = portMUX_INITIALIZER_UNLOCKED;

// =========================== BSD FUNCTIONS ===========================
void IRAM_ATTR bsd_onChange() {
  bool lvl = digitalRead(BSD_RX_PIN);
  uint32_t now = micros();
  if (!lvl) {
    bsd_lastFall = now;
  } else {
    if (bsd_lastFall) {
      uint16_t low = (uint16_t)(now - bsd_lastFall);
      uint16_t s2s = bsd_prevFall ? (uint16_t)(bsd_lastFall - bsd_prevFall) : 0xFFFF;
      bsd_prevFall = bsd_lastFall;
      uint16_t i = bsd_widx & (BSD_RING_SZ - 1);
      bsd_ring[i].low_us = low;
      bsd_ring[i].s2s_us = s2s;
      bsd_widx = (i + 1) & (BSD_RING_SZ - 1);
      bsd_lastFall = 0;
    }
  }
}
static inline void bsd_begin_rx() {
  pinMode(BSD_RX_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BSD_RX_PIN), bsd_onChange, CHANGE);
}
static inline void bsd_end_rx() {
  detachInterrupt(digitalPinToInterrupt(BSD_RX_PIN));
}

static inline BsdClass bsd_classify(uint16_t low) {
  if (low >= BSD_SYNC_MIN && low <= BSD_SYNC_MAX) return BSD_SYNC;
  if (low >= BSD_BIT0_MIN && low <= BSD_BIT0_MAX) return BSD_BIT0;
  if (low >= BSD_BIT1_MIN && low <= BSD_BIT1_MAX) return BSD_BIT1;
  return BSD_BAD;
}

void bsd_decodeBits(uint8_t *bits, uint8_t n) {
  if (n < 19) return;
  uint8_t reg = (bits[5] << 2) | (bits[6] << 1) | bits[7];
  uint8_t data = 0;
  for (int i = 0; i < 8; i++) data |= bits[9 + i] << (7 - i);

  uint8_t received_parity = bits[17];
  uint8_t calculated_parity = 0;
  for (int i = 9; i < 17; i++) calculated_parity ^= bits[i];
  if (received_parity != calculated_parity) return;

  switch (reg) {
    case 0: bsd_reg0_value = data; break;
    case 2: bsd_reg2_value = data; break;
    case 6: bsd_reg6_value = data; break;
  }
}
static bool bsd_rx_service_once() {
  static uint16_t rpos = 0;
  uint8_t bits[32];
  uint8_t bcount = 0;
  bool any = false;

  while (rpos != bsd_widx) {
    uint16_t i = rpos & (BSD_RING_SZ - 1);
    uint16_t low = bsd_ring[i].low_us;
    uint16_t s2s = bsd_ring[i].s2s_us;
    rpos = (i + 1) & (BSD_RING_SZ - 1);

    BsdClass c = bsd_classify(low);
    if (c == BSD_BAD || s2s > 1000) {
      if (bcount > 0) {
        bsd_decodeBits(bits, bcount);
        any = true;
        bcount = 0;
      }
    } else {
      if (c == BSD_BIT0 || c == BSD_BIT1) {
        if (bcount < sizeof(bits)) bits[bcount++] = (c == BSD_BIT1);
      }
    }
  }
  return any;
}

void bsd_enableTransmitter() {
  pinMode(BSD_TX_PIN, OUTPUT);
  digitalWrite(BSD_TX_PIN, HIGH);
}
void bsd_disableTransmitter() {
  pinMode(BSD_TX_PIN, INPUT);
}
void bsd_sendPulse(unsigned long lowUs) {
  digitalWrite(BSD_TX_PIN, LOW);
  delayMicroseconds(lowUs);
  digitalWrite(BSD_TX_PIN, HIGH);
  delayMicroseconds(BSD_periodUs - lowUs);
}
void bsd_sendSync() {
  bsd_sendPulse(BSD_SYNC_PULSE);
}
void bsd_sendBit(bool b) {
  bsd_sendPulse(b ? BSD_ONE_PULSE : BSD_ZERO_PULSE);
}
void bsd_buildWriteFrame(uint8_t data, uint8_t *frame) {
  for (int i = 0; i < 9; i++) frame[i] = bsd_header_reg0_write[i];
  for (int i = 0; i < 8; i++) frame[9 + i] = (data >> (7 - i)) & 1;
  uint8_t parity = 0;
  for (int i = 9; i < 17; i++) parity ^= frame[i];
  frame[17] = parity;
  frame[18] = 0;
}
void bsd_sendFrame(const uint8_t *frame) {
  bsd_enableTransmitter();
  for (int i = 0; i < 5; i++) bsd_sendSync();
  for (int i = 0; i < 19; i++) bsd_sendBit(frame[i]);
  bsd_sendBit(0);
  bsd_disableTransmitter();
}

static void bsd_tx_reads_and_write(uint8_t writeData, bool useCmdTiming) {
  bsd_sendFrame(bsd_frame_reg7);
  delayMicroseconds(BSD_DELAY_BETWEEN_REG_US);
  bsd_sendFrame(bsd_frame_reg6);
  delayMicroseconds(BSD_DELAY_BETWEEN_REG_US);
  bsd_sendFrame(bsd_frame_reg2);
  delayMicroseconds(BSD_DELAY_BETWEEN_REG_US);
  uint8_t frameW0[19];
  bsd_buildWriteFrame(writeData, frameW0);
  bsd_sendFrame(frameW0);
  (void)useCmdTiming;
}
bool bsd_tryDevice() {
  line_acquire(LINE_BSD);  // PATCH
                           // bsd_begin_rx();
  bsd_tx_reads_and_write(bsd_writeData, true);
  unsigned long t0 = micros();
  bool got = false;
  while ((micros() - t0) < 50000) {
    if (bsd_rx_service_once()) got = true;
    taskYIELD();
  }
  // bsd_end_rx();
  line_release();
  return got;
}

// =========================== LIN FUNCTIONS ===========================
byte calculateChecksum(byte data[], int length, bool isEnhanced) {
  uint16_t sum = 0;
  int startIndex = isEnhanced ? 1 : 2;
  for (int i = startIndex; i < length - 1; i++) {
    sum += data[i];
    if (sum > 0xFF) sum = (sum & 0xFF) + 1;
  }
  return (byte)(~sum);
}

void sendBreak(long baudRate) {
  HARDWARE_SERIAL.flush();
  HARDWARE_SERIAL.end();
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);

  unsigned int breakBits;
  if (baudRate <= 2800) breakBits = 16;
  else if (baudRate <= 9600) breakBits = 14;
  else breakBits = 13;

  unsigned long breakDuration = (1000000UL * breakBits) / baudRate;
  delayMicroseconds(breakDuration);

  digitalWrite(TX_PIN, HIGH);  // delimiter (>= 1 бит)
  delayMicroseconds(1000000UL / baudRate);

  HARDWARE_SERIAL.begin(baudRate, SERIAL_8N1, RX_PIN, TX_PIN);
  HARDWARE_SERIAL.write(0x55);
  HARDWARE_SERIAL.flush();
}

void sendLINMessage(byte pid, long baudRate) {

  sendBreak(baudRate);
  HARDWARE_SERIAL.write(pid);
  HARDWARE_SERIAL.flush();
  // xSemaphoreGive(uartMtx);
  taskYIELD();
}

// === УСТОЙЧИВЫЙ ПРИЁМ LIN ===
void receiveLINData(long baudRate) {

  HARDWARE_SERIAL.updateBaudRate(baudRate);

  const uint32_t bitUs = (1000000UL + baudRate / 2) / baudRate;
  const uint32_t byteUs = 10 * bitUs;  // 8N1 ≈ 10 бит
  const uint32_t tmoUs = 8 * byteUs;   // окно ~8 байт

  linLength = 0;
  uint32_t t0 = micros();

  while ((uint32_t)(micros() - t0) < tmoUs && linLength < 11) {
    int c = HARDWARE_SERIAL.read();
    if (c >= 0) {
      if (linLength < (int)sizeof(linBuffer)) linBuffer[linLength++] = (uint8_t)c;
      t0 = micros();  // продлеваем окно, пока текут байты
    } else {
      delayMicroseconds(50);  // дать UARTу качнуть DMA
    }
  }
  if (linLength < 3) {
    linLength = 0;
    return;
  }

  linPID = linBuffer[1];
  linChecksum = linBuffer[linLength - 1];
  int payloadLen = linLength - 3;
  for (int i = 0; i < payloadLen && i < 8; i++) linData[i] = linBuffer[i + 2];

  byte expectedClassic = calculateChecksum(linBuffer, linLength, false);
  byte expectedEnhanced = calculateChecksum(linBuffer, linLength, true);

  isClassicValid = (linChecksum == expectedClassic);
  isEnhancedValid = (linChecksum == expectedEnhanced);


  if (linPID == 0x92) {
    baudRateE9 = baudRate;
    enhancedChecksumE9 = isEnhancedValid;
  }
  if (linPID == 0xD6) {
    baudRateEC = baudRate;
    enhancedChecksumEC = isEnhancedValid;
  }

  if (linLength > 2) {
    if (baudRate == 19200) responseH = true;
    if (baudRate == 9600) responseM = true;
    if (baudRate == 2800) responseL = true;
  }
}
void updateMyDataE9() {
  myDataE9[0] = enhancedChecksumE9 ? voltLIN2[valueLIN] : voltLIN1[valueLIN];
  myDataE9[1] = 0x00;
  myDataE9[2] = 0xFF;
  myDataE9[3] = 0x00;
}


void updateMyDataEC() {
  if (enhancedChecksumEC) {
    myDataEC[0] = voltLIN2[valueLIN];
    myDataEC[1] = 0x00;
    myDataEC[2] = 0x55;
    myDataEC[3] = buttonHeld ? 0x50 : 0x8C;
    myDataEC[4] = 0x13;
  } else {
    myDataEC[0] = voltLIN2[valueLIN];
    myDataEC[1] = 0xC7;
    myDataEC[2] = 0xA8;
    myDataEC[3] = buttonHeld ? 0xDA : 0x8C;
  }
}

void sendMessageE9(const byte data[4]) {
  sendBreak(baudRateE9);
  byte frame[7];
  frame[0] = 0x55;
  frame[1] = 0xE9;
  for (int i = 0; i < 4; i++) frame[i + 2] = data[i];
  byte checksum = calculateChecksum(frame, 7, enhancedChecksumE9);
  HARDWARE_SERIAL.write(0xE9);
  for (int i = 0; i < 4; i++) HARDWARE_SERIAL.write(data[i]);
  HARDWARE_SERIAL.write(checksum);
  HARDWARE_SERIAL.flush();
  char tmp[16];
  strcpy(messageLine3, "E9: ");
  for (int i = 0; i < 4; i++) {
    sprintf(tmp, "%X ", data[i]);
    strcat(messageLine3, tmp);
  }
  sprintf(tmp, "C:%X", checksum);
  strcat(messageLine3, tmp);
}

void sendMessageEC(const byte data[5]) {
  if (enhancedChecksumEC) {
    byte frame[8];
    frame[0] = 0x55;
    frame[1] = 0xEC;
    for (int i = 0; i < 5; i++) frame[i + 2] = data[i];
    byte checksum = calculateChecksum(frame, 8, true);
    sendBreak(baudRateEC);
    HARDWARE_SERIAL.write(0xEC);

    for (int i = 0; i < 5; i++) HARDWARE_SERIAL.write(data[i]);
    HARDWARE_SERIAL.write(checksum);
    HARDWARE_SERIAL.flush();
    startFlag = (data[3] == 0xDA || data[3] == 0x50);
    char tmp[16];
    strcpy(messageLine3, "EC: ");
    for (int i = 0; i < 5; i++) {
      sprintf(tmp, "%X ", data[i]);
      strcat(messageLine3, tmp);
    }
    sprintf(tmp, "C:%X", checksum);
    strcat(messageLine3, tmp);
  } else {
    byte frame[7];
    frame[0] = 0x55;
    frame[1] = 0xEC;
    for (int i = 0; i < 4; i++) frame[i + 2] = data[i];
    byte checksum = calculateChecksum(frame, 7, false);
    sendBreak(baudRateEC);
    HARDWARE_SERIAL.write(0xEC);
    for (int i = 0; i < 4; i++) HARDWARE_SERIAL.write(data[i]);
    HARDWARE_SERIAL.write(checksum);
    HARDWARE_SERIAL.flush();
    startFlag = (data[3] == 0xDA || data[3] == 0x50);
    char tmp[16];
    strcpy(messageLine3, "EC: ");
    for (int i = 0; i < 4; i++) {
      sprintf(tmp, "%X ", data[i]);
      strcat(messageLine3, tmp);
    }
    sprintf(tmp, "C:%X", checksum);
    strcat(messageLine3, tmp);
  }
}

void handleVoltUp() {
  if (valueLIN < 29) {
    valueLIN++;
    if (currentProtocol == PROTOCOL_BSD) bsd_writeData = valueLIN + 0x0F;
  }
}
void handleVoltDown() {
  if (valueLIN > 0) {
    valueLIN--;
    if (currentProtocol == PROTOCOL_BSD) bsd_writeData = valueLIN + 0x0F;
  }
}

// =========================== DISPLAY ===========================
void updateHeader() {
  static ProtocolMode lastDrawnProtocol = (ProtocolMode)-1;
  static String lastIdText = "";
  static bool lastResponseL = false, lastResponseM = false, lastResponseH = false;
  static String lastVerText = "";

  if (forceHeaderReset) {
    lastIdText = "";
    lastResponseL = lastResponseM = lastResponseH = false;
    lastVerText = "";
    forceHeaderReset = false;
  }

  if (lastDrawnProtocol != currentProtocol) {
    tft.fillRect(0, 0, SCREEN_WIDTH, headerHeight, TFT_BLACK);
    tft.setFreeFont(FSB24);
    tft.setTextDatum(TL_DATUM);

    String headerText;
    switch (currentProtocol) {
      case PROTOCOL_LIN:
        tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
        headerText = "LIN: ";
        break;
      case PROTOCOL_BSD:
        tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        headerText = "BSS: ";
        break;
      default:
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        headerText = "COM";
        break;
    }
    tft.drawString(headerText, 0, 0, GFXFF);
    headerX = tft.textWidth(headerText, GFXFF);
    lastDrawnProtocol = currentProtocol;
    lastIdText = "";
    lastResponseL = lastResponseM = lastResponseH = false;
    lastVerText = "";
  }

  if (currentProtocol != PROTOCOL_SEARCHING) {
    String idText = "   ";
    String verText = "";
    bool valid = false;

    if (currentProtocol == PROTOCOL_LIN) {
      valid = (linPID == 0x92 || linPID == 0xD6 || (linPID == TestPID && TestPID != 0x00))
              && (linLength > 3 && (isClassicValid || isEnhancedValid));
      if (valid) idText = String(linData[0]);
      verText = isClassicValid ? " V.1.x" : " V.2.x";
    } else if (currentProtocol == PROTOCOL_BSD) {
      valid = bsd_deviceFound && (bsd_reg6_value > 0);
      if (valid) idText = String(bsd_reg6_value);
      verText = "   V.BSD";
    }

    bool needsRedraw = (idText != lastIdText || responseL != lastResponseL || responseM != lastResponseM || responseH != lastResponseH || verText != lastVerText);
    if (needsRedraw) {
      int x = headerX;
      tft.fillRect(x, 0, SCREEN_WIDTH - x, headerHeight, TFT_BLACK);
      tft.setFreeFont(FSB24);
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString(idText, x, 0, GFXFF);
      x += tft.textWidth(idText, GFXFF);

      if (valid && currentProtocol == PROTOCOL_LIN) {
        tft.setFreeFont(FSB18);
        if (responseL) {
          tft.setTextColor(colorL, TFT_BLACK);
          tft.drawString("L", x, 10, GFXFF);
          x += tft.textWidth("L", GFXFF);
        }
        if (responseM) {
          tft.setTextColor(colorM, TFT_BLACK);
          tft.drawString("M", x, 10, GFXFF);
          x += tft.textWidth("M", GFXFF);
        }
        if (responseH) {
          tft.setTextColor(colorH, TFT_BLACK);
          tft.drawString("H", x, 10, GFXFF);
          x += tft.textWidth("H", GFXFF);
        }
      }

      tft.setFreeFont(FSB9);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString(verText, x, 22, GFXFF);

      lastIdText = idText;
      lastResponseL = responseL;
      lastResponseM = responseM;
      lastResponseH = responseH;
      lastVerText = verText;
    }
  }

  // WiFi статус
  tft.setFreeFont(FSB9);
  tft.setTextDatum(TR_DATUM);
  if (wifiActive) {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("WiFi", SCREEN_WIDTH - 4, 4, GFXFF);
  } else {
    int w = tft.textWidth("WiFi", GFXFF) + 6;
    tft.fillRect(SCREEN_WIDTH - w, 0, w, headerHeight, TFT_BLACK);
  }
}

void updateMessage() {
  static String lastMessage = "";
  static ProtocolMode lastProtocol = (ProtocolMode)-1;

  String msg = "";
  if (currentProtocol == PROTOCOL_BSD) {
    char bsd_msg[64];
    sprintf(bsd_msg, "BSS: REG0:%d REG6:%d ", bsd_writeData, bsd_reg6_value);
    msg = String(bsd_msg);
  } else if (currentProtocol == PROTOCOL_LIN) {
    msg = String(messageLine3);
  }
  if (msg == lastMessage && currentProtocol == lastProtocol) return;

  messageSprite.fillSprite(TFT_BLACK);
  if (msg.length() > 0) messageSprite.drawString(msg, 0, 0);
  messageSprite.pushSprite(0, headerHeight + voltHeight);

  lastMessage = msg;
  lastProtocol = currentProtocol;
}

void updateDeviceList() {
  static String lastDeviceData = "";
  static ProtocolMode lastProtocol = (ProtocolMode)-1;

  String currentData = "";
  if (currentProtocol == PROTOCOL_BSD) {
    currentData = "BSD:" + String(bsd_reg0_value) + ":" + String(bsd_reg6_value) + ":" + String(bsd_writeData, HEX);
  } else if (currentProtocol == PROTOCOL_LIN) {
    currentData = "LIN:" + String(deviceCount);
    for (int i = 0; i < min(deviceCount, 4); i++) {
      currentData += ":" + String(devices[i].pid, HEX);
      for (int j = 0; j < devices[i].dataLength; j++) currentData += String(devices[i].data[j], HEX);
      currentData += String(devices[i].checksum, HEX);
    }
  }
  if (currentData == lastDeviceData && currentProtocol == lastProtocol) return;

  devListSprite.fillSprite(TFT_BLACK);

  if (currentProtocol == PROTOCOL_BSD && currentData.length() > 0) {
    int y = 0;
    char buf[64];
    sprintf(buf, "R6:%d(0x%02X)", bsd_reg6_value, bsd_reg6_value);
    devListSprite.drawString(buf, 0, y);
    y += devListSprite.fontHeight();
    sprintf(buf, "R2:%d(0x%02X)", bsd_reg2_value, bsd_reg2_value);
    devListSprite.drawString(buf, 0, y);
    y += devListSprite.fontHeight();
    sprintf(buf, "R0:%d(0x%02X)", bsd_writeData, bsd_writeData);
    devListSprite.drawString(buf, 0, y);
    y += devListSprite.fontHeight();
    sprintf(buf, "W:0x%02X(%d)", bsd_writeData, bsd_writeData);
    devListSprite.drawString(buf, 0, y);
  } else if (currentProtocol == PROTOCOL_LIN && deviceCount > 0) {
    int y = 0;
    int count = min(deviceCount, 4);
    for (int i = 0; i < count; i++) {
      String line = "P" + String(devices[i].pid, HEX) + "|D";
      for (int j = 0; j < devices[i].dataLength; j++) line += "|" + String(devices[i].data[j], HEX);
      line += "|C" + String(devices[i].checksum, HEX);
      devListSprite.drawString(line, 0, y);
      y += devListSprite.fontHeight();
    }
  }
  devListSprite.pushSprite(0, listYStart);

  lastDeviceData = currentData;
  lastProtocol = currentProtocol;
}

void updateVoltage() {
  int y = headerHeight;
  static bool vPrinted = false;
  static int vX = 0;
  static String lastVolt = "";
  static ProtocolMode lastProtocol = (ProtocolMode)-1;

  String currentVolt = "";
  if (currentProtocol == PROTOCOL_BSD) currentVolt = String(setVOLT[valueLIN]);
  else if (currentProtocol == PROTOCOL_LIN) {
    currentVolt = String(setVOLT[valueLIN]);
    if (startFlag) currentVolt += " Start";
  }
  if (currentVolt == lastVolt && currentProtocol == lastProtocol) return;

  if (currentProtocol == PROTOCOL_SEARCHING && lastProtocol != PROTOCOL_SEARCHING) {
    tft.fillRect(0, y, SCREEN_WIDTH, voltHeight, TFT_BLACK);
    vPrinted = false;
    vX = 0;
    lastVolt = "";
    lastProtocol = PROTOCOL_SEARCHING;
    return;
  }

  if (currentProtocol != PROTOCOL_SEARCHING) {
    if (!vPrinted) {
      tft.setFreeFont(FSB24);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setTextDatum(TL_DATUM);
      tft.drawString("Volt:", 0, y, GFXFF);
      vX = tft.textWidth("Volt:", GFXFF) + 6;
      vPrinted = true;
    }
    voltageSprite.fillSprite(TFT_BLACK);
    if (currentVolt.length() > 0) voltageSprite.drawString(currentVolt, 0, 0);
    voltageSprite.pushSprite(vX, y);
  }
  lastVolt = currentVolt;
  lastProtocol = currentProtocol;
}

// =========================== Detect LIN Baud ===========================
void detectLINBaudRate() {
  for (long baud : BAUD_RATES) {
    HARDWARE_SERIAL.begin(baud);
    if (HARDWARE_SERIAL.available()) {
      detectedBaudRate = baud;
      break;
    }
  }
}

// =========================== WEB JSON ===========================
String createJson() {
  // Снимок исходящих
  byte e9DataSnap[4], ecDataSnap[5];
  bool e9EnhancedSnap = false, ecEnhancedSnap = false;
  bool startSnap = false;

  int valueLINSnap = valueLIN;
  long detectedBaudSnap = detectedBaudRate;

  portENTER_CRITICAL(&linOutMux);
  memcpy(e9DataSnap, myDataE9, sizeof(e9DataSnap));
  memcpy(ecDataSnap, myDataEC, sizeof(ecDataSnap));
  e9EnhancedSnap = enhancedChecksumE9;
  ecEnhancedSnap = enhancedChecksumEC;
  startSnap = startFlag;
  portEXIT_CRITICAL(&linOutMux);

  byte checksumE9 = 0, checksumEC = 0;
  {
    byte f[7] = { 0x55, 0xE9, e9DataSnap[0], e9DataSnap[1], e9DataSnap[2], e9DataSnap[3] };
    checksumE9 = calculateChecksum(f, 7, e9EnhancedSnap);
  }
  if (ecEnhancedSnap) {
    byte f[8] = { 0x55, 0xEC, ecDataSnap[0], ecDataSnap[1], ecDataSnap[2], ecDataSnap[3], ecDataSnap[4] };
    checksumEC = calculateChecksum(f, 8, true);
  } else {
    byte f[7] = { 0x55, 0xEC, ecDataSnap[0], ecDataSnap[1], ecDataSnap[2], ecDataSnap[3] };
    checksumEC = calculateChecksum(f, 7, false);
  }

  const char *protocolStr =
    (currentProtocol == PROTOCOL_LIN) ? "LIN" : (currentProtocol == PROTOCOL_BSD) ? "BSD"
                                                                                  : "SEARCHING";

  int idByte = -1, speed = 0;
  const char *idLetter = "";
  const char *version = "N/A";

  if (currentProtocol == PROTOCOL_LIN) {
    for (int i = 0; i < deviceCount; i++) {
      if (devices[i].pid == 0x92 || devices[i].pid == 0xD6 || (devices[i].pid == TestPID && TestPID != 0x00)) {
        if (devices[i].dataLength > 0) {
          idByte = devices[i].data[0];
          speed = (int)devices[i].baudRate;
          if (speed == 19200) idLetter = "H";
          else if (speed == 9600) idLetter = "M";
          else if (speed == 2800) idLetter = "L";
          version = devices[i].enhanced ? "LIN 2.x" : "LIN 1.x";
          break;
        }
      }
    }
  } else if (currentProtocol == PROTOCOL_BSD) {
    if (bsd_deviceFound && bsd_reg6_value > 0) {
      idByte = bsd_reg6_value;
      version = "BSD 1.3kHz";
      speed = 1300;
    }
  }

  const char *voltage = setVOLT[valueLINSnap];

  static char json[4096];
  int len = 0;
  len += snprintf(json + len, sizeof(json) - len,
                  "{"
                  "\"protocol\":\"%s\","
                  "\"id\":%d,"
                  "\"idLetter\":\"%s\","
                  "\"version\":\"%s\","
                  "\"speed\":%d,"
                  "\"voltage\":\"%s%s\","
                  "\"flags\":{"
                  "\"start\":%s,"
                  "\"sendMode\":%s,"
                  "\"searchMode\":%s,"
                  "\"detectedBaud\":%ld,"
                  "\"valueIndex\":%d,"
                  "\"valueStr\":\"%s\""
                  "},",
                  protocolStr, idByte, idLetter, version, speed,
                  voltage, (currentProtocol != PROTOCOL_BSD && startSnap) ? " Start" : "",
                  startSnap ? "true" : "false", sendMode ? "true" : "false", searchMode ? "true" : "false",
                  detectedBaudSnap, valueLINSnap, voltage);

  len += snprintf(json + len, sizeof(json) - len, "\"lin\":{");
  len += snprintf(json + len, sizeof(json) - len, "\"deviceCount\":%d,", deviceCount);
  len += snprintf(json + len, sizeof(json) - len, "\"devices\":[");
  for (int i = 0; i < deviceCount; i++) {
    if (i > 0) len += snprintf(json + len, sizeof(json) - len, ",");
    len += snprintf(json + len, sizeof(json) - len,
                    "{"
                    "\"pid\":%d,"
                    "\"baud\":%ld,"
                    "\"enhanced\":%s,"
                    "\"checksum\":%d,"
                    "\"data\":[",
                    devices[i].pid, devices[i].baudRate, devices[i].enhanced ? "true" : "false", devices[i].checksum);
    for (int j = 0; j < devices[i].dataLength; j++) {
      if (j > 0) len += snprintf(json + len, sizeof(json) - len, ",");
      len += snprintf(json + len, sizeof(json) - len, "%d", devices[i].data[j]);
    }
    len += snprintf(json + len, sizeof(json) - len, "]}");
  }

  len += snprintf(json + len, sizeof(json) - len, "],\"outgoing\":{");

  len += snprintf(json + len, sizeof(json) - len,
                  "\"E9\":{\"mode\":\"%s\",\"checksum\":%d,\"data\":[%d,%d,%d,%d]},",
                  e9EnhancedSnap ? "2.x" : "1.x", checksumE9, e9DataSnap[0], e9DataSnap[1], e9DataSnap[2], e9DataSnap[3]);

  int ecLen = ecEnhancedSnap ? 5 : 4;
  len += snprintf(json + len, sizeof(json) - len,
                  "\"EC\":{\"mode\":\"%s\",\"checksum\":%d,\"data\":[", ecEnhancedSnap ? "2.x" : "1.x", checksumEC);
  for (int i = 0; i < ecLen; i++) {
    if (i > 0) len += snprintf(json + len, sizeof(json) - len, ",");
    len += snprintf(json + len, sizeof(json) - len, "%d", ecDataSnap[i]);
  }
  len += snprintf(json + len, sizeof(json) - len, "]}}");  // end outgoing
  len += snprintf(json + len, sizeof(json) - len, "}");    // end lin

  len += snprintf(json + len, sizeof(json) - len,
                  ",\"bsd\":{"
                  "\"reg0\":%d,"
                  "\"reg2\":%d,"
                  "\"reg6\":%d,"
                  "\"writeData\":%d"
                  "}"
                  "}",
                  bsd_reg0_value, bsd_reg2_value, bsd_reg6_value, bsd_writeData);

  return String(json);
}

// =========================== WiFi/AP CONTROL (CORE0) ===========================
static void wifi_task(void *pv) {
  WifiCmd cmd;
  for (;;) {
    if (xQueueReceive(wifiQ, &cmd, portMAX_DELAY) == pdTRUE) {
      // простая защита от «дёрганья»
      uint32_t now = millis();
      if (now - wifiLastToggleMs < WIFI_TOGGLE_GUARD_MS) continue;
      wifiLastToggleMs = now;

      wifiBusy = true;
      if (cmd == WIFI_CMD_START) {
        if (!wifiActive) {
          Serial.println("🔄 Запуск WiFi AP...");
          // Чистая инициализация
          WiFi.mode(WIFI_OFF);
          delay(80);
          WiFi.mode(WIFI_AP);
          WiFi.setSleep(false);
          esp_wifi_set_ps(WIFI_PS_NONE);

          if (!WiFi.softAP(ssid, password)) {
            Serial.println("❌ softAP failed!");
            WiFi.mode(WIFI_OFF);
          } else {
            delay(80);
            server.begin();
            wifiActive = true;
            forceHeaderReset = true;
            Serial.printf("✅ WiFi AP запущен: %s\n", WiFi.softAPIP().toString().c_str());
          }
        }
      } else if (cmd == WIFI_CMD_STOP) {
        if (wifiActive) {
          Serial.println("🔄 Остановка WiFi AP...");
          server.end();
          delay(60);
          WiFi.softAPdisconnect(true);
          delay(60);
          WiFi.mode(WIFI_OFF);
          wifiActive = false;
          forceHeaderReset = true;
          Serial.println("✅ WiFi AP остановлен");
        }
      }
      wifiBusy = false;
    }
  }
}

void startWeb() {
  WifiCmd c = WIFI_CMD_START;
  if (wifiQ) xQueueSend(wifiQ, &c, 0);
}

void stopWeb() {
  WifiCmd c = WIFI_CMD_STOP;
  if (wifiQ) xQueueSend(wifiQ, &c, 0);
}

// =========================== TASKS ===========================
void taskProtocolSearch(void *pvParameters) {
  for (;;) {
    if (currentProtocol == PROTOCOL_SEARCHING) {
      responseL = responseM = responseH = false;
      if (searchingLIN) {
        deviceCount = 0;

        // ✅ ЗАХВАТИТЬ ЛИНИЮ для всего LIN-сканирования
        line_acquire(LINE_LIN, 9600);

        for (int i = 0; i < 3; i++) {
          for (byte pid : PID_ARRAY) {
            sendLINMessage(pid, BAUD_RATES[i]);
            receiveLINData(BAUD_RATES[i]);
            delayMicroseconds(500);
            taskYIELD();
            if (linLength > 3) {
              if (!deviceExists(linPID) && deviceCount < MAX_DEVICES) {
                devices[deviceCount].pid = linPID;
                devices[deviceCount].baudRate = BAUD_RATES[i];
                devices[deviceCount].enhanced = isEnhancedValid;
                deviceCount++;
              } else {
                for (int j = 0; j < deviceCount; j++) {
                  if (devices[j].pid == linPID) {
                    if (BAUD_RATES[i] > devices[j].baudRate) {
                      devices[j].baudRate = BAUD_RATES[i];
                      devices[j].enhanced = isEnhancedValid;
                    }
                    devices[j].dataLength = linLength - 3;
                    for (int k = 0; k < devices[j].dataLength; k++) devices[j].data[k] = linData[k];
                    devices[j].checksum = linChecksum;
                    break;
                  }
                }
              }
            }
          }
        }

        // ✅ ОСВОБОДИТЬ ЛИНИЮ
        line_release();

        if (deviceCount > 0) {
          if (currentProtocol != PROTOCOL_LIN) {
            if (++protoOkStreak >= PROTO_CONFIDENCE_UP) {
              currentProtocol = PROTOCOL_LIN;
              searchMode = false;
              missCount = 0;
              protoMissStreak = 0;
              protoOkStreak = 0;
              protoModeSinceMs = millis();
              forceHeaderReset = true;
            }
          } else {
            // уже в LIN — просто сбрасываем промахи
            protoMissStreak = 0;
          }
        } else {
          searchingLIN = false;
          if (currentProtocol == PROTOCOL_LIN) {
            if (++protoMissStreak >= PROTO_CONFIDENCE_DOWN && (millis() - protoModeSinceMs) >= PROTO_MIN_HOLD_MS) {
              currentProtocol = PROTOCOL_SEARCHING;
              searchMode = true;
              deviceCount = 0;
              missCount = 0;
              protoOkStreak = 0;
              forceHeaderReset = true;
            }
          } else {
            protoOkStreak = 0;
          }
        }
      } else {
        if (bsd_tryDevice() && bsd_reg6_value > 0) {
          if (currentProtocol != PROTOCOL_BSD) {
            if (++protoOkStreak >= PROTO_CONFIDENCE_UP) {
              currentProtocol = PROTOCOL_BSD;
              bsd_deviceFound = true;
              searchMode = false;
              protoMissStreak = 0;
              protoOkStreak = 0;
              protoModeSinceMs = millis();
              forceHeaderReset = true;
            }
          } else {
            protoMissStreak = 0;
          }
        } else {
          searchingLIN = true;
          if (currentProtocol == PROTOCOL_BSD) {
            if (++protoMissStreak >= PROTO_CONFIDENCE_DOWN && (millis() - protoModeSinceMs) >= PROTO_MIN_HOLD_MS) {
              currentProtocol = PROTOCOL_SEARCHING;
              bsd_deviceFound = false;
              searchMode = true;
              protoOkStreak = 0;
              forceHeaderReset = true;
            }
          } else {
            protoOkStreak = 0;
          }
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskLIN(void *pvParameters) {
  static bool headerShown = false;
  for (;;) {
    if (currentProtocol == PROTOCOL_LIN) {
      bool cycleHadSuccess = false;
      bool validHeaderSeen = false;

      // ✅ ЗАХВАТИТЬ ЛИНИЮ один раз для всего цикла
      line_acquire(LINE_LIN, devices[0].baudRate);

      for (int i = 0; i < deviceCount; i++) {
        sendLINMessage(devices[i].pid, devices[i].baudRate);
        receiveLINData(devices[i].baudRate);
        if (linLength == 0 || (!isClassicValid && !isEnhancedValid)) continue;

        cycleHadSuccess = true;
        devices[i].dataLength = linLength - 3;
        for (int k = 0; k < devices[i].dataLength; k++) devices[i].data[k] = linData[k];
        devices[i].checksum = linChecksum;

        if (devices[i].pid == 0x92) {
          updateMyDataE9();
          sendMessageE9(myDataE9);
          validHeaderSeen = true;
        } else if (devices[i].pid == 0xD6) {
          updateMyDataEC();
          sendMessageEC(myDataEC);
          validHeaderSeen = true;
        } else if (devices[i].pid == TestPID && TestPID != 0x00) {
          if (TestPID_UseE9) {
            updateMyDataE9();
            sendMessageE9(myDataE9);
          } else {
            updateMyDataEC();
            sendMessageEC(myDataEC);
          }
          validHeaderSeen = true;
        }
      }

      line_release();

      bool hasValidDevice = false;
      for (int i = 0; i < deviceCount; i++) {
        if ((devices[i].pid == 0x92 || devices[i].pid == 0xD6 || (devices[i].pid == TestPID && TestPID != 0x00)) && devices[i].dataLength > 0 && (isClassicValid || isEnhancedValid)) {
          hasValidDevice = true;
          break;
        }
      }

      if (!hasValidDevice && cycleHadSuccess) {
        currentProtocol = PROTOCOL_SEARCHING;
        searchMode = true;
        deviceCount = 0;
        missCount = 0;
        headerShown = false;
        searchingLIN = true;
        forceHeaderReset = true;

      } else {
        if (!cycleHadSuccess) missCount++;
        else missCount = 0;
        if (missCount >= MAX_MISSES) {
          currentProtocol = PROTOCOL_SEARCHING;
          searchMode = true;
          deviceCount = 0;
          missCount = 0;
          headerShown = false;
          searchingLIN = true;
          forceHeaderReset = true;
        }
      }

      if (validHeaderSeen && !headerShown) {
        headerShown = true;
      } else if (!validHeaderSeen && headerShown) headerShown = false;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

void taskBSD(void *pvParameters) {
  for (;;) {
    if (currentProtocol == PROTOCOL_BSD) {

      line_acquire(LINE_BSD);  // PATCH

      //bsd_begin_rx();
      bsd_tx_reads_and_write(bsd_writeData, true);
      unsigned long groupStart = millis();
      bool responseReceived = false;
      while (millis() - groupStart < BSD_DELAY_BETWEEN_GROUP_MS) {
        if (bsd_rx_service_once()) responseReceived = true;
        vTaskDelay(1);
      }

      if (responseReceived) {
        bsd_missCount = 0;
        static unsigned long reg6_zero_start = 0;

        if (bsd_reg6_value == 0) {
          if (reg6_zero_start == 0) reg6_zero_start = millis();
          else if (millis() - reg6_zero_start > 3000) {
            currentProtocol = PROTOCOL_SEARCHING;
            bsd_deviceFound = false;
            searchingLIN = true;
            forceHeaderReset = true;
            reg6_zero_start = 0;
          }
        } else {
          reg6_zero_start = 0;
        }
      } else {
        if (++bsd_missCount >= BSD_MAX_MISSES) {
          currentProtocol = PROTOCOL_SEARCHING;
          bsd_deviceFound = false;
          searchingLIN = true;
          forceHeaderReset = true;
          bsd_missCount = 0;
        }
      }

      line_release();  // PATCH
      vTaskDelay(BSD_DELAY_BETWEEN_GROUP_MS / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void taskEncoder(void *pvParameters) {
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);

  for (;;) {
    noInterrupts();
    int16_t raw = encDelta;
    int16_t rem = raw % ENC_STEPS_PER_NOTCH;
    int16_t steps = raw / ENC_STEPS_PER_NOTCH;
    encDelta = rem;
    interrupts();
    while (steps > 0) {
      steps--;
      handleVoltUp();
    }
    while (steps < 0) {
      steps++;
      handleVoltDown();
    }
    static Btn swBtnFsm{ SW };
    pollButton(swBtnFsm);
    buttonHeld = (!digitalRead(SW)) || swBtnFsm.held;

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
// =========================== LOOP & SETUP ===========================
void loop() {
  UiCmd c;
  while (uiQ && xQueueReceive(uiQ, &c, 0) == pdTRUE) {
    switch (c) {
      case UI_VOLT_UP: handleVoltUp(); break;
      case UI_VOLT_DOWN: handleVoltDown(); break;
      case UI_START:
        webStartRequested = true;  // дальше сработает уже существующий код
        break;
    }
  }
#if USE_DISPLAY
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 100) {
    updateHeader();
    updateMessage();
    taskYIELD();
    updateDeviceList();
    taskYIELD();
    updateVoltage();
    taskYIELD();
    lastDisplayUpdate = millis();
  }
#endif
  // «отпускание» Start из веба
  static bool pendingRelease = false;
  static unsigned long releaseAt = 0;
  if (webStartRequested) {
    buttonHeld = true;
    vTaskDelay(1);
    if (currentProtocol == PROTOCOL_LIN) {
      line_acquire(LINE_LIN, baudRateEC);  // ✅ ДОБАВИТЬ
      updateMyDataEC();
      sendMessageEC(myDataEC);
      line_release();
    }
    releaseAt = millis() + 200;
    pendingRelease = true;
    webStartRequested = false;
  }
  if (pendingRelease && (int32_t)(millis() - releaseAt) >= 0) {
    buttonHeld = false;
    pendingRelease = false;
  }
  // Кнопка Wi-Fi: только заявки
  {
    static uint32_t ts = 0;
    static bool stable = HIGH;
    bool raw = digitalRead(WIFI_BTN_PIN);
    if (raw != stable) {
      if (ts == 0) ts = millis();
      if (millis() - ts >= 25) {  // debounce
        bool prev = stable;
        stable = raw;
        ts = 0;
        bool wasPressed = (prev != wifiBtnIdleLevel);
        bool nowPressed = (stable != wifiBtnIdleLevel);
        if (!wasPressed && nowPressed && !wifiBusy) {
          if (wifiActive) stopWeb();
          else startWeb();
        }
      }
    } else {
      ts = 0;
    }
  }
  vTaskDelay(20 / portTICK_PERIOD_MS);
}

// =========================== HTML (как было) ===========================
const char MAIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<title>LIN/BSS Alternator Test</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
/* ... (тот же CSS, что и в предыдущей версии — без изменений) ... */
body { background:#18181B;color:#ECECEC;font-family:'Segoe UI',Arial,sans-serif;margin:0;padding:0;}
.header-main { background:#232329;padding:16px 20px 10px 20px;font-size:2rem;font-weight:bold;letter-spacing:2px;display:flex;align-items:center;justify-content:space-between;}
.refresh-btn { background:#232329;border:none;border-radius:50%;width:36px;height:36px;cursor:pointer;display:flex;align-items:center;justify-content:center;transition:background .2s;}
.refresh-btn:hover { background:#32323a;}
.refresh-btn svg { width:22px;height:22px;fill:#FFD600;}
.info-panel { background:#222225;color:#ECECEC;display:flex;gap:28px;align-items:center;font-size:1.15rem;padding:12px 20px;}
.info-panel .logo { font-size:2.2rem;font-weight:bold;color:#FFD600;}
.info-panel .logo.searching { color:#FF6B6B;}
.info-panel .logo.lin { color:#10B981;}
.info-panel .logo.bsd { color:#3B82F6;}
.info-panel .pid { color:#FF9800;font-weight:bold;}
.info-panel .speed { color:#2196F3;font-weight:bold;}
.info-panel .protocol { color:#9B51E0;font-weight:bold;}
.voltage-panel { background:#2D2D32;padding:16px 20px 14px 20px;display:flex;gap:16px;align-items:center;}
.volt-btn { background:#FFD600;border:none;color:#18181B;font-size:1.35rem;font-weight:bold;border-radius:7px;padding:7px 20px;margin:0 6px;cursor:pointer;box-shadow:0 2px 8px #0002;transition:background .2s;}
.volt-btn:hover { background:#FFF066;}
.volt-value { font-size:2.5rem;font-weight:bold;color:#FF3333;margin:0 18px;letter-spacing:2px;min-width:90px;text-align:center;}
.start-btn { background:#10b981;color:#fff;border:none;border-radius:7px;font-size:1.15rem;font-weight:bold;padding:8px 32px;margin-left:auto;cursor:pointer;box-shadow:0 2px 8px #0002;transition:background .2s;}
.start-btn:hover { background:#14d8a6;}
.protocol-status { background:#1F2937;padding:10px 20px;color:#FFD600;font-weight:bold;text-align:center;border-bottom:2px solid #374151;}
.protocol-status.searching { background:#7F1D1D;color:#FEE2E2;}
.protocol-status.lin { background:#064E3B;color:#A7F3D0;}
.protocol-status.bsd { background:#1E3A8A;color:#BFDBFE;}
.accordion-section { margin:18px 0 0 0;border-radius:13px;overflow:hidden;box-shadow:0 2px 16px #0003;background:#232329;}
.accordion-header { background:#292933;color:#FFD600;padding:13px 20px;font-size:1.08rem;font-weight:bold;letter-spacing:1px;cursor:pointer;display:flex;align-items:center;border:none;outline:none;transition:background .2s;}
.accordion-header .arrow { margin-right:10px;font-size:1.4em;color:#FFD600;}
.accordion-content { display:none;background:#202025;padding:10px 20px 16px 20px;color:#FFD600;font-family:'Fira Mono',monospace;font-size:1.1rem;overflow-x:auto;}
.accordion-content.active { display:block;}
.table-block { width:100%;overflow-x:auto;}
.data-table { width:100%;border-collapse:collapse;margin:0 0 10px 0;background:#292933;color:#ECECEC;font-size:1.04rem;}
.data-table th,.data-table td { padding:7px 12px;border-bottom:1px solid #31313A;text-align:center;}
.data-table th { background:#2d2d32;color:#FFD600;font-size:1.04em;}
.data-table .pid { color:#FF9800;font-weight:bold;}
.data-table .data { color:#31EA8C;}
.data-table .crc { color:#FFD600;}
.data-table .speed { color:#2196F3;}
.data-table .protocol { color:#9B51E0;}
.data-table .register { color:#F59E0B;font-weight:bold;}
.data-table .description { color:#A78BFA;}
@media (max-width:650px){
  .header-main,.info-panel,.voltage-panel{flex-direction:column;align-items:flex-start;}
  .info-panel,.voltage-panel{gap:7px;font-size:1.02rem;}
  .volt-value{font-size:1.7rem;min-width:65px;}
  .start-btn{margin-left:0;width:100%;}
  .accordion-header{font-size:1.02rem;}
}
</style>
</head>
<body>
<div class='header-main'>
  LIN/BSS Alternator Test
  <button class='refresh-btn' onclick='location.reload();'>
    <svg viewBox='0 0 24 24'><path d='M12 6V3L8 7l4 4V8c3.3 0 6 2.7 6 6s-2.7 6-6 6-6-2.7-6-6h-2c0 4.4 3.6 8 8 8s8-3.6 8-8-3.6-8-8-8z'/></svg>
  </button>
</div>

<div class='protocol-status' id='protocol-status'>Searching for devices...</div>

<div class='info-panel' id='info-panel'>
  <span class='logo searching'>COM</span>
  <span>ID: <span class='pid'>N/A</span></span>
  <span>Speed: <span class='speed'>N/A</span></span>
  <span>Protocol: <span class='protocol'>N/A</span></span>
</div>

<div class='voltage-panel'>
  <button class='volt-btn' onclick="sendVoltCmd('down')">Volt-</button>
  <span class='volt-value' id='volt-value'>13.0V</span>
  <button class='volt-btn' onclick="sendVoltCmd('up')">Volt+</button>
  <button class='start-btn' onclick="sendStartCmd()">Start</button>
</div>

<div class='accordion-section'>
  <button class='accordion-header' onclick="toggleAccordion('incoming')">
    <span class='arrow'>&#9658;</span>Incoming Data (LIN Responses / BSS Registers)
  </button>
  <div class='accordion-content' id='incoming'>
    <div id='lin-incoming' style='display:none;'>
      <table class='data-table'><tr><th>#</th><th>PID</th><th>Data</th><th>CRC</th><th>Protocol</th><th>Speed</th></tr></table>
    </div>
    <div id='bsd-incoming' style='display:none;'>
      <table class='data-table'>
        <tr><th>Register</th><th>Value (DEC)</th><th>Value (HEX)</th><th>Description</th></tr>
      </table>
    </div>
  </div>
</div>

<div class='accordion-section'>
  <button class='accordion-header' onclick="toggleAccordion('outgoing')">
    <span class='arrow'>&#9658;</span>Outgoing Data (LIN Messages / BSS Commands)
  </button>
  <div class='accordion-content' id='outgoing'>
    <div id='lin-outgoing' style='display:none;'>
      <table class='data-table'><tr><th>PID</th><th>Data</th><th>CRC</th><th>Mode</th><th>Voltage</th></tr></table>
    </div>
    <div id='bsd-outgoing' style='display:none;'>
      <table class='data-table'><tr><th>Frame</th><th>Data</th><th>Description</th></tr></table>
    </div>
  </div>
</div>

<div class='accordion-section'>
  <button class='accordion-header' onclick="toggleAccordion('system')">
    <span class='arrow'>&#9658;</span>System Info & Debug
  </button>
  <div class='accordion-content active' id='system'>
    Protocol Mode: <span class='protocol'>SEARCHING</span> |
    Device Count: <span class='speed'>0</span> |
    Voltage Index: <span class='crc'>9 (13.0V)</span> |
    Search Mode: <span class='data'>ON</span>
  </div>
</div>

<script>
let currentProtocol = 'SEARCHING';
function toggleAccordion(id){
  const block = document.getElementById(id);
  const btn = block.previousElementSibling.querySelector('.arrow');
  const open = block.classList.toggle('active');
  btn.innerHTML = open ? '&#9660;' : '&#9658;';
}
function fmtHex(n){ if(n===null||n===undefined||isNaN(n))return'0x00'; let v=Number(n)&0xFF; return '0x'+v.toString(16).toUpperCase().padStart(2,'0'); }
function updateProtocolDisplay(protocol){
  const statusDiv=document.getElementById('protocol-status');
  const logoSpan=document.querySelector('.info-panel .logo');
  statusDiv.className='protocol-status'; logoSpan.className='logo';
  if(protocol==='LIN'){ statusDiv.classList.add('lin'); logoSpan.classList.add('lin');
    statusDiv.textContent='LIN Protocol Active'; logoSpan.textContent='LIN';
    document.getElementById('lin-incoming').style.display='block';
    document.getElementById('lin-outgoing').style.display='block';
    document.getElementById('bsd-incoming').style.display='none';
    document.getElementById('bsd-outgoing').style.display='none';
  } else if(protocol==='BSD'){ statusDiv.classList.add('bsd'); logoSpan.classList.add('bsd');
    statusDiv.textContent='BSD Protocol Active'; logoSpan.textContent='BSD';
    document.getElementById('lin-incoming').style.display='none';
    document.getElementById('lin-outgoing').style.display='none';
    document.getElementById('bsd-incoming').style.display='block';
    document.getElementById('bsd-outgoing').style.display='block';
  } else { statusDiv.classList.add('searching'); logoSpan.classList.add('searching');
    statusDiv.textContent='Searching for LIN/BSD devices...'; logoSpan.textContent='COM';
    document.getElementById('lin-incoming').style.display='none';
    document.getElementById('lin-outgoing').style.display='none';
    document.getElementById('bsd-incoming').style.display='none';
    document.getElementById('bsd-outgoing').style.display='none';
  }
  currentProtocol=protocol;
}
function renderInfoPanel(data){
  const logoClass = data.protocol==='LIN'?'lin':data.protocol==='BSD'?'bsd':'searching';
  const idDisp = (data.id>=0)?(String(data.id)+(data.idLetter?` (${data.idLetter})`:'')):'N/A';
  const html = `
    <span class="logo ${logoClass}">${data.protocol==='LIN'?'LIN':data.protocol==='BSD'?'BSD':'COM'}</span>
    <span>ID: <span class="pid">${idDisp}</span></span>
    <span>Speed: <span class="speed">${data.speed||'N/A'}</span></span>
    <span>Protocol: <span class="protocol">${data.version}</span></span>
  `;
  document.getElementById('info-panel').innerHTML = html;
}
function renderVoltage(data){ document.getElementById('volt-value').textContent=data.voltage; }
function renderIncomingLIN(data){
  const devs=data.lin?.devices||[];
  let rows=`<tr><th>#</th><th>PID</th><th>Data</th><th>CRC</th><th>Protocol</th><th>Speed</th></tr>`;
  for(let i=0;i<devs.length;i++){
    const d=devs[i];
    const dataCells=(d.data||[]).map(v=>`<span class="data">${fmtHex(v)}</span>`).join(' ');
    rows+=`<tr><td>${i+1}</td><td class='pid'>${fmtHex(d.pid)}</td><td>${dataCells}</td><td class='crc'>${fmtHex(d.checksum)}</td><td class='protocol'>${d.enhanced?'2.x':'1.x'}</td><td class='speed'>${d.baud||''}</td></tr>`;
  }
  document.getElementById('lin-incoming').innerHTML=`<table class="data-table">${rows}</table>`;
}
function renderOutgoingLIN(data){
  const e9=data.lin?.outgoing?.E9; const ec=data.lin?.outgoing?.EC;
  let rows=`<tr><th>PID</th><th>Data</th><th>CRC</th><th>Mode</th><th>Voltage</th></tr>`;
  if(e9){ const d=(e9.data||[]).map(fmtHex).join(' ');
    rows+=`<tr><td class='pid'>0xE9</td><td class='data'>${d}</td><td class='crc'>${fmtHex(e9.checksum)}</td><td class='protocol'>${e9.mode}</td><td class='speed'>${data.voltage}</td></tr>`;
  }
  if(ec){ const d=(ec.data||[]).map(fmtHex).join(' ');
    rows+=`<tr><td class='pid'>0xEC</td><td class='data'>${d}</td><td class='crc'>${fmtHex(ec.checksum)}</td><td class='protocol'>${ec.mode}</td><td class='speed'>${data.voltage}</td></tr>`;
  }
  document.getElementById('lin-outgoing').innerHTML=`<table class="data-table">${rows}</table>`;
}
function renderIncomingBSD(data){
  const b=data.bsd||{};
  const rows=`
  <tr><th>Register</th><th>Value (DEC)</th><th>Value (HEX)</th><th>Description</th></tr>
  <tr><td class="register">REG0</td><td class="data">${b.writeData??''} <br><small style="color:#888;">(Received: ${b.reg0??''})</small></td><td class="crc">${fmtHex(b.writeData)} <br><small style="color:#888;">(${fmtHex(b.reg0)})</small></td><td class="description">Voltage Control</td></tr>
  <tr><td class="register">REG2</td><td class="data">${b.reg2??''}</td><td class="crc">${fmtHex(b.reg2)}</td><td class="description">Status</td></tr>
  <tr><td class="register">REG6</td><td class="data">${b.reg6??''}</td><td class="crc">${fmtHex(b.reg6)}</td><td class="description">Device ID</td></tr>`;
  document.getElementById('bsd-incoming').innerHTML=`<table class="data-table">${rows}</table>`;
}
function renderOutgoingBSD(data){
  const b=data.bsd||{};
  const rows=`
   <tr><th>Frame</th><th>Data</th><th>Description</th></tr>
   <tr><td class="register">REG6</td><td class="data">Read Request</td><td class="description">Device ID Request</td></tr>
   <tr><td class="register">REG2</td><td class="data">Read Request</td><td class="description">Status Request</td></tr>
   <tr><td class="register">REG0</td><td class="crc">${fmtHex(b.writeData)}</td><td class="description">Write Voltage (${document.getElementById('volt-value').textContent})</td></tr>`;
  document.getElementById('bsd-outgoing').innerHTML=`<table class="data-table">${rows}</table>`;
}
function renderFooter(data){
  let line=`Protocol Mode: <span class='protocol'>${data.protocol}</span>`;
  if(data.protocol==='LIN'){ line+=` | Device Count: <span class='speed'>${data.lin?.deviceCount??0}</span>`; }
  else if(data.protocol==='BSD'){ line+=` | BSD WriteData: <span class='crc'>${fmtHex(data.bsd?.writeData)}</span>`; }
  line+=` | Voltage Index: <span class='crc'>${data.flags?.valueIndex} (${data.flags?.valueStr}V)</span>`;
  line+=` | Search Mode: <span class='data'>${data.flags?.searchMode?'ON':'OFF'}</span>`;
  if(data.protocol==='LIN'){
    line+=` | Send Mode: <span class='data'>${data.flags?.sendMode?'Send+Read':'Read Only'}</span>`;
    line+=` | Detected Baud: <span class='speed'>${data.flags?.detectedBaud??''}</span>`;
    line+=` | Button Held: <span class='data'>${data.flags?.start?'Yes':'No'}</span>`;
  }
  document.getElementById('system').innerHTML=line;
}
function updateUI(data){
  updateProtocolDisplay(data.protocol);
  renderInfoPanel(data);
  renderVoltage(data);
  if(data.protocol==='LIN'){ renderIncomingLIN(data); renderOutgoingLIN(data); }
  else if(data.protocol==='BSD'){ renderIncomingBSD(data); renderOutgoingBSD(data); }
  renderFooter(data);
}
async function fetchData(){
  try{ const r=await fetch('/data'); const data=await r.json(); updateUI(data); }
  catch(e){ console.error('Fetch error:',e); }
}
function sendVoltCmd(dir){ fetch(dir==='up'?'/volt_up':'/volt_down').then(()=>setTimeout(fetchData,200)); }
function sendStartCmd(){ fetch('/toggle_encoder').then(()=>setTimeout(fetchData,200)); }
setInterval(fetchData,1500); fetchData();
</script>
</body>
</html>
)rawliteral";

// =========================== SETUP ===========================
void setup() {
  // Стабильный старт Wi-Fi OFF (не включаем здесь AP)
  WiFi.mode(WIFI_OFF);
  delay(100);
  esp_wifi_stop();
  esp_wifi_deinit();
  delay(100);

  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  pinMode(WIFI_BTN_PIN, INPUT_PULLUP);
  wifiBtnIdleLevel = digitalRead(WIFI_BTN_PIN);

  // PATCH BEGIN: init line arbiter
  line_init();
  // PATCH END

  // Энкодер
  encState = (digitalRead(CLK) << 1) | digitalRead(DT);
  attachInterrupt(digitalPinToInterrupt(CLK), enc_isr_any, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT), enc_isr_any, CHANGE);

#if USE_DISPLAY
  tft.init();
  tft.invertDisplay(false);
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  devListSprite.setColorDepth(8);
  devListSprite.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT - listYStart);
  devListSprite.setFreeFont(FM9);
  devListSprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  devListSprite.setTextDatum(TL_DATUM);

  messageSprite.setColorDepth(8);
  messageSprite.createSprite(SCREEN_WIDTH, msgHeight);
  messageSprite.setFreeFont(FSB9);
  messageSprite.setTextColor(TFT_WHITE, TFT_BLACK);
  messageSprite.setTextDatum(TL_DATUM);

  tft.setFreeFont(FSB24);
  voltX = tft.textWidth("Volt:") + 6;

  voltageSprite.setColorDepth(8);
  voltageSprite.createSprite(SCREEN_WIDTH - voltX, voltHeight);
  voltageSprite.setFreeFont(FSB24);
  voltageSprite.setTextColor(TFT_RED, TFT_BLACK);
  voltageSprite.setTextDatum(TL_DATUM);
#endif

  Serial.begin(115200);
  HARDWARE_SERIAL.begin(9600);
  detectLINBaudRate();
  wifiQ = xQueueCreate(4, sizeof(WifiCmd));
  uiQ = xQueueCreate(16, sizeof(UiCmd));
  // --- Web routes (регистрируем один раз) ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", MAIN_HTML);
  });
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String js = createJson();  // без jsonMux!
    size_t n = std::min<size_t>(sizeof(g_jsonBuf) - 1, (size_t)js.length());
    portENTER_CRITICAL(&jsonMux);
    memcpy(g_jsonBuf, js.c_str(), n);
    g_jsonBuf[n] = 0;
    portEXIT_CRITICAL(&jsonMux);
    request->send(200, "application/json", g_jsonBuf);
  });

  server.on("/volt_up", HTTP_GET, [](AsyncWebServerRequest *request) {
    UiCmd c = UI_VOLT_UP;
    xQueueSend(uiQ, &c, 0);
    request->send(200, "text/plain", "OK");
  });
  server.on("/volt_down", HTTP_GET, [](AsyncWebServerRequest *request) {
    UiCmd c = UI_VOLT_DOWN;
    xQueueSend(uiQ, &c, 0);
    request->send(200, "text/plain", "OK");
  });
  server.on("/toggle_encoder", HTTP_GET, [](AsyncWebServerRequest *request) {
    static uint32_t lastTick = 0;
    uint32_t now = xTaskGetTickCount();
    if (now - lastTick > pdMS_TO_TICKS(300)) {
      lastTick = now;
      UiCmd c = UI_START;
      xQueueSend(uiQ, &c, 0);
      request->send(200, "text/plain", "Start OK");
    } else {
      request->send(429, "text/plain", "Too Fast");
    }
  });
  // === ЗАДАЧИ ===
  // Wi-Fi стек и веб — Core0
  // Протоколы — Core1, одинаковые небольшие приоритеты
  xTaskCreatePinnedToCore(taskProtocolSearch, "ProtocolSearch", 8192, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(taskLIN, "taskLIN", 12288, NULL, 7, NULL, 1);
  xTaskCreatePinnedToCore(taskBSD, "taskBSD", 12288, NULL, 7, NULL, 1);  //(taskBSD, "taskBSD", 12288, NULL, 7, NULL, 1);
  // Энкодер — Core0
  xTaskCreatePinnedToCore(taskEncoder, "taskEncoder", 2048, NULL, 9, NULL, 0);

  xTaskCreatePinnedToCore(wifi_task, "wifi_task", 4096, nullptr, 8, nullptr, 0);  // Core0

  // BSD init
  bsd_writeData = 0x0F + valueLIN;   // всегда в ногу с valueLIN
  //bsd_writeData = 0x18;
  bsd_reg0_value = 0;
  bsd_reg6_value = 0;
  bsd_deviceFound = false;
}
