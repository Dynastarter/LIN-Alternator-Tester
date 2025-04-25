#include <Wire.h>
#include <SPI.h>  // Для работы с аппаратным SPI
//#include <Adafruit_GFX.h>     // Базовая графическая библиотека Adafruit
#include <TFT_eSPI.h>  // Драйвер для ST7789

#include <FS.h>
using fs::FS;

#include <WiFi.h>       // Подключение библиотеки для работы с WiFi, позволяющей подключить ESP32 к беспроводной сети
#include <WebServer.h>  // Подключение библиотеки для создания веб-сервера, через который можно управлять устройством
// ------------------------
//  WiFi и веб-сервер
// ------------------------
const char* ssid = "LIN";           // SSID сети WiFi, к которой будет подключаться устройство
const char* password = "12345678";  // Пароль для подключения к WiFi
WebServer server(80);               // Создаем объект веб-сервера, который слушает порт 80



// Инициализация дисплея ST7789 через аппаратный SPI
TFT_eSPI tft = TFT_eSPI();  // настройки пинов и параметров дисплея задаются в файле конфигурации TFT_eSPI (User_Setup.h)

String prevHeader = "";
String prevVoltage = "";
String prevMessage = "";

// Глобальные переменные для настройки цветов символов
uint16_t colorL = TFT_RED;    // Цвет для символа "L"
uint16_t colorM = TFT_GREEN;  // Цвет для символа "M"
uint16_t colorH = TFT_BLUE;   // Цвет для символа "H"


#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240  // Задайте реальное значение высоты вашего дисплея

//const int SCREEN_WIDTH = 320;  // Ширина дисплея
const int headerHeight = 27;  // Высота области заголовка (строка 0)
const int voltHeight = 27;    // Высота области для напряжения (строка 1)
const int msgHeight = 27;     // Высота области для сообщения (строка 2)
const int listYStart = 83;    // Начало области списка устройств (строка 3)
int yPos = 0;


// ------------------------------------------------------------------------
// Подключаем библиотеку для работы с энкодером
#include "GyverEncoder.h"  // Библиотека для чтения энкодера + кнопки

// =========================================================
// ====== Энкодер (Encoder) ======
// =========================================================

#define CLK 14
#define DT 27
#define SW 25

Encoder enc(CLK, DT, SW, TYPE2);

bool buttonPressed = false;  //0
bool buttonHeld = false;     //0

// =========================================================
// ====== Настройка LIN-порта ======

// Выбор используемого порта (по умолчанию - USE_SERIAL1)
//#define USE_SERIAL1
#define USE_SERIAL2
// #define USE_SERIAL3
// #define USE_SOFTWARE_SERIAL

#ifdef USE_SERIAL1
#define HARDWARE_SERIAL Serial1  // Используем Serial1
#define TX_PIN 18                // Пин TX для LIN
#define RX_PIN 19                // Пин RX для LIN
#elif defined(USE_SERIAL2)
#define HARDWARE_SERIAL Serial2
#define TX_PIN 17  // для wemos17 для меги16
#define RX_PIN 16  //  для wemos16 для меги17
#elif defined(USE_SERIAL3)
#define HARDWARE_SERIAL Serial3
#define TX_PIN 14
#define RX_PIN 15
#elif defined(USE_SOFTWARE_SERIAL)
#define HARDWARE_SERIAL linSerial
#define TX_PIN 10
#define RX_PIN 11
#include <SoftwareSerial.h>
SoftwareSerial linSerial(RX_PIN, TX_PIN);
#else
#error "Выберите один из доступных портов!"
#endif

// =========================================================
// ====== Массивы "виртуальных" значений напряжений ======
int voltLIN1[] = {
  0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
  0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
  0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26,
  0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C
};

int voltLIN2[] = {
  0x3C, 0x40, 0x44, 0x48, 0x4C, 0x50, 0x54, 0x58,
  0x5C, 0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 0x78,
  0x7C, 0x80, 0x84, 0x88, 0x8C, 0x90, 0x94, 0x98,
  0x9C, 0xA0, 0xA4, 0xA8, 0xAC, 0xB0
};

const char* setVOLT[] = {
  "12.1", "12.2", "12.3", "12.4", "12.5", "12.6", "12.7", "12.8", "12.9", "13.0",
  "13.1", "13.2", "13.3", "13.4", "13.5", "13.6", "13.7", "13.8", "13.9", "14.0",
  "14.1", "14.2", "14.3", "14.4", "14.5", "14.6", "14.7", "14.8", "14.9", "15.0"
};

int valueLIN = 9;

// =========================================================
// ====== Глобальные массивы для отправки по LIN ======
byte myDataE9[4] = { 0x00, 0x00, 0x00, 0x00 };
byte myDataEC[5] = { 0x00, 0xC7, 0xA8, 0x8C, 0x13 };

// =========================================================
// ====== Структура Device и список найденных устройств ======
struct Device {
  byte pid;
  long baudRate;
  bool enhanced;
  int dataLength;  // Количество полученных байт данных (linLength - 3)
  byte data[8];    // Полученные данные (до 8 байт)
  byte checksum;   // Последняя контрольная сумма

  uint8_t lastData[8];      // Массив для хранения последних полученных данных устройства
  int lastDataLength;       // Длина последних полученных данных
  uint8_t lastChecksum;     // Последняя полученная контрольная сумма
  unsigned long Timestamp;  // Время последнего получения данных (в миллисекундах)
                            //bool responseL, responseM, responseH;       // Флаги, обозначающие наличие отклика на разных скоростях (низкая, средняя, высокая скорость)
  bool responseL = false;   // 2800
  bool responseM = false;   // 9600
  bool responseH = false;   // 19200
};

#define MAX_DEVICES 10
Device devices[MAX_DEVICES];
int deviceCount = 0;

bool deviceExists(byte pid) {
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].pid == pid) return true;
  }
  return false;
}

bool searchMode = true;

// =========================================================
// ====== Массив PID для поиска и скорости ======
/*
const byte PID_ARRAY[] = {
  0x11, 0x14, 0x55, 0x61, 0x73, 0x80,
  0x92, 0x97, 0xA8, 0xD3, 0xD6, 0xD8,
  0xE7, 0xE9, 0xEC, 0xF0
};
*/
const byte PID_ARRAY[] = {  0x06, 0x08, 0x0D, 0x11, 0x14, 0x1A, 0x1F, 0x20, 0x25, 0x2B,
                            0x2E, 0x32, 0x37, 0x39, 0x42, 0x47, 0x49, 0x4C, 0x50, 0x55,
                            0x5B, 0x5E, 0x61, 0x64, 0x6A, 0x6F, 0x73, 0x76, 0x78, 0x80, 
                            0x85, 0x8B, 0x8E, 0x92, 0x97, 0x99, 0x9C, 0xA3, 0xA6, 0xA8, 
                            0xAD, 0xB1, 0xB4, 0xBA, 0xC1, 0xC4, 0xCA, 0xCF, 0xD3, 0xD6, 
                            0xD8, 0xDD, 0xE2, 0xE7, 0xE9, 0xEC, 0xF0, 0xF5, 0xFB };                                                 // Массив PID, которые будут отправляться


const long BAUD_RATES[] = { 2800, 9600, 19200 };

long detectedBaudRate = 9600;

byte linPID = 0;
byte linData[8];
byte linChecksum = 0;

bool isClassicValid = false;
bool isEnhancedValid = false;

bool isClassicMode = false;
bool isEnhancedMode = false;

bool responseL = false;  // 2800
bool responseM = false;  // 9600
bool responseH = false;  // 19200

byte linBuffer[11];
int linLength = 0;

long baudRateE9 = 9600;
bool enhancedChecksumE9 = false;
long baudRateEC = 9600;
bool enhancedChecksumEC = false;

bool sendMode = true;  // Режим работы для веб управления: true = send+read, false = только read

// Буфер для дополнительного сообщения, выводимого на дисплее
char messageLine3[64] = "";

// =========================================================
// Функция calculateChecksum
byte calculateChecksum(byte data[], int length, bool isEnhanced) {
  uint16_t sum = 0;
  int startIndex = isEnhanced ? 1 : 2;
  for (int i = startIndex; i < length - 1; i++) {
    sum += data[i];
    if (sum > 0xFF) {
      sum = (sum & 0xFF) + 1;
    }
  }
  return (byte)(~sum);
}


//  Функция генерации break-сигнала
// ------------------------
void sendBreak(long baudRate) {
  HARDWARE_SERIAL.flush();    // Очищаем буфер передачи HARDWARE_SERIAL
  HARDWARE_SERIAL.end();      // Завершаем работу последовательного порта
  pinMode(TX_PIN, OUTPUT);    // Настраиваем TX_PIN как выходной
  digitalWrite(TX_PIN, LOW);  // Принудительно устанавливаем низкий уровень на TX_PIN для формирования break-сигнала

  unsigned int bitTimes;  // Переменная для хранения количества битовых интервалов для break-сигнала
  if (baudRate <= 9600)
    bitTimes = 15;  // Если скорость меньше или равна 9600, используем 15 битовых интервалов
  else if (baudRate <= 19200)
    bitTimes = 13;  // Если скорость меньше или равна 19200, используем 13 битовых интервалов
  else
    bitTimes = 11;  // Иначе используем 11 битовых интервалов

  unsigned long breakDuration = (1000000UL * bitTimes) / baudRate;  // Рассчитываем длительность break-сигнала в микросекундах
  delayMicroseconds(breakDuration);                                 // Ждем нужное время для формирования break-сигнала

  digitalWrite(TX_PIN, HIGH);                                   // Устанавливаем высокий уровень на TX_PIN после break-сигнала
  unsigned long highDuration = 1000000UL / baudRate;            // Рассчитываем длительность одного битового интервала в микросекундах
  delayMicroseconds(highDuration);                              // Ждем один битовый интервал
  HARDWARE_SERIAL.begin(baudRate, SERIAL_8N1, RX_PIN, TX_PIN);  // Перезапускаем последовательный порт с заданной скоростью и настройками
  delayMicroseconds(highDuration);                              // Ждем еще один битовый интервал
  HARDWARE_SERIAL.write(0x55);                                  // Отправляем стандартный байт 0x55, который используется для синхронизации
  HARDWARE_SERIAL.flush();                                      // Очищаем выходной буфер последовательного порта
}


// =========================================================
// Функция sendLINMessage
void sendLINMessage(byte pid, long baudRate) {
  sendBreak(baudRate);
  HARDWARE_SERIAL.write(pid);
  HARDWARE_SERIAL.flush();
}

// =========================================================
// Функция receiveLINData
void receiveLINData(long baudRate) {
  unsigned long timeout = millis() + 50;                       // Устанавливаем таймаут 50 мс для приёма данных
  linLength = 0;                                               // Обнуляем счётчик принятых байт
  while (millis() < timeout) {                                 // Пока не истёк таймаут
    if (HARDWARE_SERIAL.available()) {                         // Если появились байты во входном буфере
      while (HARDWARE_SERIAL.available() && linLength < 11) {  // Читаем, пока байты доступны и не превышен размер буфера
        linBuffer[linLength] = HARDWARE_SERIAL.read();         // Считываем один байт в linBuffer
        linLength++;                                           // Увеличиваем количество считанных байт
      }
    }
  }
  if (linLength == 0) {             // Если ничего не получено
    Serial.println("No response");  // Выводим сообщение в Serial
    return;                         // Выходим из функции
  }
  linPID = linBuffer[1];
  linChecksum = linBuffer[linLength - 1];
  for (int i = 2; i < linLength - 1; i++) {
    linData[i - 2] = linBuffer[i];
  }
  byte expectedClassic = calculateChecksum(linBuffer, linLength, false);
  byte expectedEnhanced = calculateChecksum(linBuffer, linLength, true);
  isClassicValid = (linChecksum == expectedClassic);
  isEnhancedValid = (linChecksum == expectedEnhanced);
  if (isEnhancedValid) {
    isEnhancedMode = true;
    isClassicMode = false;
  } else if (isClassicValid) {
    isClassicMode = true;
    isEnhancedMode = false;
  } else {
    isClassicMode = false;
    isEnhancedMode = false;
  }
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
    Serial.print("Ответ получен на скорости: ");
    if (baudRate == 19200) Serial.println("H");
    else if (baudRate == 9600) Serial.println("M");
    else if (baudRate == 2800) Serial.println("L");
  }
  printLINData();
  // Вместо отдельного LCD-вывода вызываем обновление TFT-дисплея
  // (функция updateDisplay_ST() вызывается в основном loop)
}

// =========================================================
// Функция printLINData (вывод в Serial)
void printLINData() {
  Serial.print("PID: 0x");
  Serial.print(linPID, HEX);
  Serial.print(" | Data: ");
  for (int i = 0; i < linLength - 3; i++) {
    Serial.print("0x");
    Serial.print(linData[i], HEX);
    Serial.print(" ");
  }
  Serial.print("| Checksum: 0x");
  Serial.print(linChecksum, HEX);
  Serial.print(" | Protocol: ");
  if (isClassicValid && isEnhancedValid) {
    Serial.println("LIN 1.x & 2.x");
  } else if (isClassicValid) {
    Serial.println("LIN 1.x");
  } else if (isEnhancedValid) {
    Serial.println("LIN 2.x");
  } else {
    Serial.println("ERROR");
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateHeader() {
  // Очищаем область заголовка
  // tft.fillRect(0, 0, SCREEN_WIDTH, headerHeight, TFT_BLACK);
  tft.setTextSize(3);

  int x = 0;    // Начинаем с левого края
  String part;  // Для формирования отдельных частей текста

  // 1. Выводим статичный текст "LIN:" в зелёном цвете
  part = "LIN:";
  tft.setCursor(x, 0);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print(part);
  x += tft.textWidth(part);  // x = ширина текста "LIN:"

  // 2. Если требуется вывести значение linData[0]
  if (linPID == 0x92 || linPID == 0xD6) {
    part = String(linData[0]);
    tft.setCursor(x, 0);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(part);
    x += tft.textWidth(part);
  }

  // 3. Если активен флаг responseL — выводим символ " L" (пробел для отделения)
  if (responseL) {
    part = " L";  // начиная со знака пробела для отделения
    tft.setCursor(x, 0);
    tft.setTextColor(colorL, TFT_BLACK);  // Используем заранее определённую переменную цвета colorL
    tft.print(part);
    x += tft.textWidth(part);
  }

  // 4. Если активен флаг responseM — выводим символ "M"
  if (responseM) {
    part = "M";
    tft.setCursor(x, 0);
    tft.setTextColor(colorM, TFT_BLACK);
    tft.print(part);
    x += tft.textWidth(part);
  }

  // 5. Если активен флаг responseH — выводим символ "H"
  if (responseH) {
    part = "H";
    tft.setCursor(x, 0);
    tft.setTextColor(colorH, TFT_BLACK);
    tft.print(part);
    x += tft.textWidth(part);
  }

  // 6. Вывод информации о версии протокола
  if (isClassicValid)
    part = " V.1.x";
  else
    part = " V.2.x";

  tft.setCursor(x, 0);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print(part);
}


// Функция обновления области напряжения
void updateVoltage() {
  int yVoltage = headerHeight;  // Расположение области по оси Y (под заголовком)

  // Вывод статической метки "Volt:" (выводим один раз)
  static bool labelPrinted = false;
  if (!labelPrinted) {
    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Статичный текст можно выводить белым
    tft.setCursor(0, yVoltage);
    tft.print("Volt:");
    labelPrinted = true;
  }

  // Динамически изменяемое значение напряжения отображается справа от метки
  String currentVoltage = String(setVOLT[valueLIN]);
  int xVoltageValue = 110;  // Начало динамической части (подберите значение в зависимости от ширины "Volt:")

  // Если напряжение изменилось — обновляем только динамическую область
  if (currentVoltage != prevVoltage) {
    // Очистка области динамического значения
    //tft.fillRect(xVoltageValue, yVoltage, SCREEN_WIDTH - xVoltageValue, voltHeight, TFT_BLACK);
    tft.setTextSize(3);
    tft.setTextColor(TFT_RED, TFT_BLACK);  // Динамическое значение выводим, например, красным
    tft.setCursor(xVoltageValue, yVoltage);
    tft.print(currentVoltage);

    prevVoltage = currentVoltage;
  }
}


void updateMessage() {
  String currentMessage = String(messageLine3);
  if (currentMessage != prevMessage) {
    int yMessage = headerHeight + voltHeight;
    tft.fillRect(0, yMessage, SCREEN_WIDTH, msgHeight, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(0, yMessage);
    tft.print(currentMessage);
    prevMessage = currentMessage;
  }
}
void updateDeviceList() {
  int yDevices = listYStart;
  // Здесь можно задать постоянную область для списка (например, до конца экрана)
  //tft.fillRect(0, yDevices, SCREEN_WIDTH, SCREEN_HEIGHT - yDevices, TFT_BLACK);
  tft.setTextSize(1);
  for (int i = 0; i < min(deviceCount, 4); i++) {
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(0, yDevices);

    char buf[32];
    snprintf(buf, sizeof(buf), "P:0x%X D:", devices[i].pid);
    tft.print(buf);
    for (int j = 0; j < devices[i].dataLength; j++) {
      char tmp[6];
      snprintf(tmp, sizeof(tmp), "0x%X ", devices[i].data[j]);
      tft.print(tmp);
    }
    snprintf(buf, sizeof(buf), "C:0x%X ", devices[i].checksum);
    tft.print(buf);
    yDevices += 16;  // Высота строки для маленького шрифта
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// =========================================================
// Функция detectLINBaudRate
void detectLINBaudRate() {
  for (long baud : BAUD_RATES) {
    HARDWARE_SERIAL.begin(baud);
    delay(10);
    if (HARDWARE_SERIAL.available()) {
      detectedBaudRate = baud;
      Serial.print("Detected LIN Baud Rate: ");
      Serial.println(baud);
      break;
    }
  }
}

// =========================================================
// Функция sendMessageE9
void sendMessageE9(const byte data[4]) {
  sendBreak(baudRateE9);
  byte frame[7];
  frame[0] = 0x55;
  frame[1] = 0xE9;
  for (int i = 0; i < 4; i++) {
    frame[i + 2] = data[i];
  }
  byte checksum = calculateChecksum(frame, 7, enhancedChecksumE9);
  HARDWARE_SERIAL.write(0xE9);
  for (int i = 0; i < 4; i++) {
    HARDWARE_SERIAL.write(data[i]);
  }
  HARDWARE_SERIAL.write(checksum);
  HARDWARE_SERIAL.flush();
  Serial.println();
  Serial.print("Sent PID:0xE9: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(" Data: 0x");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print("Checksum: ");
  Serial.println(checksum, HEX);

  // Обновляем сообщение для вывода на дисплей
  char tmp[16];
  strcpy(messageLine3, "E9: ");
  for (int i = 0; i < 4; i++) {
    sprintf(tmp, "%X ", data[i]);
    strcat(messageLine3, tmp);
  }
  sprintf(tmp, "C:%X", checksum);
  strcat(messageLine3, tmp);
}

// =========================================================
// Функция sendMessageEC
void sendMessageEC(const byte data[5]) {
  if (enhancedChecksumEC) {
    byte frame[8];
    frame[0] = 0x55;
    frame[1] = 0xEC;
    for (int i = 0; i < 5; i++) {
      frame[i + 2] = data[i];
    }
    byte checksum = calculateChecksum(frame, 8, true);
    sendBreak(baudRateEC);
    HARDWARE_SERIAL.write(0xEC);
    for (int i = 0; i < 5; i++) {
      HARDWARE_SERIAL.write(data[i]);
    }
    HARDWARE_SERIAL.write(checksum);
    HARDWARE_SERIAL.flush();
    Serial.println();
    Serial.print("Sent PID:0xEC (LIN2.x, 5 bytes): ");
    for (int i = 0; i < 5; i++) {
      Serial.print(" Data: 0x");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print("Checksum: 0x");
    Serial.println(checksum, HEX);

    // Обновляем сообщение для дисплея
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
    for (int i = 0; i < 4; i++) {
      frame[i + 2] = data[i];
    }
    byte checksum = calculateChecksum(frame, 7, false);
    sendBreak(baudRateEC);
    HARDWARE_SERIAL.write(0xEC);
    for (int i = 0; i < 4; i++) {
      HARDWARE_SERIAL.write(data[i]);
    }
    HARDWARE_SERIAL.write(checksum);
    HARDWARE_SERIAL.flush();
    Serial.println();
    Serial.print("Sent PID:0xEC (LIN1.x, 4 bytes): ");
    for (int i = 0; i < 4; i++) {
      Serial.print(" Data: 0x");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print("Checksum: 0x");
    Serial.println(checksum, HEX);
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

// =========================================================
// Функция updateMyDataEC
void updateMyDataEC() {
  if (enhancedChecksumEC) {
    myDataEC[0] = voltLIN2[valueLIN];
    myDataEC[1] = 0x00;
    myDataEC[2] = 0x55;
    if (buttonHeld) {
      myDataEC[3] = 0x50;
    } else {
      myDataEC[3] = 0x8C;
    }
    myDataEC[4] = 0x13;
  } else {
    myDataEC[0] = voltLIN2[valueLIN];
    myDataEC[1] = 0xC7;
    myDataEC[2] = 0xA8;
    if (buttonHeld) {
      myDataEC[3] = 0xDA;
    } else {
      myDataEC[3] = 0x8C;
    }
  }
}

// =========================================================
// Функция updateMyDataE9
void updateMyDataE9() {
  if (isClassicMode) {
    myDataE9[0] = voltLIN1[valueLIN];
  } else if (isEnhancedMode) {
    myDataE9[0] = voltLIN2[valueLIN];
  }
  myDataE9[1] = 0x00;
  myDataE9[2] = 0xFF;
  myDataE9[3] = 0x00;
}

// =========================================================
// Функция encoderTask
void encoderTask() {
  static bool initialized = false;
  if (!initialized) {
    pinMode(CLK, INPUT_PULLUP);
    pinMode(DT, INPUT_PULLUP);
    pinMode(SW, INPUT_PULLUP);
    attachInterrupt(
      digitalPinToInterrupt(CLK), [] {
        enc.tick();
      },
      CHANGE);
    attachInterrupt(
      digitalPinToInterrupt(DT), [] {
        enc.tick();
      },
      CHANGE);
    initialized = true;
  }
  enc.tick();
  if (enc.isRight() && valueLIN < 29) {
    valueLIN++;
  }
  if (enc.isLeft() && valueLIN > 0) {
    valueLIN--;
  }
  if (enc.isTurn()) {
    Serial.print("Value: ");
    Serial.println(valueLIN);
    Serial.print("Set Voltage: ");
    Serial.println(setVOLT[valueLIN]);
    // Здесь можно обновлять сообщение на дисплее, если требуется
    Serial.print("voltLIN1: ");
    Serial.println(voltLIN1[valueLIN], HEX);
  }
  if (enc.isHold()) {
    buttonHeld = true;
  } else if (enc.isRelease()) {
    buttonHeld = false;
  }
}

// ------------------------
//  Веб-интерфейс: обработчики кнопок
// ------------------------
void handleVoltUp() {
  if (valueLIN < 29) {                               // Если текущий индекс напряжения меньше максимума (29)
    valueLIN++;                                      // Увеличиваем индекс, то есть повышаем напряжение
    Serial.print("Voltage increased. New index: ");  // Выводим сообщение о повышении напряжения в Serial Monitor
    Serial.println(valueLIN);                        // Выводим новый индекс напряжения
  }
  // Отправляем HTML-страницу с автоматическим обновлением страницы через скрипт
  //server.send(200, "text/html", "<html><body>Voltage increased. <script>setTimeout(function(){location.reload();}, 50);</script></body></html>");

  // Отправляем HTML-страницу с подтверждением и ссылкой для возврата на главную страницу
  server.sendHeader("Location", "/");               // Устанавливаем заголовок редиректа на главную страницу
  server.send(303, "text/html", "Redirecting...");  // Отправляем ответ с кодом 303
}

void handleVoltDown() {
  if (valueLIN > 0) {                                // Если текущий индекс напряжения больше 0
    valueLIN--;                                      // Уменьшаем индекс, то есть понижаем напряжение
    Serial.print("Voltage decreased. New index: ");  // Выводим сообщение о понижении напряжения
    Serial.println(valueLIN);                        // Выводим новый индекс напряжения
  }
  // Отправляем HTML-страницу с автоматическим обновлением страницы через скрипт
  //server.send(200, "text/html", "<html><body>Voltage decreased. <script>setTimeout(function(){location.reload();}, 50);</script></body></html>");

  // Отправляем HTML-страницу с подтверждением и ссылкой для возврата на главную страницу
  server.sendHeader("Location", "/");               // Устанавливаем заголовок редиректа на главную страницу
  server.send(303, "text/html", "Redirecting...");  // Отправляем ответ с кодом 303
}

// Новая функция-обработчик для переключения состояния "encoder button"
void handleToggleEncoder() {
  buttonHeld = !buttonHeld;                               // Переключаем состояние: если было true – станет false, и наоборот
  String stateStr = buttonHeld ? "Pressed" : "Released";  // Формируем строку с текущим состоянием
  Serial.print("Encoder button state changed to: ");
  Serial.println(stateStr);
  // Отправляем HTML-страницу с автоматическим обновлением страницы через скрипт
  //server.send(200, "text/html", "<html><body>Encoder button state is now " + stateStr + ". <script>setTimeout(function(){location.reload();}, 50);</script></body></html>");

  // Отправляем HTML-страницу с подтверждением и ссылкой для возврата на главную страницу
  server.sendHeader("Location", "/");               // Устанавливаем заголовок редиректа на главную страницу
  server.send(303, "text/html", "Redirecting...");  // Отправляем ответ с кодом 303
}

// ------------------------
//  Веб-интерфейс: главная страница
// ------------------------
void handleRoot() {
  // Вычисление контрольных сумм для исходящих сообщений
  byte checksumE9 = 0;
  {
    byte frameE9[7];
    frameE9[0] = 0x55;
    frameE9[1] = 0xE9;
    for (int i = 0; i < 4; i++) {
      frameE9[i + 2] = myDataE9[i];
    }
    checksumE9 = calculateChecksum(frameE9, 7, enhancedChecksumE9);
  }

  byte checksumEC = 0;
  {
    if (enhancedChecksumEC) {
      byte frameEC[8];
      frameEC[0] = 0x55;
      frameEC[1] = 0xEC;
      for (int i = 0; i < 5; i++) {
        frameEC[i + 2] = myDataEC[i];
      }
      checksumEC = calculateChecksum(frameEC, 8, true);
    } else {
      byte frameEC[7];
      frameEC[0] = 0x55;
      frameEC[1] = 0xEC;
      for (int i = 0; i < 4; i++) {
        frameEC[i + 2] = myDataEC[i];
      }
      checksumEC = calculateChecksum(frameEC, 7, false);
    }
  }

  // Поиск устройства с PID 0x92 или 0xD6 для отображения ID и связанных параметров
  int idByte = -1;
  String idLetter = "";
  String protocolVersion = "N/A";
  String deviceSpeedStr = "N/A";
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].pid == 0x92 || devices[i].pid == 0xD6) {
      if (devices[i].dataLength > 0) {
        idByte = devices[i].data[0];
        if (devices[i].responseH) idLetter = "H";
        else if (devices[i].responseM) idLetter = "M";
        else if (devices[i].responseL) idLetter = "L";
        protocolVersion = devices[i].enhanced ? "LIN 2.x" : "LIN 1.x";
        deviceSpeedStr = String(devices[i].baudRate);
        break;
      }
    }
  }
  String idDisplay = (idByte != -1) ? (String(idByte) + " (" + idLetter + ")") : "N/A";

  // Формирование HTML страницы
  String html = "<html><head><meta charset='UTF-8'>";
  html += "<meta http-equiv='refresh' content='3'>";
  html += "<title>LIN Protocol Analysis</title>";
  html += "<style>"
          "body { font-family: Arial, sans-serif; background: #f4f4f4; margin: 0; padding: 20px; }"
          "table { width: 100%; border-collapse: collapse; margin-bottom: 20px; }"
          "th, td { border: 1px solid #ddd; padding: 8px; text-align: center; }"
          "th { background-color: #007ACC; color: white; }"
          "tr:nth-child(even) { background-color: #e6f2ff; }"
          ".voltage { font-weight: bold; color: #1a1a1a; }"
          ".mode { font-style: italic; color: #333; }"
          ".refresh-button { background-color: #007ACC; color: white; border: none; padding: 10px 20px; cursor: pointer; margin: 5px; }"
          ".refresh-button:hover { background-color: #005999; }"
          /* Изменённые стили для байтов, PID и контрольной суммы */
          ".pid { font-size: 28px; color: orange; }"
          ".byte { color: red; }"
          ".checksum { color: purple; }"
          ".byte-box { display: inline-block; border: 1px solid #ccc; padding: 4px 6px; margin: 4px; border-radius: 4px; font-family: monospace; }"
          ".voltage-display { font-size: 32px; font-weight: bold; margin: 0 20px; display: inline-block; vertical-align: middle; }"
          ".header-row { display: flex; justify-content: space-around; align-items: center; margin: 20px 0; }"
          ".header-row div { font-size: 24px; }"
          /* Логотип LIN: буквы с синим, зелёным, жёлтым цветом, красная точка позиционируется над буквой I */
          ".logo { font-size: 72px; font-weight: Comic Sans MS; }"
          ".logo span.blue { color: #007ACC; }"
          ".logo span.green { color: #00CC66; position: relative; display: inline-block; }"
          ".logo span.yellow { color: #FFCC00; }"
          ".logo span.red { color: #FF0000; position: absolute; left: 50%; transform: translateX(-50%); top: -0.8em; }"
          ".footer { font-size: 14px; text-align: center; margin-top: 30px; border-top: 1px solid #ccc; padding-top: 10px; }"
          "</style></head><body>";

  // Заголовок и панель кнопок с напряжением между кнопками Volt+ и Volt-
  html += "<h1>LIN Protocol Analysis</h1>";
  html += "<div style='margin-bottom:20px; text-align: center;'>";
  html += "<button class='refresh-button' onclick=\"location.href='/volt_down';\">Volt -</button> ";
  html += "<span class='voltage-display'>" + String(setVOLT[valueLIN]) + " V</span>";
  html += "<button class='refresh-button' onclick=\"location.href='/volt_up';\">Volt +</button>";
  html += "<button class='refresh-button' onclick=\"location.href='/toggle_encoder';\">Start Button</button>";
  html += "<button class='refresh-button' onclick='location.reload();'>Refresh</button>";
  html += "</div>";

  // Горизонтальный блок с логотипом и информацией (расположен слева направо)
  html += "<div class='header-row'>";
  html += "<div class='logo'><span class='blue'>L</span><span class='green'>I"
          "<span class='red'>•</span></span><span class='yellow'>N</span></div>";
  html += "<div>ID:</div>";
  html += "<div>" + idDisplay + "</div>";
  html += "<div>Speed: " + deviceSpeedStr + "</div>";
  html += "<div>Protocol: " + protocolVersion + "</div>";
  html += "</div>";

  // Таблица входящих LIN ответов
  html += "<h2>Incoming LIN Responses</h2>";
  html += "<table>";
  html += "<tr>"
          "<th>№</th>"
          "<th><span class='pid'>PID</span></th>"
          "<th>Baud Rate</th>"
          "<th>Protocol</th>"
          "<th>Response Flags</th>"
          "<th><span class='byte'>Data</span></th>"
          "<th><span class='checksum'>Checksum</span></th>"
          "<th>Timestamp (ms)</th>"
          "</tr>";
  for (int i = 0; i < deviceCount; i++) {
    html += "<tr>";
    html += "<td>" + String(i + 1) + "</td>";
    html += "<td><span class='pid'>0x" + String(devices[i].pid, HEX) + "</span></td>";
    html += "<td>" + String(devices[i].baudRate) + "</td>";
    html += "<td class='mode'>" + String(devices[i].enhanced ? "LIN 2.x" : "LIN 1.x") + "</td>";
    String flags = "";
    if (devices[i].responseL) flags += "L ";
    if (devices[i].responseM) flags += "M ";
    if (devices[i].responseH) flags += "H ";
    html += "<td>" + flags + "</td>";
    String dataStr = "";
    for (int j = 0; j < devices[i].dataLength; j++) {
      dataStr += "<span class='byte-box'><span class='byte'>" + String(j + 1) + ". 0x" + String(devices[i].data[j], HEX) + "</span></span>";
    }
    html += "<td>" + dataStr + "</td>";
    html += "<td><span class='checksum'>0x" + String(devices[i].checksum, HEX) + "</span></td>";
    html += "<td>" + String(devices[i].Timestamp) + "</td>";
    html += "</tr>";
  }
  html += "</table>";

  // Таблица исходящих сообщений LIN (с контрольной суммой)
  html += "<h2>Outgoing LIN Messages</h2>";
  html += "<table>";
  html += "<tr>"
          "<th><span class='pid'>PID</span></th>"
          "<th><span class='byte'>Data Bytes</span></th>"
          "<th>Voltage</th>"
          "<th>Mode</th>"
          "<th><span class='checksum'>Checksum</span></th>"
          "</tr>";
  // Сообщение PID 0xE9
  html += "<tr>";
  html += "<td><span class='pid'>0xE9</span></td>";
  html += "<td>";
  for (int i = 0; i < 4; i++) {
    html += "<span class='byte-box'><span class='byte'>" + String(i + 1) + ". 0x" + String(myDataE9[i], HEX) + "</span></span>";
  }
  html += "</td>";
  String voltageStr = setVOLT[valueLIN];
  String voltageHex = isClassicMode ? "0x" + String(voltLIN1[valueLIN], HEX) : "0x" + String(voltLIN2[valueLIN], HEX);
  html += "<td class='voltage'>" + voltageStr + " (" + voltageHex + ")</td>";
  html += "<td class='mode'>" + String(isClassicMode ? "LIN 1.x" : "LIN 2.x") + "</td>";
  html += "<td><span class='checksum'>0x" + String(checksumE9, HEX) + "</span></td>";
  html += "</tr>";

  // Сообщение PID 0xEC
  html += "<tr>";
  html += "<td><span class='pid'>0xEC</span></td>";
  html += "<td>";
  int countEC = enhancedChecksumEC ? 5 : 4;
  for (int i = 0; i < countEC; i++) {
    html += "<span class='byte-box'><span class='byte'>" + String(i + 1) + ". 0x" + String(myDataEC[i], HEX) + "</span></span>";
  }
  html += "</td>";
  String voltageStrEC = setVOLT[valueLIN];
  String voltageHexEC = "0x" + String(voltLIN2[valueLIN], HEX);
  html += "<td class='voltage'>" + voltageStrEC + " (" + voltageHexEC + ")</td>";
  html += "<td class='mode'>" + String(enhancedChecksumEC ? "LIN 2.x" : "LIN 1.x") + "</td>";
  html += "<td><span class='checksum'>0x" + String(checksumEC, HEX) + "</span></td>";
  html += "</tr>";
  html += "</table>";

  // Нижний блок с подробной информацией о состоянии глобальных переменных и логике работы кода
  html += "<div class='footer'>";
  html += "Device Count: " + String(deviceCount) + " | ";
  html += "Voltage Index: " + String(valueLIN) + " (" + String(setVOLT[valueLIN]) + ") | ";
  html += "Search Mode: " + String(searchMode ? "ON" : "OFF") + " | ";
  html += "Send Mode: " + String(sendMode ? "Send+Read" : "Read Only") + " | ";
  html += "Detected LIN Baud Rate: " + String(detectedBaudRate) + " | ";
  html += "Encoder Button Held: " + String(buttonHeld ? "Yes" : "No");
  html += "<br>";
  html += "Polling PID Array: [";
  for (unsigned int i = 0; i < sizeof(PID_ARRAY) / sizeof(PID_ARRAY[0]); i++) {
    html += "0x" + String(PID_ARRAY[i], HEX);
    if (i < (sizeof(PID_ARRAY) / sizeof(PID_ARRAY[0]) - 1))
      html += ", ";
  }
  html += "] | Baud Rates: [2800, 9600, 19200]";
  html += "</div>";

  // Отображение "сырых" данных с шины в самой нижней строке
  html += "<div style='font-size: 14px; text-align: center; margin-top: 10px;'>";
  html += "Raw Bus Data: ";
  for (int i = 0; i < linLength; i++) {
    html += "<span class='byte-box'>0x" + String(linBuffer[i], HEX) + "</span> ";
  }
  html += "</div>";

  html += "</body></html>";

  server.send(200, "text/html", html);
}




// =========================================================
// Функция setup
void setup() {
  tft.init();  // Инициализирует дисплей; разрешение берётся из настроек User_Setup.h
  tft.invertDisplay(false);
  tft.setRotation(1);                      // Поворот дисплея (значения 0-3)
  tft.fillScreen(TFT_BLACK);               // Используем цвет TFT_BLACK
  tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Первый параметр — цвет текста, второй — фон (если нужно)
  tft.setTextSize(2);                      // Масштабирование шрифта
  tft.setCursor(0, 0);                     // Установка курсора


  Serial.begin(115200);
  HARDWARE_SERIAL.begin(9600);
  // Автоопределение скорости LIN
  detectLINBaudRate();
  strcpy(messageLine3, "Start");

  // Запускаем WiFi в режиме точки доступа
  WiFi.softAP(ssid, password);             // Инициализируем точку доступа с заданными SSID и паролем
  Serial.println("Access Point started");  // Выводим сообщение о запуске точки доступа
  Serial.println(WiFi.softAPIP());         // Выводим IP-адрес точки доступа

  server.on("/", handleRoot);                         // Регистрируем обработчик для главной страницы (URL "/")
  server.on("/volt_down", handleVoltDown);            // Регистрируем обработчик для URL "/volt_down" (понижение напряжения)
  server.on("/volt_up", handleVoltUp);                // Регистрируем обработчик для URL "/volt_up" (повышение напряжения)
  server.on("/toggle_encoder", handleToggleEncoder);  // Регистрируем обработчик для URL "/toggle_encoder"
  server.begin();                                     // Запускаем веб-сервер
  detectLINBaudRate();                                // Вызываем функцию для обнаружения скорости LIN, с которой работают устройства
}

// =========================================================
// Функция loop
void loop() {
  encoderTask();
  server.handleClient();  // Обрабатываем входящие HTTP-запросы веб-сервера
  if (searchMode) {
    deviceCount = 0;
    for (int i = 0; i < 3; i++) {
      for (byte pid : PID_ARRAY) {
        Serial.print("\nPolling PID: 0x");
        Serial.print(pid, HEX);
        Serial.print(" at ");
        Serial.print(BAUD_RATES[i]);
        Serial.println(" baud");
        sendLINMessage(pid, BAUD_RATES[i]);
        delay(10);
        receiveLINData(BAUD_RATES[i]);
        if (linLength > 3) {
          if (!deviceExists(linPID)) {
            if (deviceCount < MAX_DEVICES) {
              devices[deviceCount].pid = linPID;
              devices[deviceCount].baudRate = BAUD_RATES[i];
              devices[deviceCount].enhanced = isEnhancedValid;
              deviceCount++;
              Serial.print("Device with PID 0x");
              Serial.print(linPID, HEX);
              Serial.println(" found.");
            }
          } else {
            for (int j = 0; j < deviceCount; j++) {
              if (devices[j].pid == linPID) {
                if (BAUD_RATES[i] > devices[j].baudRate) {
                  devices[j].baudRate = BAUD_RATES[i];
                  devices[j].enhanced = isEnhancedValid;
                  Serial.print("Device with PID 0x");
                  Serial.print(linPID, HEX);
                  Serial.print(" updated to higher baud rate: ");
                  Serial.println(BAUD_RATES[i]);
                }
                devices[j].dataLength = linLength - 3;
                for (int k = 0; k < devices[j].dataLength; k++) {
                  devices[j].data[k] = linData[k];
                }
                devices[j].checksum = linChecksum;
                break;
              }
            }
          }
        }
      }
    }
    if (deviceCount > 0) {
      searchMode = false;
      Serial.println("Search complete. Switching to poll mode for found devices.");
    } else {
      delay(10);
    }
  } else {
    bool deviceLost = false;
    for (int i = 0; i < deviceCount; i++) {
      Serial.print("Polling device with PID: 0x");
      Serial.println(devices[i].pid, HEX);
      sendLINMessage(devices[i].pid, devices[i].baudRate);
      delay(1);
      receiveLINData(devices[i].baudRate);
      if (linLength == 0 || (!isClassicValid && !isEnhancedValid)) {
        Serial.print("Device with PID 0x");
        Serial.print(devices[i].pid, HEX);
        Serial.println(" not responding or protocol error.");
        deviceLost = true;
        break;
      } else {
        devices[i].dataLength = linLength - 3;
        for (int k = 0; k < devices[i].dataLength; k++) {
          devices[i].data[k] = linData[k];
        }
        devices[i].checksum = linChecksum;
        if (devices[i].pid == 0x92) {
          updateMyDataE9();
          sendMessageE9(myDataE9);
        } else if (devices[i].pid == 0xD6) {
          updateMyDataEC();
          sendMessageEC(myDataEC);
        } else {
          Serial.print("Polling unknown device with PID: 0x");
          Serial.println(devices[i].pid, HEX);
        }
      }
    }
     // Обновляем только изменившиеся области дисплея:
    updateHeader();
    updateMessage();
    updateDeviceList();
    updateVoltage();
  

    if (deviceLost) {
      Serial.println("Device lost. Returning to search mode.");
      searchMode = true;
      deviceCount = 0;
     tft.fillScreen(TFT_BLACK);
     delay(300);
      strcpy(messageLine3, "Poisk");
    }
   
  }
}
