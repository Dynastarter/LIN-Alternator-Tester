
#include <SoftwareSerial.h>     // Подключаем библиотеку для создания программного Serial (SoftwareSerial)
#include "U8glib.h"             // Подключаем библиотеку U8glib для работы с графическим дисплеем

// Создаём объект u8g для графического дисплея ST7920 128x64 с 4х масштабированием; пины: SCK=13, MOSI=11, CS=10
U8GLIB_ST7920_128X64_4X u8g(13, 11, 10); // u8g – объект дисплея

/****************************************************
 * Пример использования GyverEncoder (TYPE2)        *
 *    *
 *                                                  *
 * Функционал:                                      *
 * - Считывание вращения энкодера и кнопки          *
 * - Отображение данных на графическом дисплее  *
 * - Работа по протоколу LIN: поиск устройств, опрос,  *
 *   отправка данных (PID=0xE9/0xEC)                  *
 ****************************************************/
#include "GyverEncoder.h"  // Подключаем библиотеку для работы с энкодером

// Определяем пины для энкодера
#define CLK 3 // Пин CLK энкодера (подключён к пину 3)
#define DT 2  // Пин DT энкодера (подключён к пину 2)
#define SW 4  // Пин кнопки энкодера (подключён к пину 4)

// Создаём объект энкодера с типом TYPE2 (поддержка кнопки)
Encoder enc(CLK, DT, SW, TYPE2); // enc – объект энкодера

// Объявляем булевые переменные для отслеживания нажатия кнопки энкодера
bool buttonPressed = false;  // Флаг для короткого нажатия
bool buttonHeld = false;     // Флаг для долгого удержания кнопки



// Здесь выбираем, какой порт будем использовать:
//  - USE_SERIAL1 (пины TX=18, RX=19) - аппаратный Serial1,
//  - USE_SERIAL2,
//  - USE_SERIAL3,
//  - USE_SOFTWARE_SERIAL.
// По умолчанию раскомментировано USE_SERIAL1.

#define USE_SERIAL1
// #define USE_SERIAL2
// #define USE_SERIAL3
// #define USE_SOFTWARE_SERIAL

#ifdef USE_SERIAL1
  #define HARDWARE_SERIAL Serial1 // HARDWARE_SERIAL – это Serial1
  #define TX_PIN 18               // TX-пин для Serial1 (пин 18)
  #define RX_PIN 19               // RX-пин для Serial1 (пин 19)
#elif defined(USE_SERIAL2)
  #define HARDWARE_SERIAL Serial2
  #define TX_PIN 16
  #define RX_PIN 17
#elif defined(USE_SERIAL3)
  #define HARDWARE_SERIAL Serial3
  #define TX_PIN 14
  #define RX_PIN 15
#elif defined(USE_SOFTWARE_SERIAL)
  #define HARDWARE_SERIAL linSerial
  #define TX_PIN 10
  #define RX_PIN 11
  SoftwareSerial linSerial(RX_PIN, TX_PIN); // Создаём объект SoftwareSerial
#else
  #error "Выберите один из доступных портов!" // Если ни один не выбран – ошибка компиляции
#endif

// Массивы виртуальных значений напряжений для двух режимов LIN
int voltLIN1[] = { 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 
                   0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C }; // Классический режим

int voltLIN2[] = { 0x3C, 0x40, 0x44, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C, 0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 
                   0x78, 0x7C, 0x80, 0x84, 0x88, 0x8C, 0x90, 0x94, 0x98, 0x9C, 0xA0, 0xA4, 0xA8, 0xAC, 0xB0 }; // Расширенный режим
/*
 int voltLIN1[] = { 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 
                   0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C }; // этот набор байтов работает с LIN:244(F4)                
*/
// Массив строк для отображения напряжений на 
const char* setVOLT[] = { "12.1", "12.2", "12.3", "12.4", "12.5", "12.6", "12.7", "12.8", "12.9", "13.0", "13.1", "13.2", "13.3", "13.4", "13.5", "13.6", "13.7", "13.8", "13.9", "14.0", "14.1", "14.2", "14.3", "14.4", "14.5", "14.6", "14.7", "14.8", "14.9", "15.0" }; // Строковые значения напряжения

// Переменная valueLIN определяет индекс выбранного напряжения (0..29)
int valueLIN = 9; // Изначально выбран индекс 9, что соответствует "13.0"

// Массивы для формирования LIN-пакетов (отправка)
// myDataE9 – 4 байта для пакета с PID=0xE9
byte myDataE9[4] = { 0x00, 0x00, 0x00, 0x00 }; // [0]=0x00, [1]=0x00, [2]=0x00, [3]=0x00
// myDataEC – 5 байт для пакета с PID=0xEC (в расширенном режиме); в классическом используется 4 байта
byte myDataEC[5] = { 0x00, 0xC7, 0xA8, 0x8C, 0x13 }; // [0]=0x00, [1]=0xC7, [2]=0xA8, [3]=0x8C, [4]=0x13

// Структура Device для хранения информации об обнаруженных устройствах
struct Device {
  byte pid;         // PID устройства
  long baudRate;    // Скорость, на которой устройство отвечает (4800, 9600, 19200)
  bool enhanced;    // true, если устройство поддерживает расширенный режим LIN (LIN2.x)
  int dataLength;   // Количество полученных байт данных (без учета синхробайта, PID, CS)
  byte data[10];    // Массив для хранения полученных байт данных (до 8 байт)
  byte checksum;    // Последняя полученная контрольная сумма
};

#define MAX_DEVICES 10                // Максимальное количество устройств, которое можно обнаружить
Device devices[MAX_DEVICES];          // Массив структур для хранения устройств
int deviceCount = 0;                  // Количество найденных устройств

// Функция deviceExists проверяет, существует ли уже устройство с данным PID в массиве devices
bool deviceExists(byte pid) {
  for (int i = 0; i < deviceCount; i++) { // Перебираем все устройства от 0 до deviceCount-1
    if (devices[i].pid == pid) return true; // Если PID совпадает, возвращаем true
  }
  return false; // Если устройство не найдено, возвращаем false
}

bool searchMode = true; // Флаг searchMode: true – режим поиска устройств, false – режим опроса найденных устройств

// Массив PID для поиска устройств по LIN
const byte PID_ARRAY[] = { 0x11, 0x14, 0x55, 0x61, 0x80, 0x92, 0x97, 0xA8, 0xD3, 0xD6, 0xD8, 0xE7, 0xE9, 0xEC, 0x73, 0xF0 }; // Массив PID для опроса
const long BAUD_RATES[] = { 4800, 9600, 19200 }; // Массив скоростей LIN

long detectedBaudRate = 9600; // Переменная для хранения обнаруженной скорости LIN

// Глобальные переменные для LIN-обмена
byte linPID = 0;           // Переменная для хранения полученного PID
byte linData[10];          // Массив для хранения полученных байт данных
byte linChecksum = 0;      // Переменная для хранения полученной контрольной суммы

bool isClassicValid = false;   // Флаг, указывающий, совпала ли классическая контрольная сумма
bool isEnhancedValid = false;  // Флаг, указывающий, совпала ли расширенная контрольная сумма

bool isClassicMode = false;    // Флаг, выбран ли классический режим для отправки
bool isEnhancedMode = false;   // Флаг, выбран ли расширенный режим для отправки

// Флаги, указывающие, на какой скорости (4800, 9600, 19200) было получено сообщение
bool responseL = false; // Флаг для скорости 4800
bool responseM = false; // Флаг для скорости 9600
bool responseH = false; // Флаг для скорости 19200

byte linBuffer[12]; // Буфер для временного хранения принятых байт по LIN
int linLength = 0;  // Переменная для хранения количества принятых байт

// Переменные для запоминания настроек ответа для PID=0xE9 и PID=0xEC
long baudRateE9 = 9600;         // Скорость, на которой устройство ответило на PID=0x92 (для дальнейшей отправки E9)
bool enhancedChecksumE9 = false; // Флаг, показывающий, использовался ли расширенный режим для PID=0x92

long baudRateEC = 9600;          // Скорость, на которой устройство ответило на PID=0xD6 (для отправки EC)
bool enhancedChecksumEC = false; // Флаг для PID=0xD6, показывающий, использовался ли расширенный режим

// Переменные для хранения первого байта ответа для PID=0x92 и 0xD6
byte lastD6Value = 0; // Сохраняем первый байт ответа для PID=0xD6
byte last92Value = 0; // Сохраняем первый байт ответа для PID=0x92
bool has92 = false;   // Флаг, что PID 0x92 был получен
bool hasD6 = false;   // Флаг, что PID 0xD6 был получен

// Параметры для графического дисплея U8glib
const uint8_t charHeight = 8;  // Высота символа в пикселях
const uint8_t charWidth = 8;   // Ширина символа в пикселях
char messageLine3[64] = "";    // Строка для вывода дополнительного сообщения на графическом дисплее

// =====================================================================
// Функция calculateChecksum() — вычисляет контрольную сумму для LIN-кадра
// data[] – массив байт кадра, length – его длина, isEnhanced – режим расчёта
// =====================================================================
byte calculateChecksum(byte data[], int length, bool isEnhanced) {
  uint16_t sum = 0; // Инициализируем сумму нулём (16 бит для учета переноса)
  int startIndex = isEnhanced ? 1 : 2; // Если расширенный режим, начинаем с data[1] (включая PID); иначе с data[2]
  for (int i = startIndex; i < length - 1; i++) { // Перебираем байты от startIndex до (length-2)
    sum += data[i]; // Прибавляем очередной байт к сумме
    if (sum > 0xFF) { // Если сумма превышает 255
      sum = (sum & 0xFF) + 1; // Применяем перенос (оставляем младшие 8 бит и прибавляем 1)
    }
  }
  return (byte)(~sum); // Возвращаем инверсию суммы (побитовое отрицание)
}

// =====================================================================
// Функция sendBreak() — формирует Break (низкий уровень ~13 бит), затем HIGH, затем отправляет Sync=0x55
// =====================================================================
void sendBreak(long baudRate) {
  HARDWARE_SERIAL.end();              // Останавливаем текущий Serial для освобождения TX
  pinMode(TX_PIN, OUTPUT);            // Устанавливаем TX_PIN как OUTPUT
  digitalWrite(TX_PIN, LOW);          // Ставим TX_PIN в LOW
  
  unsigned long breakDuration = (1000000UL * 13) / baudRate; // Вычисляем время, необходимое для 13 бит в микросекундах
  unsigned long startTime = micros();  // Получаем текущее время в микросекундах
  while (micros() - startTime < breakDuration) { } // Ждём, пока не пройдет breakDuration
  
  digitalWrite(TX_PIN, HIGH);         // Поднимаем TX_PIN в HIGH
  unsigned long highDuration = (1000000UL / baudRate); // Вычисляем время для 1 бита HIGH
  startTime = micros();               // Снова получаем время
  while (micros() - startTime < highDuration) { } // Ждём highDuration
  
  HARDWARE_SERIAL.begin(baudRate);    // Перезапускаем Serial на нужной скорости
  startTime = micros();               // Дополнительная пауза
  while (micros() - startTime < highDuration) { } // Ждём ещё 1 бит по времени
  
  HARDWARE_SERIAL.write(0x55);        // Отправляем синхробайт 0x55
  HARDWARE_SERIAL.flush();            // Ждём завершения отправки
}

// =====================================================================
// Функция sendLINMessage() — отправляет LIN-кадр, состоящий из Break+Sync и PID
// =====================================================================
void sendLINMessage(byte pid, long baudRate) {
  sendBreak(baudRate);                // Формируем Break и Sync (0x55)
  HARDWARE_SERIAL.write(pid);         // Отправляем PID
  HARDWARE_SERIAL.flush();            // Ждём завершения отправки
}

// =====================================================================
// Функция receiveLINData() — принимает LIN-ответ и обрабатывает его
// =====================================================================
void receiveLINData(long baudRate) {
  unsigned long timeout = millis() + 50; // Устанавливаем таймаут 50 мс для приёма данных
  linLength = 0;                         // Обнуляем счётчик принятых байт
  
  while (millis() < timeout) {           // Пока не истёк таймаут
    if (HARDWARE_SERIAL.available()) {   // Если появились байты во входном буфере
      while (HARDWARE_SERIAL.available() && linLength < 11) { // Читаем, пока байты доступны и не превышен размер буфера
        linBuffer[linLength] = HARDWARE_SERIAL.read(); // Считываем один байт в linBuffer
        linLength++;                   // Увеличиваем количество считанных байт
      }
    }
  }
  
  if (linLength == 0) {                // Если ничего не получено
    Serial.println("No response");     // Выводим сообщение в Serial
    return;                            // Выходим из функции
  }
  
  linPID = linBuffer[1];               // Присваиваем переменной linPID второй байт (PID)
  linChecksum = linBuffer[linLength - 1]; // Последний байт буфера – контрольная сумма
  
  // Копируем полезные данные (между PID и checksum) в массив linData
  for (int i = 2; i < linLength - 1; i++) { 
    linData[i - 2] = linBuffer[i];     // Сохраняем данные, начиная с linData[0]
  }
  
  // Вычисляем ожидаемые контрольные суммы
  byte expectedClassic  = calculateChecksum(linBuffer, linLength, false); // Классическая CS
  byte expectedEnhanced = calculateChecksum(linBuffer, linLength, true);  // Расширенная CS
  
  isClassicValid  = (linChecksum == expectedClassic); // Сравниваем: совпала ли классическая CS
  isEnhancedValid = (linChecksum == expectedEnhanced); // Совпала ли расширенная CS
  
  // Определяем, какой режим использовать для отправки
  if (isEnhancedValid) {             // Если расширенная CS верна
    isEnhancedMode = true;           // Выбираем расширенный режим
    isClassicMode  = false;          // Отключаем классический режим
  } else if (isClassicValid) {       // Если классическая CS верна
    isClassicMode  = true;           // Выбираем классический режим
    isEnhancedMode = false;          // Отключаем расширенный режим
  } else {                           // Если ни одна CS не совпала
    isClassicMode  = false;          // Выключаем оба режима
    isEnhancedMode = false;
  }
  
  // Если полученный PID равен 0x92, запоминаем текущую скорость и режим CS для E9
  if (linPID == 0x92) {
    baudRateE9 = baudRate;           // Записываем скорость
    enhancedChecksumE9 = isEnhancedValid; // Записываем, использовался ли расширенный режим
  }
  
  // Если PID равен 0xD6, аналогично для EC
  if (linPID == 0xD6) {
    baudRateEC = baudRate;           // Запоминаем скорость
    enhancedChecksumEC = isEnhancedValid; // Запоминаем режим CS
  }
  
  // Если в кадре есть полезные данные (linLength > 2)
  if (linLength > 2) {
    if (baudRate == 19200) responseH = true; // Если скорость 19200, ставим флаг responseH
    if (baudRate == 9600)  responseM = true;  // Если скорость 9600, ставим флаг responseM
    if (baudRate == 4800)  responseL = true;  // Если скорость 4800, ставим флаг responseL
    
    Serial.print("Ответ получен на скорости: "); // Выводим сообщение в Serial
    if (baudRate == 19200) Serial.println("H");   // Печатаем "H"
    else if (baudRate == 9600) Serial.println("M"); // Печатаем "M"
    else if (baudRate == 4800) Serial.println("L"); // Печатаем "L"
  }
  
  printLINData();      // Вызываем функцию для печати информации о LIN-ответе в Serial
  if (linLength > 3) { // Если есть хотя бы один байт полезных данных
    //displayLINData();  // Вызываем функцию для отображения данных на LCD
  }
}

// =====================================================================
// Функция printLINData() — выводит в Serial информацию о принятом LIN-кадре
// =====================================================================
void printLINData() {
  Serial.print("PID: 0x");                  // Печатаем префикс "PID: 0x"
  Serial.print(linPID, HEX);                // Печатаем PID в шестнадцатеричном формате
  Serial.print(" | Data: ");                // Печатаем разделитель и префикс для данных
  for (int i = 0; i < linLength - 3; i++) {  // Цикл по всем полезным байтам (исключая sync, PID и checksum)
    Serial.print("0x");                     // Печатаем префикс "0x" для каждого байта
    Serial.print(linData[i], HEX);           // Печатаем байт в HEX
    Serial.print(" ");                       // Пробел между байтами
  }
  Serial.print("| Checksum: 0x");            // Печатаем префикс для контрольной суммы
  Serial.print(linChecksum, HEX);           // Печатаем checksum
  Serial.print(" | Protocol: ");            // Печатаем разделитель и префикс для протокола
  if (isClassicValid && isEnhancedValid) {   // Если оба режима CS совпали
    Serial.println("LIN 1.x & 2.x");         // Печатаем "LIN 1.x & 2.x"
  } else if (isClassicValid) {               // Если совпала только классическая CS
    Serial.println("LIN 1.x");               // Печатаем "LIN 1.x"
  } else if (isEnhancedValid) {              // Если совпала только расширенная CS
    Serial.println("LIN 2.x");               // Печатаем "LIN 2.x"
  } else {
    Serial.println("ERROR");                 // Если ни одна CS не совпала, печатаем "ERROR"
  }
}

// =====================================================================
// Функция displayLINHeader() — выводит заголовок на графический дисплей ST7920
// =====================================================================
void displayLINHeader() {
  u8g.setFont(u8g_font_6x12);               // Устанавливаем шрифт 6x12 для графического дисплея
  char buf[32];                             // Создаём буфер на 32 символа для формирования строки
  u8g.drawStr(0, charHeight, "LIN:");       // Рисуем "LIN:" на координатах (0, charHeight)

  // Если значение last92Value больше 0, формируем строку с этим значением
  if (last92Value > 0) {
    sprintf(buf, "%u", last92Value);        // Формируем строку с числом (например, "146")
    u8g.drawStr(3 * charWidth, charHeight, buf); // Рисуем строку на позиции x = 3*charWidth
  } else if (lastD6Value > 0) {              // Иначе, если lastD6Value больше 0
    sprintf(buf, "%u", lastD6Value);         // Формируем строку с этим значением
    u8g.drawStr(3 * charWidth, charHeight, buf); // Рисуем её
  } else {                                  // Если ни одно значение не установлено
    u8g.drawStr(3 * charWidth, charHeight, "  "); // Рисуем "--"
  }

  // Ещё раз проверяем флаги has92 и hasD6 и выводим значение, если есть
  if (has92) {
    sprintf(buf, "%u", last92Value);        // Формируем строку с last92Value
    u8g.drawStr(3 * charWidth, charHeight, buf); // Рисуем её
  } else if (hasD6) {
    sprintf(buf, "%u", lastD6Value);         // Формируем строку с lastD6Value
    u8g.drawStr(3 * charWidth, charHeight, buf); // Рисуем её
  } else {
    u8g.drawStr(3 * charWidth, charHeight, "  "); // Если нет – рисуем "--"
  }

  // Выводим флаги скорости: если был ответ на 4800, 9600 или 19200, выводим соответствующие буквы
  if (responseL) u8g.drawStr(6 * charWidth, charHeight, "L"); // Если получен ответ на 4800, рисуем "L"
  if (responseM) u8g.drawStr(7 * charWidth, charHeight, "M"); // Если получен ответ на 9600, рисуем "M"
  if (responseH) u8g.drawStr(8 * charWidth, charHeight, "H"); // Если получен ответ на 19200, рисуем "H"

  // Выводим режим протокола: "LIN 1.x" или "LIN 2.x"
  if (isClassicValid) {
    u8g.drawStr(10 * charWidth, charHeight, "LIN 1.x"); // Если классический режим, рисуем "LIN 1.x"
  } else if (isEnhancedValid) {
    u8g.drawStr(10 * charWidth, charHeight, "LIN 2.x"); // Если расширенный, рисуем "LIN 2.x"
  }
}

// =====================================================================
// Функция displayEncoderVoltage() — выводит на графический дисплей текущее напряжение
// =====================================================================
void displayEncoderVoltage() {
  u8g.setFont(u8g_font_6x10);              // Устанавливаем шрифт 6x10 для вывода напряжения
  char buf[16];                            // Буфер на 16 символов для строки напряжения
  sprintf(buf, "Volt: %s", setVOLT[valueLIN]); // Формируем строку вида "Volt: 13.0" на основе valueLIN
  u8g.drawStr(0, 2 * charHeight, buf);     // Рисуем эту строку на координате y = 2 * charHeight (вторая строка)
}

// =====================================================================
// Функция displayLINDetail() — выводит детальную информацию о LIN-кадре на графическом дисплее
// =====================================================================
void displayLINDetail() {
  u8g.setFont(u8g_font_6x10);              // Устанавливаем шрифт 6x10
  char buf[32];                            // Буфер на 32 символа для строки
  if (linPID == 0x92 || linPID == 0xD6) {     // Если PID равен 0x92 или 0xD6
    sprintf(buf, "P:x%X D:", linPID);       // Формируем строку "P:x<PID> D:" (например, "P:x92 D:")
    u8g.drawStr(0, 3 * charHeight, buf);     // Рисуем её на координатах (0, 3 * charHeight)
    char dataStr[32] = "";                   // Буфер dataStr для накопления байтов данных
    for (int i = 0; i < linLength - 3; i++) { // Перебираем каждый полезный байт данных
      char tmp[8];                         // Буфер tmp на 8 символов
      sprintf(tmp, "x%X", linData[i]);       // Преобразуем байт linData[i] в строку вида "x<HEX>"
      strcat(dataStr, tmp);                  // Добавляем tmp к dataStr
    }
    u8g.drawStr(u8g.getStrWidth(buf), 3 * charHeight, dataStr); // Рисуем dataStr рядом с ранее выведенной строкой
    sprintf(buf, " C:x%X", linChecksum);      // Формируем строку для контрольной суммы, например " C:x3F"
    u8g.drawStr(80, 3 * charHeight, buf);      // Рисуем строку для CS на координате x = 80
  } else {                                  // Если PID не равен 0x92/0xD6
    sprintf(buf, "0x%X", linPID);             // Формируем строку вида "0x<PID>"
    u8g.drawStr(0, 3 * charHeight, buf);       // Рисуем её на экране
    char dataStr[32] = "";                    // Буфер для накопления байтов данных
    for (int i = 0; i < linLength - 3; i++) {   // Перебираем полезные данные
      char tmp[8];                          // Буфер tmp на 8 символов
      sprintf(tmp, " %X", linData[i]);       // Преобразуем байт в строку с пробелом, например " A8"
      strcat(dataStr, tmp);                  // Добавляем tmp к dataStr
    }
    u8g.drawStr(u8g.getStrWidth(buf), 3 * charHeight, dataStr); // Рисуем dataStr рядом с первой частью
    sprintf(buf, " %X", linChecksum);         // Формируем строку для контрольной суммы, например " 3F"
    u8g.drawStr(80, 3 * charHeight, buf);      // Рисуем её на x = 80
  }
}

// =====================================================================
// Функция displayMessageLine3() — выводит дополнительное сообщение на графическом дисплее
// =====================================================================
void displayMessageLine3() {
  u8g.setFont(u8g_font_5x7);               // Устанавливаем шрифт 5x7 для мелкого текста
  u8g.drawStr(0, 8 * charHeight, messageLine3); // Рисуем строку messageLine3 на координате y = 8 * charHeight
}

// =====================================================================
// Функция displayDevices() — выводит список обнаруженных устройств на графическом дисплее
// =====================================================================
void displayDevices() {
  u8g.setFont(u8g_font_4x6);                // Устанавливаем самый маленький шрифт 4x6
  char buf[64];                           // Буфер на 64 символа для строки устройства
  int startLine = 4;                      // Начинаем вывод с 4-й строки (y = 4 * charHeight)
  for (int i = 0; i < deviceCount; i++) {  // Перебираем все найденные устройства
    sprintf(buf, "%X:D:", devices[i].pid); // Формируем строку с PID, например "92:D:"
    for (int j = 0; j < devices[i].dataLength; j++) { // Перебираем все байты данных устройства
      char tmp[8];                       // Временный буфер tmp на 8 символов
      sprintf(tmp, "x%X", devices[i].data[j]); // Преобразуем байт в строку вида "x<HEX>"
      strcat(buf, tmp);                  // Добавляем tmp к строке buf
    }
    char tmp[16];                        // Буфер tmp на 16 символов для контрольной суммы
    sprintf(tmp, "C:x%X", devices[i].checksum); // Формируем строку для контрольной суммы, например "C:x3F"
    strcat(buf, tmp);                    // Конкатенируем её к buf
    u8g.drawStr(0, (startLine + i) * charHeight, buf); // Рисуем строку для каждого устройства на соответствующей строке
  }
}

// =====================================================================
// Функция updateDisplay() — объединяет вывод всех элементов на графическом дисплее
// =====================================================================
void updateDisplay() {
  u8g.firstPage();                       // Начинаем процесс перерисовки дисплея
  do {
    displayLINHeader();                  // Выводим заголовок (строка 0)
    displayEncoderVoltage();             // Выводим выбранное напряжение (строка 1)
    //displayLINDetail();                // Вывод детальной информации (можно раскомментировать, если нужно)
    displayMessageLine3();               // Вывод дополнительного сообщения (строка 3)
    displayDevices();                    // Вывод списка устройств (строки 4 и ниже)
  } while (u8g.nextPage());              // Завершаем процесс перерисовки
}
void clearDisplay() {
  // Начинаем новый цикл отрисовки, но ничего не выводим
  u8g.firstPage();
  do {
    // Здесь не рисуем никаких объектов – экран будет очищен
  } while (u8g.nextPage());
}


// =====================================================================
// Функция detectLINBaudRate() — пытается определить активную скорость LIN
// =====================================================================
void detectLINBaudRate() {
  for (long baud : BAUD_RATES) {           // Перебираем скорости из массива BAUD_RATES
    HARDWARE_SERIAL.begin(baud);           // Запускаем Serial на данной скорости
    delay(10);                             // Ждём 10 мс для стабилизации
    if (HARDWARE_SERIAL.available()) {     // Если на этой скорости появились входящие данные
      detectedBaudRate = baud;             // Сохраняем эту скорость в detectedBaudRate
      Serial.print("Detected LIN Baud Rate: ");
      Serial.println(baud);                // Выводим обнаруженную скорость в Serial
      break;                               // Прерываем цикл, так как скорость найдена
    }
  }
}

// =====================================================================
// Функция sendMessageE9() — отправляет пакет с PID=0xE9 и 4 байтами данных (myDataE9)
// =====================================================================
void sendMessageE9(const byte data[4]) {
  sendBreak(baudRateE9);                      // Формируем Break на скорости baudRateE9

  // Собираем кадр из 7 байт: [0]=0x55, [1]=0xE9, [2..5]=data[0..3], [6]=checksum
  byte frame[7];                              // Объявляем массив frame из 7 байт
  frame[0] = 0x55;                            // Первый байт frame[0] = 0x55 (Sync)
  frame[1] = 0xE9;                            // Второй байт frame[1] = 0xE9 (PID)
  for (int i = 0; i < 4; i++) {               // Цикл для копирования 4 байт из data
    frame[i + 2] = data[i];                   // frame[2]..frame[5] = data[0]..data[3]
  }
  byte checksum = calculateChecksum(frame, 7, enhancedChecksumE9); // Вычисляем контрольную сумму для frame

  HARDWARE_SERIAL.write(0xE9);                // Отправляем PID 0xE9
  for (int i = 0; i < 4; i++) {               // Цикл для отправки 4 байт данных
    HARDWARE_SERIAL.write(data[i]);           // Отправляем каждый байт data[i]
  }
  HARDWARE_SERIAL.write(checksum);            // Отправляем контрольную сумму
  HARDWARE_SERIAL.flush();                    // Ждём завершения отправки

  Serial.println();                           // Печатаем пустую строку в Serial
  Serial.print("Sent PID:0xE9: ");              // Выводим префикс для отправленного пакета
  for (int i = 0; i < 4; i++) {               // Цикл для вывода данных в Serial
    Serial.print(" Data: 0x");                 
    Serial.print(data[i], HEX);               
    Serial.print(" ");
  }
  Serial.print("Checksum: ");
  Serial.println(checksum, HEX);              // Выводим контрольную сумму в Serial


  char tmp[16];                               // Объявляем временный буфер tmp
  strcpy(messageLine3, "E9: ");               // Копируем строку "E9: " в messageLine3
  for (int i = 0; i < 4; i++) {               // Цикл для формирования строки messageLine3
    sprintf(tmp, "%X ", data[i]);             // Формируем строку для каждого байта данных
    strcat(messageLine3, tmp);                // Добавляем строку tmp к messageLine3
  }
  sprintf(tmp, "C:%X", checksum);             // Формируем строку для контрольной суммы
  strcat(messageLine3, tmp);                  // Добавляем её к messageLine3
}

// =====================================================================
// Функция sendMessageEC() — отправляет пакет с PID=0xEC. В LIN2.x отправляются 5 байт, в LIN1.x — 4 байта.
// =====================================================================
void sendMessageEC(const byte data[5]) {
  if (enhancedChecksumEC) {                   // Если расширенный режим (LIN2.x)
    byte frame[8];                            // Объявляем массив frame из 8 байт
    frame[0] = 0x55;                          // frame[0] = 0x55 (Sync)
    frame[1] = 0xEC;                          // frame[1] = 0xEC (PID)
    frame[2] = data[0];                       // frame[2] = data[0]
    frame[3] = data[1];                       // frame[3] = data[1]
    frame[4] = data[2];                       // frame[4] = data[2]
    frame[5] = data[3];                       // frame[5] = data[3]
    frame[6] = data[4];                       // frame[6] = data[4]
    byte checksum = calculateChecksum(frame, 8, true); // Вычисляем контрольную сумму для 8 байт frame
    sendBreak(baudRateEC);                    // Формируем Break на скорости baudRateEC
    HARDWARE_SERIAL.write(0xEC);              // Отправляем PID 0xEC
    for (int i = 0; i < 5; i++) {             // Цикл для отправки 5 байт данных
      HARDWARE_SERIAL.write(data[i]);         // Отправляем data[i]
    }
    HARDWARE_SERIAL.write(checksum);          // Отправляем контрольную сумму
    HARDWARE_SERIAL.flush();                  // Ждём завершения отправки

    Serial.println();                         // Печатаем пустую строку в Serial
    Serial.print("Sent PID:0xEC (LIN2.x, 5 bytes): ");
    for (int i = 0; i < 5; i++) {             // Цикл для вывода данных в Serial
      Serial.print(" Data: 0x");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print("Checksum: 0x");
    Serial.println(checksum, HEX);            // Выводим контрольную сумму в Serial

   
  } else {                                    // Если классический режим (LIN1.x)
    byte frame[7];                            // Объявляем массив frame из 7 байт
    frame[0] = 0x55;                          // frame[0] = 0x55 (Sync)
    frame[1] = 0xEC;                          // frame[1] = 0xEC (PID)
    frame[2] = data[0];                       // frame[2] = data[0]
    frame[3] = data[1];                       // frame[3] = data[1]
    frame[4] = data[2];                       // frame[4] = data[2]
    frame[5] = data[3];                       // frame[5] = data[3]
    byte checksum = calculateChecksum(frame, 7, false); // Вычисляем контрольную сумму для 7 байт в классическом режиме
    sendBreak(baudRateEC);                    // Формируем Break на скорости baudRateEC
    HARDWARE_SERIAL.write(0xEC);              // Отправляем PID 0xEC
    for (int i = 0; i < 4; i++) {             // Цикл для отправки 4 байт данных
      HARDWARE_SERIAL.write(data[i]);         // Отправляем data[i]
    }
    HARDWARE_SERIAL.write(checksum);          // Отправляем контрольную сумму
    HARDWARE_SERIAL.flush();                  // Ждём завершения отправки

    Serial.println();                         // Печатаем пустую строку в Serial
    Serial.print("Sent PID:0xEC (LIN1.x, 4 bytes): ");
    for (int i = 0; i < 4; i++) {             // Цикл для вывода данных в Serial
      Serial.print(" Data: 0x");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print("Checksum: 0x");
    Serial.println(checksum, HEX);            // Выводим контрольную сумму в Serial

    char tmp[16];                             // Объявляем временный буфер tmp
    strcpy(messageLine3, "EC: ");             // Копируем строку "EC: " в messageLine3
    for (int i = 0; i < 4; i++) {             // Цикл для формирования строки messageLine3
      sprintf(tmp, "%X ", data[i]);           // Формируем строку для data[i]
      strcat(messageLine3, tmp);              // Добавляем tmp к messageLine3
    }
    sprintf(tmp, "C:%X", checksum);           // Формируем строку для контрольной суммы
    strcat(messageLine3, tmp);                // Добавляем её к messageLine3

    
  }
}

// =====================================================================
// Функция updateMyDataEC() — обновляет массив myDataEC в зависимости от режима и состояния кнопки
// =====================================================================
void updateMyDataEC() {
  if (enhancedChecksumEC) {               // Если используется расширенный режим (LIN2.x)
    myDataEC[0] = voltLIN2[valueLIN];       // [0] Берём значение из voltLIN2 по индексу valueLIN
    myDataEC[1] = 0x00;                     // [1] Устанавливаем 0x00
    myDataEC[2] = 0x55;                     // [2] Устанавливаем 0x55
    if (buttonHeld) {                       // Если кнопка удерживается
      myDataEC[3] = 0x50;                   // [3] Устанавливаем 0x50
    } else {                                // Если кнопка не удерживается
      myDataEC[3] = 0x8C;                   // [3] Устанавливаем 0x8C
    }
    myDataEC[4] = 0x13;                     // [4] Устанавливаем 0x13
  } else {                                // Если используется классический режим (LIN1.x)
    myDataEC[0] = voltLIN2[valueLIN];       // [0] Берём значение из voltLIN2 по индексу valueLIN
    myDataEC[1] = 0xC7;                     // [1] Устанавливаем 0xC7
    myDataEC[2] = 0xA8;                     // [2] Устанавливаем 0xA8
    if (buttonHeld) {                       // Если кнопка удерживается
      myDataEC[3] = 0xDA;                   // [3] Устанавливаем 0xDA
    } else {                                // Если не удерживается
      myDataEC[3] = 0x8C;                   // [3] Устанавливаем 0x8C
    }
    // [4] В классическом режиме не используется
  }
}

// =====================================================================
// Функция updateMyDataE9() — обновляет массив myDataE9 для PID=0xE9
// =====================================================================
void updateMyDataE9() {
  if (isClassicMode) {                    // Если выбран классический режим
    myDataE9[0] = voltLIN1[valueLIN];      // [0] Берём значение из voltLIN1 по индексу valueLIN
  } else if (isEnhancedMode) {            // Если выбран расширенный режим
    myDataE9[0] = voltLIN2[valueLIN];      // [0] Берём значение из voltLIN2 по индексу valueLIN
  }
/*
  myDataE9[1] = 0x00;                     // [1] Устанавливаем 0x00
  myDataE9[2] = 0x00;                     // [2] Устанавливаем 0x00  
  myDataE9[3] = 0x00;                     // [3] Устанавливаем 0x00
*/
  myDataE9[1] = 0x00;                     // [1] Устанавливаем 0x00
  myDataE9[2] = 0xFF;                     // [2] Устанавливаем 0x00  //{ [value], 0x00, 0xFF, 0x00 }; //второй вариант отправки байта регулировки напряжения
  myDataE9[3] = 0x00;                     // [3] Устанавливаем 0x00  // этот набор байтов работает с LIN:244(F4)  ??????
}

// =====================================================================
// Функция encoderTask() — обрабатывает данные энкодера (вращение и нажатия)
// =====================================================================
void encoderTask() {
  static bool initialized = false;         // Флаг, чтобы инициализация пинов и прерываний выполнялась один раз
  if (!initialized) {                        // Если ещё не инициализировано
    pinMode(CLK, INPUT_PULLUP);              // Устанавливаем пин CLK как INPUT с подтяжкой
    pinMode(DT, INPUT_PULLUP);               // Устанавливаем пин DT как INPUT с подтяжкой
    pinMode(SW, INPUT_PULLUP);               // Устанавливаем пин SW как INPUT с подтяжкой

    attachInterrupt(digitalPinToInterrupt(CLK), [] { // Привязываем прерывание к пину CLK
      enc.tick();                           // Вызываем enc.tick() при изменении уровня
    }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DT), [] {  // Привязываем прерывание к пину DT
      enc.tick();                           // Вызываем enc.tick() при изменении уровня
    }, CHANGE);

    // enc.setHoldTimeout(700);             // Можно установить время удержания для определения длинного нажатия
    initialized = true;                    // Устанавливаем флаг инициализации
  }

  enc.tick();                              // Обновляем состояние энкодера

  if (enc.isRight() && valueLIN < 29) {      // Если энкодер вращается вправо и значение меньше 29
    valueLIN++;                            // Увеличиваем valueLIN на 1
  }
  if (enc.isLeft() && valueLIN > 0) {        // Если энкодер вращается влево и значение больше 0
    valueLIN--;                            // Уменьшаем valueLIN на 1
  }

  if (enc.isTurn()) {                      // Если зафиксировано любое вращение (вправо или влево)
    Serial.print("Value: ");              
    Serial.println(valueLIN);              // Выводим текущее значение valueLIN в Serial
    Serial.print("Set Voltage: ");
    Serial.println(setVOLT[valueLIN]);     // Выводим строковое значение напряжения по индексу valueLIN

    

    Serial.print("voltLIN1: ");
    Serial.println(voltLIN1[valueLIN], HEX); // Выводим значение из voltLIN1 в шестнадцатеричном формате
  }
/*
  if (enc.isHold()) {                     // Если кнопка удерживается
    buttonHeld = true;                    // Устанавливаем флаг buttonHeld = true
  } else if (enc.isRelease()) {           // Если кнопка отпущена
    buttonHeld = false;                   // Устанавливаем флаг buttonHeld = false
  }
*/
  buttonHeld = enc.isHolded();
}

// =====================================================================
// Функция setup() — выполняется один раз при старте контроллера
// =====================================================================
void setup() {
  u8g.setRot180();                        // Поворачиваем графический дисплей на 180° (если нужно)
  strcpy(messageLine3, "Start");            // Инициализируем messageLine3 строкой "Start"
  Serial.begin(115200);                     // Запускаем USB Serial для отладки на скорости 115200
  HARDWARE_SERIAL.begin(9600);              // Запускаем аппаратный LIN-порт на скорости 9600

  

  detectLINBaudRate();                    // Вызываем функцию для определения активной скорости LIN
}

// =====================================================================
// Функция loop() — главный бесконечный цикл программы
// =====================================================================
void loop() {
  encoderTask();                          // 1) Сначала обрабатываем энкодер

  if (searchMode) {                       // 2) Если режим поиска устройств (searchMode == true)
    deviceCount = 0;                      // Обнуляем счётчик найденных устройств
    for (int i = 0; i < 3; i++) {          // Перебираем скорости из массива BAUD_RATES
      for (byte pid : PID_ARRAY) {         // Перебираем каждый PID из PID_ARRAY
        Serial.print("\nPolling PID: 0x");  
        Serial.print(pid, HEX);            // Выводим PID в шестнадцатеричном формате
        Serial.print(" at ");
        Serial.print(BAUD_RATES[i]);       // Выводим скорость
        Serial.println(" baud");

        sendLINMessage(pid, BAUD_RATES[i]); // Отправляем запрос с данным PID и скоростью
        delay(10);                         // Ждём 10 мс
        receiveLINData(BAUD_RATES[i]);      // Принимаем ответ на заданной скорости

        if (linPID == 0x92 && linLength > 3) { // Если получен PID 0x92 и есть данные
          last92Value = linData[0];         // Сохраняем первый байт данных в last92Value
          has92 = true;                     // Устанавливаем флаг has92
        }
        if (linPID == 0xD6 && linLength > 3) { // Если получен PID 0xD6 и есть данные
          lastD6Value = linData[0];         // Сохраняем первый байт данных в lastD6Value
          hasD6 = true;                     // Устанавливаем флаг hasD6
        }

        if (linLength > 3) {                // Если в кадре есть полезные данные (linLength > 3)
          if (!deviceExists(linPID)) {      // Проверяем, нет ли уже устройства с таким PID
            if (deviceCount < MAX_DEVICES) { // Если ещё есть место в массиве устройств
              devices[deviceCount].pid = linPID;           // Сохраняем PID в устройство
              devices[deviceCount].baudRate = BAUD_RATES[i]; // Сохраняем скорость
              devices[deviceCount].enhanced = isEnhancedValid; // Сохраняем, какой режим CS использовался
              deviceCount++;              // Увеличиваем счётчик устройств
              Serial.print("Device with PID 0x");
              Serial.print(linPID, HEX);
              Serial.println(" found.");
            }
          } else {                          // Если устройство с таким PID уже существует
            for (int j = 0; j < deviceCount; j++) { // Ищем его в массиве
              if (devices[j].pid == linPID) { // Если найдено устройство с этим PID
                if (BAUD_RATES[i] > devices[j].baudRate) { // Если текущая скорость выше сохранённой
                  devices[j].baudRate = BAUD_RATES[i]; // Обновляем скорость
                  devices[j].enhanced = isEnhancedValid; // Обновляем режим CS
                  Serial.print("Device with PID 0x");
                  Serial.print(linPID, HEX);
                  Serial.print(" updated to higher baud rate: ");
                  Serial.println(BAUD_RATES[i]);
                }
                devices[j].dataLength = linLength - 3; // Обновляем длину полученных данных
                for (int k = 0; k < devices[j].dataLength; k++) { // Копируем данные в устройство
                  devices[j].data[k] = linData[k];
                }
                devices[j].checksum = linChecksum; // Сохраняем контрольную сумму
                break;                             // Прерываем цикл, так как устройство найдено
              }
            }
          }
        }
      }
    }
    if (deviceCount > 0) {                  // Если найдено хотя бы одно устройство
      searchMode = false;                 // Выходим из режима поиска (переходим в режим опроса)
      Serial.println("Search complete. Switching to poll mode for found devices.");
    } else {
      delay(10);                          // Иначе ждём немного и повторяем поиск
    }
  } else {                                // 3) Режим опроса найденных устройств (searchMode == false)
    bool deviceLost = false;              // Флаг, что устройство потеряно
    for (int i = 0; i < deviceCount; i++) { // Перебираем все найденные устройства
      Serial.print("Polling device with PID: 0x");
      Serial.println(devices[i].pid, HEX);  // Выводим PID устройства

      sendLINMessage(devices[i].pid, devices[i].baudRate); // Отправляем запрос устройству с сохранённой скоростью
      delay(1);                           // Небольшая задержка
      receiveLINData(devices[i].baudRate); // Принимаем ответ от устройства

      if (linLength == 0 || (!isClassicValid && !isEnhancedValid)) { // Если нет ответа или CS не совпадает
        Serial.print("Device with PID 0x");
        Serial.print(devices[i].pid, HEX);
        Serial.println(" not responding or protocol error.");
        deviceLost = true;                // Устанавливаем флаг, что устройство потеряно
        break;                            // Прерываем опрос
      } else {                            // Если устройство ответило корректно
        devices[i].dataLength = linLength - 3; // Обновляем длину полученных данных
        for (int k = 0; k < devices[i].dataLength; k++) { // Копируем данные в устройство
          devices[i].data[k] = linData[k];
        }
        devices[i].checksum = linChecksum; // Сохраняем контрольную сумму

        if (devices[i].pid == 0x92) {       // Если PID устройства равен 0x92
          updateMyDataE9();                // Обновляем массив myDataE9
          sendMessageE9(myDataE9);         // Отправляем пакет с PID=0xE9
        } else if (devices[i].pid == 0xD6) {  // Если PID устройства равен 0xD6
          updateMyDataEC();                // Обновляем массив myDataEC
          sendMessageEC(myDataEC);         // Отправляем пакет с PID=0xEC
        } else {                          // Если устройство имеет другой PID
          Serial.print("Polling unknown device with PID: 0x");
          Serial.println(devices[i].pid, HEX);
        }
      }
    }
    if (deviceLost) {                     // Если устройство потеряно
      Serial.println("Device lost. Returning to search mode.");
      searchMode = true;                  // Возвращаемся в режим поиска
      deviceCount = 0;                    // Сбрасываем список устройств
     clearDisplay();
     
    }
    updateDisplay();                      // Обновляем графический дисплей ST7920
    delay(10);                            // Небольшая задержка
  }
}


