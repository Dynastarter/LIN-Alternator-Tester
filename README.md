# LIN-Alternator-Tester by Dynastarter

This Arduino code is a fairly comprehensive implementation for working with the LIN bus:

It can detect devices, poll them, and send control commands taking into account various LIN modes (1.x and 2.x).

It implements checksum and synchronization processing via the Break signal.

It also provides interactive input using an encoder, allowing you to change parameters (for example, voltage), as well as visual display of data on two types of displays.

Thus, this project can serve as a basis for diagnostics, testing, or control of a LIN network in automotive systems or other areas where this protocol is used.

=================================================================================================

1. Used libraries and hardware modules
Libraries:

Wire.h – for working with the I2C bus.

LiquidCrystal_I2C.h – for controlling the 20x4 LCD display.

U8glib.h – for working with the graphic display (ST7920 128x64 module).

GyverEncoder.h – for processing signals from the encoder (including the button).

Hardware connection:

Encoder: pins CLK (3), DT (2) and SW (4) are used to read the rotation and press the button.

LCD display: connected via I2C (address 0x27).

Graphic display: controlled using U8glib with pins SCK (13), MOSI (11) and CS (10).

LIN interface: the hardware serial port is used (by default Serial1 with pins TX=18 and RX=19), but alternatives are provided (Serial2, Serial3 or SoftwareSerial).

2. Working with voltage parameters and sent data
Voltage arrays:

voltLIN1 and voltLIN2 contain “virtual” voltage values ​​as hex values.

setVOLT is an array of string values ​​(e.g. “12.1”, “12.2”, …, “15.0”), which are selected using the encoder (index valueLIN).

Generating LIN messages:

Two data arrays are defined: myDataE9 for messages with PID 0xE9 (4 bytes) and myDataEC for messages with PID 0xEC (5 bytes).

In the updateMyDataE9() and updateMyDataEC() functions, the data is updated depending on the selected voltage and the state of the button (short/long press).

3. Implementing the LIN protocol
Sending the Break signal:

The sendBreak() function disables the current serial, switches the TX pin to OUTPUT mode, holds it LOW for a time corresponding to 13 bits, then HIGH for about 1 bit, after which it initializes the serial port again and sends a sync byte (0x55).

Calculating the checksum:

The calculateChecksum() function sums up the data (starting from a certain index - depending on the mode: classic or extended) and returns the inverted value of the sum. This allows you to distinguish LIN 1.x from LIN 2.x (enhanced mode).

Sending LIN messages:

The sendLINMessage() function first sends Break, then PID.

The sendMessageE9() and sendMessageEC() functions form and send a complete frame for the corresponding PIDs, including data and checksum. This takes into account which protocol mode is used (LIN 1.x or LIN 2.x).

Receiving and processing LIN data:

The receiveLINData() function waits for a response from the device, reads bytes into a buffer, determines the PID, data and checksum, and then compares the received checksum with the calculated one (for classic and extended modes). The results (for example, the correctness of the protocol) are stored in global flags.

4. Searching and polling devices
Detecting devices (Search Mode):

In search mode (searchMode == true), the code iterates over three standard speeds (4800, 9600, 19200) and a list of known PIDs (PID_ARRAY array).

For each combination, a message is sent, and if the device responds (the number of bytes received is greater than 3), its data is stored in the array of devices structures.

If the device has already been found, its parameters (for example, speed) can be updated if a higher communication speed is detected.

Poll of found devices (Poll Mode):

After a successful search, the mode is switched (searchMode = false).

The code cyclically polls each found device, sending the corresponding PID and waiting for a response.

Depending on the PID:

If the PID is 0x92 - a message with PID 0xE9 is sent.

If the PID is 0xD6 - a message with PID 0xEC is sent.

If the device stops responding or the checksums do not match, the system switches back to the search mode.

5. Encoder processing
encoderTask() function:

Configures the encoder pins with pull-up (INPUT_PULLUP) and enables interrupts for the CLK and DT pins.

When the encoder is rotated, the valueLIN is increased or decreased, which is used to select the voltage.

When the value changes, the data is output to Serial and the value on the LCD is updated.

It also determines whether the encoder button is held down (buttonHeld flag), which affects the formation of the messages sent (for example, one of the data bytes changes).

6. Displaying information on displays
LCD display (LiquidCrystal_I2C):

Displays basic information: start message, received LIN data, selected voltage, and message sending status.

Graphic display (U8glib):

The updateDisplay() function combines the output of several blocks:

Header (displayLINHeader): displays the word "LIN:" with speed indicators (L, M, H) and protocol information (LIN 1.x or LIN 2.x).

Selected voltage (displayEncoderVoltage): displays the selected voltage from the setVOLT array.

Additional message (displayMessageLine3): information about the message sent (for example, data for PID 0xE9 or 0xEC).

List of detected devices (displayDevices): each line contains the PID of the device, data and checksum.

7. Setup() and loop() functions
setup():

Initializes the graphic display with image flipping (setRot180).

Starts USB Serial for debugging and the selected LIN port.

Initializes the LCD, prints the initial message

====================================================================================================
============================================================================================================================================================================================
============================================================================================================================================================================================
============================================================================================================================================================================================
============================================================================================================================================================================================

Этот Arduino код представляет собой достаточно комплексную реализацию для работы с шиной LIN:

Он умеет обнаруживать устройства, опрашивать их и отправлять управляющие команды с учетом различных режимов LIN (1.x и 2.x).

Реализована обработка контрольных сумм и синхронизации посредством сигнала Break.

Дополнительно предусмотрен интерактивный ввод с помощью энкодера, позволяющий изменять параметры (например, напряжение), а также наглядное отображение данных на двух типах дисплеев.

Таким образом, данный проект может служить основой для диагностики, тестирования или управления сетью LIN в автомобильных системах или других областях, где применяется данный протокол.



1. Используемые библиотеки и аппаратные модули
Библиотеки:

Wire.h – для работы с шиной I2C.

LiquidCrystal_I2C.h – для управления LCD-дисплеем 20x4.

U8glib.h – для работы с графическим дисплеем (модуль ST7920 128×64).

GyverEncoder.h – для обработки сигналов от энкодера (включая кнопку).

Аппаратное подключение:

Энкодер: пины CLK (3), DT (2) и SW (4) используются для считывания вращения и нажатия кнопки.

LCD-дисплей: подключается через I2C (адрес 0x27).

Графический дисплей: управляется с помощью U8glib с пинами SCK (13), MOSI (11) и CS (10).

LIN-интерфейс: используется аппаратный последовательный порт (по умолчанию Serial1 с пинами TX=18 и RX=19), но предусмотрены альтернативы (Serial2, Serial3 или SoftwareSerial).

2. Работа с параметрами напряжения и отправляемыми данными
Массивы напряжений:

voltLIN1 и voltLIN2 содержат «виртуальные» значения напряжения в виде hex-значений.

setVOLT – массив строковых значений (например, "12.1", "12.2", …, "15.0"), которые выбираются с помощью энкодера (индекс valueLIN).

Формирование сообщений LIN:

Определены два массива данных: myDataE9 для сообщений с PID 0xE9 (4 байта) и myDataEC для сообщений с PID 0xEC (5 байт).

В функции updateMyDataE9() и updateMyDataEC() производится обновление данных в зависимости от выбранного напряжения и состояния кнопки (короткое/долгое нажатие).

3. Реализация LIN-протокола
Отправка сигнала Break:

Функция sendBreak() отключает текущий сериал, переводит TX-пин в режим OUTPUT, держит его LOW на время, соответствующее 13 битам, затем HIGH примерно на 1 бит, после чего снова инициализирует последовательный порт и отправляет синхробайт (0x55).

Вычисление контрольной суммы:

Функция calculateChecksum() суммирует данные (начиная с определённого индекса – в зависимости от режима: классический или расширенный) и возвращает инвертированное значение суммы. Это позволяет отличать LIN 1.x от LIN 2.x (enhanced mode).

Отправка LIN-сообщений:

Функция sendLINMessage() сначала посылает Break, затем – PID.

Функции sendMessageE9() и sendMessageEC() формируют и отправляют полный кадр для соответствующих PID, включая данные и контрольную сумму. При этом учитывается, какой режим протокола используется (LIN 1.x или LIN 2.x).

Приём и обработка LIN-данных:

Функция receiveLINData() ждёт ответ от устройства, считывает байты в буфер, определяет PID, данные и контрольную сумму, а затем сравнивает полученную контрольную сумму с рассчитанной (для классического и расширенного режимов). Результаты (например, корректность протокола) сохраняются в глобальных флагах.

4. Поиск и опрос устройств
Обнаружение устройств (Search Mode):

В режиме поиска (searchMode == true) код перебирает три стандартные скорости (4800, 9600, 19200) и список известных PID (массив PID_ARRAY).

Для каждой комбинации отправляется сообщение, и если устройство отвечает (количество полученных байт больше 3), его данные сохраняются в массив структур devices.

Если устройство уже найдено, его параметры (например, скорость) могут обновляться, если обнаружена более высокая скорость связи.

Опрос найденных устройств (Poll Mode):

После успешного поиска режим переключается (searchMode = false).

Код циклически опрашивает каждое найденное устройство, посылая соответствующий PID и ожидая ответ.

В зависимости от PID:

Если PID равен 0x92 – отправляется сообщение с PID 0xE9.

Если PID равен 0xD6 – отправляется сообщение с PID 0xEC.

Если устройство перестаёт отвечать или контрольные суммы не совпадают, система переходит обратно в режим поиска.

5. Обработка энкодера
Функция encoderTask():

Настраивает пины энкодера с подтяжкой (INPUT_PULLUP) и подключает прерывания для пинов CLK и DT.

При вращении энкодера увеличивается или уменьшается значение valueLIN, которое используется для выбора напряжения.

При изменении значения выводятся данные в Serial и обновляется значение на LCD.

Также определяется, удерживается ли кнопка энкодера (флаг buttonHeld), что влияет на формирование отправляемых сообщений (например, изменяется один из байтов данных).

6. Вывод информации на дисплеи
LCD-дисплей (LiquidCrystal_I2C):

Отображает базовую информацию: стартовое сообщение, полученные данные LIN, выбранное напряжение и статус отправки сообщений.

Графический дисплей (U8glib):

Функция updateDisplay() объединяет вывод нескольких блоков:

Заголовок (displayLINHeader): выводится слово "LIN:" с индикаторами скорости (L, M, H) и информацией о протоколе (LIN 1.x или LIN 2.x).

Выбранное напряжение (displayEncoderVoltage): выводится выбранное напряжение из массива setVOLT.

Дополнительное сообщение (displayMessageLine3): информация о посланном сообщении (например, данные для PID 0xE9 или 0xEC).

Список обнаруженных устройств (displayDevices): каждая строка содержит PID устройства, данные и контрольную сумму.

7. Функции setup() и loop()
setup():

Инициализирует графический дисплей с переворотом изображения (setRot180).

Запускает USB Serial для отладки и выбранный LIN-порт.

Инициализирует LCD, выводит начальное сообщение ("Start").

Вызывает функцию detectLINBaudRate() для попытки автоопределения активной скорости LIN.

loop():

Шаг 1: Вызывает encoderTask() для обработки ввода с энкодера.

Шаг 2 (Search Mode):

Если устройства ещё не найдены, перебираются все комбинации скоростей и PID для поиска активных устройств.

При получении ответа устройство добавляется в список.

Если найден хотя бы один, система переходит в режим опроса.

Шаг 3 (Poll Mode):

Каждое найденное устройство опрашивается, отправляются дополнительные команды (в зависимости от PID – отправка PID 0xE9 или 0xEC).

Если устройство перестаёт отвечать или возникает ошибка, система возвращается в режим поиска.

После каждого цикла опроса обновляется графический дисплей для визуального отображения текущего состояния сети LIN.


