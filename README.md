# ESP32 RFID Reader (RDM6300) UART Decoder

Проект на ESP-IDF v5.4 для считывания RFID меток с помощью модуля **RDM6300** по UART.  
Тестировалось на **ESP32-P4**, но может быть использовано и с другими платами на базе ESP32.

## 📦 Возможности

- Работа с UART-модулем RFID RDM6300
- Поддержка стандартных RFID меток 125 кГц (включая синие брелки и белые карты)
- Автоматическое преобразование HEX в десятичный ID
- Обработка и логирование "сырых" и расшифрованных данных
- Проверка корректности формата пакета

---

## 🔧 Аппаратные требования

- **ESP32-P4 DevKit** (или любая плата ESP32)
- **RFID модуль RDM6300** (125 кГц)
- RFID метки (синие или белые, совместимые с RDM6300)

---

## 📡 Схема подключения

| ESP32 GPIO | Назначение | RDM6300 Pin |
|------------|------------|-------------|
| GPIO21     | RX         | TX          |
| GPIO20     | TX         | RX *(необязательно)* |
| GND        | Земля      | GND         |
| 3.3V       | Питание    | VCC (3.3V)  |

**Важно:** RDM6300 работает от 3.3V! Не подключайте к 5V на ESP32.

### 📷 Схема подключения:

```
+------------------+        +------------------+
|      ESP32       |        |     RDM6300      |
|                  |        |                  |
|   GPIO21 (RX) <---------- TX                |
|   GPIO20 (TX) ----------> RX   (не обяз.)   |
|        GND ----------------> GND             |
|       3.3V ----------------> VCC             |
+------------------+        +------------------+
```

---

## 🚀 Быстрый старт

### 1. Установите ESP-IDF v5.4

Инструкция: https://docs.espressif.com/projects/esp-idf/en/v5.4.0/esp32/get-started/

### 2. Клонируйте репозиторий

```bash
git clone https://github.com/your_username/esp32-rdm6300-reader.git
cd esp32-rdm6300-reader
```

### 3. Соберите и загрузите проект

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

> Замените `/dev/ttyUSB0` на ваш COM-порт.

---

## 📋 Пример вывода

При поднесении синей метки (с надписью `0000844238`), вы увидите:

```
I (309) UART: UART initialized successfully
I (309) RFID: UART read task started
I (309) main_task: Returned from app_main()
I (195149) RFID: Received 14 bytes:
I (195149) RFID: 0x02 
I (195149) RFID: 0x30 
I (195149) RFID: 0x36 
I (195149) RFID: 0x30 
I (195149) RFID: 0x30 
I (195149) RFID: 0x30 
I (195149) RFID: 0x43 
I (195149) RFID: 0x45 
I (195149) RFID: 0x31 
I (195159) RFID: 0x43 
I (195159) RFID: 0x45 
I (195159) RFID: 0x32 
I (195159) RFID: 0x35 
I (195159) RFID: 0x03 
I (195169) RFID: Raw card ID: 06000CE1CE
I (195169) RFID: Hex value: 000CE1CE
I (195169) RFID: Decrypted ID: 0000844238
```

---

## 🧠 Как это работает

Модуль RDM6300 отправляет 14 байт:

- `0x02` — старт байт  
- `10 символов` — ASCII коды HEX номера карты  
- `0x03` — стоп байт

Из строки берётся 8 HEX-символов, начиная с 3-го символа, преобразуются в `unsigned long`, и форматируются как десятичный ID.

---

## 🧪 Протестировано с

- **Синие RFID брелки** (карты для Arduino)
- **Белые RFID карты**
- ESP32-P4 + ESP-IDF v5.4
- RDM6300 (125 kHz)

---

## 🛠️ Настройки UART

```c
#define RFID_UART_PORT UART_NUM_1
#define RFID_RX_PIN 21
#define RFID_TX_PIN 20
#define BUF_SIZE 1024
```

---

## 📁 Структура проекта

```
esp32-rdm6300-reader/
├── main/
│   └── main.c          # Основной код
├── CMakeLists.txt
├── README.md
└── sdkconfig           # Генерируется автоматически
```

---

## 📃 Лицензия

Проект распространяется под MIT License. Свободно используйте, модифицируйте и распространяйте.

---

## 🤝 Благодарности

- [Espressif](https://github.com/espressif) за ESP-IDF  
- [RDM6300 datasheet](https://www.elecrow.com/download/RDM6300%20UART%20RFID%20Module%20Datasheet.pdf)
