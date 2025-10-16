# 🌡️ ESP32-C3 SuperMini — BLE + DS18B20 + Web Bluetooth Dashboard

![Repo Type](https://img.shields.io/badge/repo-private-blue)
![Platform](https://img.shields.io/badge/platform-ESP32--C3-orange)
![Status](https://img.shields.io/badge/status-online-success)
![License](https://img.shields.io/badge/license-MIT-green)

Проект демонстрирует использование **ESP32-C3** в роли BLE-периферийного устройства,  
которое передаёт данные с датчика температуры **DS18B20** через BLE-уведомления (Notify).  
Для отображения данных используется современный **веб-интерфейс (Web Bluetooth)**,  
работающий прямо в браузере (Chrome / Edge / Opera).

---

## 🔧 Аппаратная часть

| Компонент | Подключение | Примечание |
|------------|--------------|-------------|
| **ESP32-C3 SuperMini** | — | Микроконтроллер с BLE |
| **DS18B20** | DQ → GPIO10 | Обязательно внешний pull-up 4.7 kΩ к 3.3 V |
| | Vdd → 3.3 V, GND → GND | |
| **Кнопка** | К GND → GPIO9 | Встроенный pull-up включён |
| **LED (встроенный)** | GPIO8 | Активный низкий уровень (0 = горит) |

---

## 🧠 Программная часть

- Среда: **ESP-IDF v5.5.1**  
- Стек BLE: **NimBLE (встроенный в ESP-IDF)**  
- Основные возможности:
  - Передача `temp=XX.X` через BLE Notify каждую секунду  
  - При нажатии кнопки → Notify `btn=1`  
  - Управление LED через BLE Write (`'1'`/`'0'`)  
  - Поддержка Web Bluetooth — управление и график температуры прямо в браузере  

---

## 💻 Веб-интерфейс

HTML-страница `index.html` (см. папку `web/`):

- Подключается к устройству `ESP32C3-BLE`  
- Отображает поток уведомлений `temp=XX.X` на графике  
- Показывает события (`btn=1`, `alarm=1`, `limit=...`) как вертикальные цветные маркеры  
- Считает усреднённые значения за каждые 10 минут и часовое среднее  

### Совместимые браузеры
| Поддерживается | Примечание |
|----------------|-------------|
| 🟢 **Google Chrome** | на Windows, Linux, Android |
| 🟢 **Microsoft Edge** | на Windows |
| 🟡 **Opera / Chromium-браузеры** | поддержка BLE зависит от версии |

> ⚠️ Web Bluetooth работает **только через HTTPS или `http://localhost`**.

---

## 📦 Компиляция и прошивка

1. Установить **ESP-IDF v5.5.1**  
2. Открыть проект в **VS Code / Espressif-IDE**  
3. Настроить порт (например, `COM31`)  
4. Сборка и прошивка:
   ```bash
   idf.py build flash monitor
   ```

---

## 📈 Пример работы

- В логе ESP32:
  ```
  I (1234) BLE_MIN: ADV STARTED
  I (2345) BLE_MIN: CONNECTED
  I (3456) BLE_MIN: NOTIFY: temp=23.5
  I (4567) BLE_MIN: NOTIFY: btn=1
  ```

- В браузере — реальный график температуры с отметками событий:

  ![Web Bluetooth graph example](docs/example_graph.png)

---

## 📘 Структура проекта

```
ESP32-C3_BLE_Temp/
 ├── main/
 │   ├── main.c              # основной код ESP-IDF
 │   └── CMakeLists.txt
 ├── web/
 │   └── index.html          # Web Bluetooth интерфейс
 ├── sdkconfig
 ├── README.md
 └── .gitignore
```

---

## 📄 Лицензия
Проект распространяется под лицензией MIT.  
Можно использовать и модифицировать свободно с указанием автора.

---

© 2025 — Jevgenijs Ricko  
[GitHub Profile](https://github.com/JevgenijsRicko)
