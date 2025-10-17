# 🌡️ ESP32-C3 SuperMini — BLE + DS18B20 + Web Bluetooth Dashboard

![Repo Type](https://img.shields.io/badge/repo-private-blue)
![Platform](https://img.shields.io/badge/platform-ESP32--C3-orange)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5.1-blue)
![BLE Stack](https://img.shields.io/badge/BLE-NimBLE-success)
![Sensor](https://img.shields.io/badge/Sensor-DS18B20-lightgrey)
![Web Bluetooth](https://img.shields.io/badge/Web%20Bluetooth-Ready-blueviolet)
![Status](https://img.shields.io/badge/status-online-success)
![Build](https://img.shields.io/badge/build-passing-brightgreen)
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

## 🤝 Contributing / Git rules

Чтобы история коммитов оставалась чистой и авторы отображались корректно,  
перед первым коммитом настрой Git:

```bash
git config --global user.name "Jevgenijs Ricko"
git config --global user.email "95184766+biakss@users.noreply.github.com"
```

> ⚠️ Рекомендуется использовать именно GitHub noreply-адрес, чтобы скрыть личный e-mail.

### Правила для участников (если появятся)
- Каждый коммит должен быть осмысленным и содержать понятное сообщение.  
  Примеры:
  - ✅ `Добавлен обработчик кнопки GPIO9`
  - ✅ `Исправлена инициализация DS18B20 при старте`
- Автоматические коммиты от VS Code имеют префикс  
  `"Auto commit from VS Code"`, что тоже допустимо.
- Перед push желательно убедиться, что проект успешно собирается (`idf.py build`).
- Если нужно изменить автора задним числом — можно использовать встроенный скрипт  
  `rewrite-author.ps1`, который обновит имя и email во всей истории.

---

## 📄 Лицензия
Проект распространяется под лицензией MIT.  
Можно использовать и модифицировать свободно с указанием автора.

---

© 2025 — Jevgenijs Ricko  
[GitHub Profile](https://github.com/JevgenijsRicko)
