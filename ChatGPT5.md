2) Сохранить текущий проект и на его основе создать новый “ESP32-C3_BLE_Midi”

Надёжный путь — просто клонировать папку проекта и переименовать проект в CMake.

Шаги (Windows, PowerShell):

Закрой проект в VS Code.

Скопируй папку (пример):

Copy-Item "C:\Documents\VS_Code_Work\ESP-IDF\ble_minimal_c3" `
          "C:\Documents\VS_Code_Work\ESP-IDF\ESP32-C3_BLE_Midi" -Recurse


Открой новую папку ESP32-C3_BLE_Midi в VS Code.

В корневом CMakeLists.txt поменяй:

project(ESP32-C3_BLE_Midi)


(Опционально) В main.c поменяй имя BLE-устройства:

#define DEV_NAME "ESP32C3-BLE-MIDI"


Убедись, что в .vscode/settings.json указан твой порт:

{ "idf.port": "COM31" }


В палитре выполни:

ESP-IDF: Set Espressif Device Target → esp32c3

ESP-IDF: Build Project → Flash → Monitor

Бонус: Zip-бэкап текущего рабочего проекта

Открой текущий проект → Ctrl+Shift+P → ESP-IDF: Open ESP-IDF Terminal → в нём:

Compress-Archive -Path .\* -DestinationPath ..\ble_minimal_c3_backup.zip -Force


Получишь архив рядом с проектом.

На будущее для BLE MIDI: стандартные UUID сервиса/характеристики у MIDI BLE уже определены; когда будешь готов — подкину минимальный GATT под MIDI (service 03B80E5A-…C700, characteristic 7772E5DB-…6BF3) и пример упаковки сообщений.

Если хочешь, сделаю всё автоматом: сгенерирую готовый шаблон ESP32-C3_BLE_Midi (структура, CMake, main.c с правильными UUID, логом и фиксированным COM31).