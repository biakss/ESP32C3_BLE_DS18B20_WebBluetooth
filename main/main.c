#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us()

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG              "BLE_MIN"

// ---- ПЛАТА: ESP32-C3 SuperMini ----
#define DEV_NAME         "ESP32C3-BLE"
#define LED_GPIO         8          // встроенный LED
#define LED_ACTIVE_LOW   1          // активный НИЗКИЙ (0=горит, 1=гаснет)
#define BUTTON_GPIO      9          // кнопка к GND, внутренний pull-up
#define DS18B20_GPIO     10         // 1-Wire DQ (обязательно внешний 4.7k к 3.3В)

// ---- UUID ----
#define MY_SVC_UUID16    0xFFF0
#define MY_CHR_UUID16    0xFFF1     // RW + Notify

static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_chr_val_handle;
static uint8_t  g_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static char     g_value[64] = "Hello from ESP32-C3!"; // READ/WRITE echo

static ble_uuid16_t adv_svc_uuid = BLE_UUID16_INIT(MY_SVC_UUID16);

// ========= LED helpers =========
static inline void led_write_state(bool on) {
    gpio_set_level(LED_GPIO, LED_ACTIVE_LOW ? !on : on);
}

// ========= 1-Wire (DS18B20) битбанг =========
// Линия отпускается во вход с pull-up; тянуть вниз — выводом low.
static inline void ow_drive_low(void) {
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DS18B20_GPIO, 0);
}
static inline void ow_release(void) {
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_INPUT);
}
static inline int ow_read_line(void) {
    return gpio_get_level(DS18B20_GPIO);
}

// reset/presence
static bool ow_reset(void) {
    ow_drive_low();
    esp_rom_delay_us(480);
    ow_release();
    esp_rom_delay_us(70);
    bool present = (ow_read_line() == 0); // 0 = presence
    esp_rom_delay_us(410);
    return present;
}

static void ow_write_bit(int bit) {
    ow_drive_low();
    if (bit) {
        esp_rom_delay_us(6);
        ow_release();
        esp_rom_delay_us(64);
    } else {
        esp_rom_delay_us(60);
        ow_release();
        esp_rom_delay_us(10);
    }
}

static int ow_read_bit(void) {
    int r;
    ow_drive_low();
    esp_rom_delay_us(6);
    ow_release();
    esp_rom_delay_us(9);
    r = ow_read_line();
    esp_rom_delay_us(55);
    return r;
}

static void ow_write_byte(uint8_t b) {
    for (int i=0;i<8;i++) ow_write_bit((b>>i)&1);
}
static uint8_t ow_read_byte(void) {
    uint8_t v=0;
    for (int i=0;i<8;i++) v |= (ow_read_bit()<<i);
    return v;
}

// Dallas CRC8
static uint8_t ds_crc8(const uint8_t *data, int len) {
    uint8_t crc=0;
    for (int i=0;i<len;i++) {
        uint8_t inbyte = data[i];
        for (int j=0;j<8;j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

// Чтение температуры, °C. Возвращает true/false, значение в *out_c
static bool ds18b20_read_c(float *out_c) {
    if (!ow_reset()) return false;
    ow_write_byte(0xCC); // SKIP ROM
    ow_write_byte(0x44); // CONVERT T
    // Конверсия до 12 бит ~ 750 мс; ставим 750-800 мс
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (!ow_reset()) return false;
    ow_write_byte(0xCC); // SKIP ROM
    ow_write_byte(0xBE); // READ SCRATCHPAD

    uint8_t scratch[9] = {0};
    for (int i=0;i<9;i++) scratch[i] = ow_read_byte();

    if (ds_crc8(scratch, 9) != 0) {
        ESP_LOGW(TAG, "DS18B20 CRC error");
        return false;
    }

    int16_t raw = (scratch[1] << 8) | scratch[0]; // little-endian
    // 12-bit resolution by default; LSB = 0.0625 °C
    *out_c = raw / 16.0f;
    return true;
}

// ========= GATT =========
static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        ESP_LOGI(TAG, "GATT READ");
        return os_mbuf_append(ctxt->om, g_value, strlen(g_value));

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        int len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= (int)sizeof(g_value)) len = sizeof(g_value) - 1;
        os_mbuf_copydata(ctxt->om, 0, len, g_value);
        g_value[len] = '\0';
        ESP_LOGI(TAG, "GATT WRITE: '%s'", g_value);

        if (g_value[0] == '1') { led_write_state(true);  ESP_LOGI(TAG, "LED ON");  }
        else if (g_value[0] == '0') { led_write_state(false); ESP_LOGI(TAG, "LED OFF"); }

        return 0;
    }
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(MY_SVC_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(MY_CHR_UUID16),
                .access_cb = chr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_chr_val_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

// ========= GAP / Advertising =========
static int gap_event_cb(struct ble_gap_event *ev, void *arg);

static void start_advertising(void) {
    struct ble_hs_adv_fields f = {0};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.name = (uint8_t*)DEV_NAME;
    f.name_len = strlen(DEV_NAME);
    f.name_is_complete = 1;
    f.uuids16 = &adv_svc_uuid;
    f.num_uuids16 = 1;
    f.uuids16_is_complete = 1;
    ESP_ERROR_CHECK(ble_gap_adv_set_fields(&f));

    struct ble_gap_adv_params advp = {0};
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER, &advp, gap_event_cb, NULL);
    if (rc == 0) ESP_LOGI(TAG, "ADV STARTED");
    else         ESP_LOGE(TAG, "adv_start err=%d", rc);
}

static int gap_event_cb(struct ble_gap_event *ev, void *arg) {
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            g_conn_handle = ev->connect.conn_handle;
            ESP_LOGI(TAG, "CONNECTED (handle=%d)", g_conn_handle);
        } else {
            ESP_LOGW(TAG, "CONNECT FAIL -> restart adv");
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "DISCONNECTED (reason=%d)", ev->disconnect.reason);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        start_advertising();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "ADV COMPLETE -> restart");
        start_advertising();
        break;

    default:
        break;
    }
    return 0;
}

// ========= NimBLE sync =========
static void on_sync(void) {
    ESP_ERROR_CHECK(ble_hs_id_infer_auto(0, &g_own_addr_type));
    ble_svc_gap_device_name_set(DEV_NAME);

    uint8_t addr[6];
    ble_hs_id_copy_addr(g_own_addr_type, addr, NULL);
    ESP_LOGI(TAG, "Addr (%s) %02X:%02X:%02X:%02X:%02X:%02X",
        g_own_addr_type == BLE_OWN_ADDR_PUBLIC ? "public" : "random",
        addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    start_advertising();
}

static void host_task(void *param) {
    ESP_LOGI(TAG, "HOST TASK START");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ========= Кнопка: ISR -> очередь -> Notify =========
typedef struct { uint32_t tick; } btn_event_t;
static QueueHandle_t g_btn_queue;

static void IRAM_ATTR button_isr(void *arg) {
    btn_event_t e = { .tick = xTaskGetTickCountFromISR() };
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(g_btn_queue, &e, &hpw);
    if (hpw) portYIELD_FROM_ISR();
}

static void button_task(void *_) {
    ESP_LOGI(TAG, "BUTTON TASK START");
    btn_event_t e;
    uint32_t last_tick = 0;
    const uint32_t debounce = pdMS_TO_TICKS(120);

    for (;;) {
        if (xQueueReceive(g_btn_queue, &e, portMAX_DELAY)) {
            if (e.tick - last_tick < debounce) continue;
            last_tick = e.tick;

            if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                const char *msg = "btn=1";
                struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
                ble_gatts_notify_custom(g_conn_handle, g_chr_val_handle, om);
                ESP_LOGI(TAG, "NOTIFY: %s", msg);
            }
            led_write_state(true); vTaskDelay(pdMS_TO_TICKS(80)); led_write_state(false);
        }
    }
}

// ========= Температура DS18B20 -> Notify =========
static void temp_notify_task(void *_) {
    ESP_LOGI(TAG, "TEMP TASK START");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        float tC;
        if (ds18b20_read_c(&tC)) {
            if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                char buf[32];
                int n = snprintf(buf, sizeof(buf), "temp=%.1f", tC);
                struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, n);
                ble_gatts_notify_custom(g_conn_handle, g_chr_val_handle, om);
                ESP_LOGI(TAG, "NOTIFY: %s", buf);
            }
        } else {
            ESP_LOGW(TAG, "DS18B20 read failed");
        }
    }
}

// ========= app_main =========
void app_main(void) {
    ESP_LOGI(TAG, "INIT START");

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // LED
    gpio_config_t io_led = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_led);
    led_write_state(false);
    ESP_LOGI(TAG, "LED READY (GPIO%d, active_low=%d)", LED_GPIO, LED_ACTIVE_LOW);

    // BTN
    gpio_config_t io_btn = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1, .pull_down_en = 0, .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_btn);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr, NULL);
    g_btn_queue = xQueueCreate(8, sizeof(btn_event_t));
    ESP_LOGI(TAG, "BUTTON READY (GPIO%d, pull-up, negedge)", BUTTON_GPIO);

    // DS18B20 линия — вход с pull-up (внешний 4.7k обязателен)
    gpio_config_t io_1w = {
        .pin_bit_mask = 1ULL << DS18B20_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_1w);
    ESP_LOGI(TAG, "DS18B20 READY (GPIO%d, ext 4.7k pull-up required)", DS18B20_GPIO);

    // NimBLE
    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(host_task);
    xTaskCreatePinnedToCore(button_task,      "btn_task",   4096, NULL, 5, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(temp_notify_task, "temp_task",  4096, NULL, 5, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "INIT DONE");
}
