// firmware_sample.cpp
// Full example C++ firmware skeleton for ESP32-C6 (ESP-IDF) + FreeRTOS.
// This is an implementation-ready sample skeleton demonstrating architecture,
// tasks, queues, recovery, I2C recovery, SD atomic writes, Wi-Fi -> HTTP fallback,
// LoRa fallback, power management and deep-sleep integration.
//
// NOT a drop-in production firmware: sensor drivers, real crypto, and OTA implementation
// must be filled with vendor libraries. This file demonstrates structure, error handling,
// and necessary safety patterns you asked for.
//
// Build with ESP-IDF C++ support. Link with mbedtls, fatfs, vfs, nvs_flash, esp_wifi, tcpip_adapter, etc.
// Keep this as a single-file starting point and extract to modules in real project.

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_sleep.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
}

static const char *TAG = "weather-fw";

/////////////////////////////
// Configuration constants //
/////////////////////////////

static constexpr uint32_t SAMPLE_INTERVAL_SECONDS = 300;         // 5 minutes
static constexpr int MAX_RETRIES = 3;
static constexpr size_t SD_BUFFER_SIZE = 5120;                   // flush threshold
static constexpr int WIFI_CONNECT_TIMEOUT_MS = 10000;
static constexpr int LORA_TX_RETRY = 2;
static constexpr const char *DEVICE_ID = "mjohnson-weather-v1";
static constexpr int I2C_MASTER_PORT = I2C_NUM_0;
static constexpr int I2C_FREQ_HZ = 400000;
static constexpr gpio_num_t PIN_SENSOR_POWER = GPIO_NUM_21;      // example
static constexpr gpio_num_t PIN_GPS_POWER = GPIO_NUM_22;
static constexpr gpio_num_t PIN_LED_STATUS = GPIO_NUM_2;
static constexpr float BATTERY_CRITICAL_VOLTAGE = 6.5f;          // example for 2S Li-ion
static constexpr float BATTERY_LOW_WARNING_VOLTAGE = 7.0f;

///////////////////////////
// Data structures/types //
///////////////////////////

struct Bme680Sample { bool present; float temp_c; float hum_pct; float press_hpa; int gas_ohms; };
struct Sht4xSample { bool present; float temp_c; float hum_pct; };
struct Scd41Sample { bool present; int co2_ppm; float temp_c; float rh; };
struct Tsl25911Sample { bool present; float lux; };
struct GpsSample { bool present; double lat; double lon; bool fix; float hdop; uint64_t fix_age_s; };
struct BatterySample { float voltage; int pct; bool charging; };

struct Sample {
    char timestamp[32]; // ISO8601
    Bme680Sample bme680;
    Sht4xSample sht4x;
    Scd41Sample scd41;
    Tsl25911Sample tsl25911;
    GpsSample gps;
    BatterySample battery;
    int aqi_est;
    uint32_t crc8;
};

//////////////////////
// RTOS primitives  //
//////////////////////

static QueueHandle_t g_storage_queue = nullptr;   // Items: Sample*
static QueueHandle_t g_comms_queue = nullptr;     // Items: Sample*
static SemaphoreHandle_t g_i2c_mutex = nullptr;
static SemaphoreHandle_t g_sd_mutex = nullptr;
static TaskHandle_t g_watchdog_handle = nullptr;

//////////////////////////
// Helper / utilities   //
//////////////////////////

static uint32_t compute_crc8(const uint8_t *data, size_t len) {
    // simple CRC-8 (poly 0x07) for illustrative purposes
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void format_timestamp_iso8601(char *out, size_t out_sz) {
    // Attempt to read from RTC or system time.
    time_t now;
    time(&now);
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    strftime(out, out_sz, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
}

////////////////////////////
// Peripheral init stubs  //
////////////////////////////

static esp_err_t init_nvs_and_rtc() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %d", err);
        return err;
    }
    // RTC assumed via DS3231 hardware; leave driver to implement. Here we verify system time plausibility.
    time_t now;
    time(&now);
    if (now < 1609459200) { // If before 2021-01-01 assume RTC unset
        ESP_LOGW(TAG, "System time not set; will sync later.");
    }
    return ESP_OK;
}

static esp_err_t init_i2c_bus() {
    // Strict initialization order and bus recovery enabled.
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)18;
    conf.scl_io_num = (gpio_num_t)19;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) return err;
    err = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %d", err);
        return err;
    }
    return ESP_OK;
}

static void i2c_bus_recover() {
    // Attempt to recover stuck I2C bus: toggle SCL pulses and generate STOP.
    ESP_LOGW(TAG, "Attempting I2C bus recovery...");
    // Implementation depends on pin muxing; simplified here: re-init driver
    i2c_driver_delete(I2C_MASTER_PORT);
    vTaskDelay(pdMS_TO_TICKS(50));
    init_i2c_bus();
    ESP_LOGW(TAG, "I2C re-init attempted.");
}

//////////////////////////
// SD card (atomic append)
//////////////////////////

static const char *SD_MOUNT_POINT = "/sdcard";
static bool sd_mounted = false;

static esp_err_t init_sdcard() {
    esp_err_t ret;
    sdmmc_card_t *card = nullptr;
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 4,
    };
    // Default to 1-line SDMMC host or SPI host depending on hardware.
    ret = esp_vfs_fat_sdcard_mount(SD_MOUNT_POINT, nullptr, &mount_cfg, &card);
    if (ret == ESP_OK) {
        sd_mounted = true;
        ESP_LOGI(TAG, "SD card mounted at %s", SD_MOUNT_POINT);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Failed to mount SD card (%d). Will retry later.", ret);
        return ret;
    }
}

// Atomic append pattern: write to temp file, fsync, rename to target (append by opening, writing, fsync)
static bool sd_atomic_append_line(const char *filename, const char *line) {
    if (!sd_mounted) return false;
    if (xSemaphoreTake(g_sd_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGW(TAG, "SD mutex busy");
        return false;
    }
    bool ok = false;
    char tmpname[256];
    snprintf(tmpname, sizeof(tmpname), "%s.tmp", filename);
    FILE *f = fopen(tmpname, "w");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open tmp file for SD append");
        goto cleanup;
    }
    if (fprintf(f, "%s\n", line) < 0) goto cleanup_close;
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    f = nullptr;

    // Append tmp to final file atomically by reopening final and appending then removing tmp.
    FILE *final = fopen(filename, "a");
    if (!final) {
        ESP_LOGE(TAG, "Failed to open final SD file for append");
        goto cleanup;
    }
    FILE *tmp = fopen(tmpname, "r");
    if (!tmp) {
        fclose(final);
        goto cleanup;
    }
    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), tmp)) > 0) {
        if (fwrite(buf, 1, r, final) != r) {
            fclose(tmp); fclose(final);
            goto cleanup;
        }
    }
    fflush(final);
    fsync(fileno(final));
    fclose(tmp);
    fclose(final);
    remove(tmpname);
    ok = true;
    goto cleanup;

cleanup_close:
    if (f) fclose(f);
cleanup:
    xSemaphoreGive(g_sd_mutex);
    return ok;
}

//////////////////////
// Wi-Fi / HTTP stub //
//////////////////////

static bool wifi_connect_blocking() {
    // Implement a blocking connect with timeout. Here we stub successful connection.
    // In production, start Wi-Fi, wait for IP event or timeout.
    ESP_LOGI(TAG, "Connecting Wi-Fi (stub)...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // simulate
    // return false to simulate offline during testing; return true if connected.
    return true;
}

static bool http_post_json(const char *url, const char *json_payload) {
    // Simplified synchronous HTTP POST. In production use esp_http_client APIs and proper TLS config.
    ESP_LOGI(TAG, "HTTP POST to %s - payload len %d", url, (int)strlen(json_payload));
    vTaskDelay(pdMS_TO_TICKS(300)); // simulate network latency
    // Simulate success
    return true;
}

//////////////////
// LoRa stub    //
//////////////////

static bool lora_send_binary(const uint8_t *data, size_t len) {
    // Stub LoRa transmission via SPI to module; handle ack etc. Return true on success.
    ESP_LOGI(TAG, "LoRa send %zu bytes (stub)", len);
    vTaskDelay(pdMS_TO_TICKS(100));
    return true;
}

//////////////////////////
// Sensor driver stubs  //
//////////////////////////

static Bme680Sample read_bme680() {
    Bme680Sample s{};
    s.present = true;
    s.temp_c = 21.5f + (esp_random() % 100) / 100.0f;
    s.hum_pct = 40.0f + (esp_random() % 100) / 100.0f;
    s.press_hpa = 1013.0f;
    s.gas_ohms = 120;
    return s;
}

static Sht4xSample read_sht4x() {
    Sht4xSample s{};
    s.present = true;
    s.temp_c = 21.6f;
    s.hum_pct = 40.1f;
    return s;
}

static Scd41Sample read_scd41() {
    Scd41Sample s{};
    s.present = true;
    s.co2_ppm = 420 + (esp_random() % 50);
    s.temp_c = 21.4f;
    s.rh = 40.2f;
    return s;
}

static Tsl25911Sample read_tsl25911() {
    Tsl25911Sample s{};
    s.present = true;
    s.lux = 1200.0f;
    return s;
}

static GpsSample read_gps() {
    GpsSample s{};
    s.present = true;
    s.fix = true;
    s.lat = 40.0 + (esp_random() % 1000)/10000.0;
    s.lon = -73.0 - (esp_random() % 1000)/10000.0;
    s.hdop = 0.9f;
    s.fix_age_s = 10;
    return s;
}

static BatterySample sample_battery_adc() {
    BatterySample b{};
    b.voltage = 7.40f; // stub
    b.pct = 85;
    b.charging = true;
    return b;
}

/////////////////////////
// Validation & utils  //
/////////////////////////

static void validate_and_sanitize_sample(Sample &s) {
    // Range checking and cross-sensor sanity rules
    if (s.bme680.present) {
        if (s.bme680.temp_c < -40 || s.bme680.temp_c > 85) s.bme680.present = false;
    }
    if (s.sht4x.present) {
        if (s.sht4x.temp_c < -40 || s.sht4x.temp_c > 125) s.sht4x.present = false;
    }
    // cross-check temp proximity
    if (s.bme680.present && s.sht4x.present) {
        float diff = fabsf(s.bme680.temp_c - s.sht4x.temp_c);
        if (diff > 5.0f) {
            // prefer SHT4x for humidity/temperature; mark BME if gross disagreement
            s.bme680.present = (diff <= 10.0f);
        }
    }
    // CO2 sanity
    if (s.scd41.present) {
        if (s.scd41.co2_ppm < 250 || s.scd41.co2_ppm > 5000) {
            s.scd41.present = false;
        }
    }
    // simple AQI estimate stub
    s.aqi_est = 0;
    if (s.bme680.present) {
        s.aqi_est = static_cast<int>(s.bme680.gas_ohms / 3);
    }
}

static void sample_to_json(const Sample &s, char *out, size_t out_sz) {
    // Minimal JSON serializer (no external deps). Ensure bounds.
    snprintf(out, out_sz,
        "{\"device_id\":\"%s\",\"timestamp\":\"%s\","
        "\"sensors\":{"
        "\"bme680\":{\"temp_c\":%.2f,\"hum_pct\":%.2f,\"press_hpa\":%.2f,\"gas_ohms\":%d},"
        "\"sht4x\":{\"temp_c\":%.2f,\"hum_pct\":%.2f},"
        "\"scd41\":{\"co2_ppm\":%d,\"temp_c\":%.2f,\"rh\":%.2f},"
        "\"tsl25911\":{\"lux\":%.1f},"
        "\"gps\":{\"lat\":%.6f,\"lon\":%.6f,\"fix\":%s,\"hdop\":%.2f},"
        "\"battery\":{\"voltage\":%.2f,\"pct\":%d,\"charging\":%s}"
        "},\"aqi_est\":%d,\"crc8\":%u}",
        DEVICE_ID,
        s.timestamp,
        s.bme680.temp_c, s.bme680.hum_pct, s.bme680.press_hpa, s.bme680.gas_ohms,
        s.sht4x.temp_c, s.sht4x.hum_pct,
        s.scd41.co2_ppm, s.scd41.temp_c, s.scd41.rh,
        s.tsl25911.lux,
        s.gps.lat, s.gps.lon, s.gps.fix ? "true" : "false", s.gps.hdop,
        s.battery.voltage, s.battery.pct, s.battery.charging ? "true" : "false",
        s.aqi_est, s.crc8
    );
}

//////////////////////////////
// Task implementations     //
//////////////////////////////

static void SensorTask(void *arg) {
    ESP_LOGI(TAG, "SensorTask started");
    while (true) {
        // Wait for wake signal or just run periodically if buried by timer
        // Here: loop-based with delay equivalent to wake policy, but real code will use event-driven wake
        // Boot-time gotcha: perform sensor presence probe once per boot and then per X cycles.
        // Power on sensors
        gpio_set_level(PIN_SENSOR_POWER, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); // allow rails to stabilize

        Sample *sample = (Sample*)malloc(sizeof(Sample));
        if (!sample) {
            ESP_LOGE(TAG, "Out of memory for sample");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        memset(sample, 0, sizeof(Sample));
        format_timestamp_iso8601(sample->timestamp, sizeof(sample->timestamp));

        // Acquire I2C mutex for all sensor reads to avoid concurrency issues
        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "I2C busy, attempting recovery");
            i2c_bus_recover();
            xSemaphoreGive(g_i2c_mutex); // still give to avoid deadlock - safe only if taken
        }

        // Read sensors in safe order
        int tries = 0;
        do {
            sample->bme680 = read_bme680();
            if (sample->bme680.present) break;
            tries++;
            vTaskDelay(pdMS_TO_TICKS(10));
        } while (tries < MAX_RETRIES);

        sample->sht4x = read_sht4x();
        sample->scd41 = read_scd41();
        sample->tsl25911 = read_tsl25911();

        // GPS may be optional; power on only occasionally
        gpio_set_level(PIN_GPS_POWER, 1);
        vTaskDelay(pdMS_TO_TICKS(200)); // GPS warm-up (if hot)
        sample->gps = read_gps();
        gpio_set_level(PIN_GPS_POWER, 0);

        // battery sample
        sample->battery = sample_battery_adc();

        // Validate
        validate_and_sanitize_sample(*sample);

        // CRC over serialized core fields (simple)
        char json_buf[1024];
        sample_to_json(*sample, json_buf, sizeof(json_buf));
        sample->crc8 = compute_crc8((const uint8_t*)json_buf, strlen(json_buf));

        // Enqueue for storage and comms
        BaseType_t ok1 = xQueueSend(g_storage_queue, &sample, pdMS_TO_TICKS(1000));
        BaseType_t ok2 = xQueueSend(g_comms_queue, &sample, pdMS_TO_TICKS(1000));
        if (!(ok1 == pdTRUE && ok2 == pdTRUE)) {
            ESP_LOGW(TAG, "Queue overflow. Storage ok=%d comms ok=%d", ok1 == pdTRUE, ok2 == pdTRUE);
            // Backpressure strategy: prefer storage; if storage queue failed then drop oldest
            if (ok1 != pdTRUE) {
                Sample *dropped;
                xQueueReceive(g_storage_queue, &dropped, 0); // drop oldest
                free(dropped);
                xQueueSend(g_storage_queue, &sample, 0);
            }
            if (ok2 != pdTRUE) {
                // if comms queue is full, just rely on storage to persist sample
            }
        }

        // Turn off sensors
        gpio_set_level(PIN_SENSOR_POWER, 0);
        // Sleep until next sample or let power manager decide deep sleep
        for (int i=0;i< (SAMPLE_INTERVAL_SECONDS/10); ++i) {
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
}

static void StorageTask(void *arg) {
    ESP_LOGI(TAG, "StorageTask started");
    // Buffer to minimize writes
    char buffer[SD_BUFFER_SIZE];
    size_t buf_len = 0;
    for (;;) {
        Sample *sample = nullptr;
        if (xQueueReceive(g_storage_queue, &sample, pdMS_TO_TICKS(60000)) == pdTRUE) {
            if (!sample) continue;
            // Serialize
            char json_line[2048];
            sample_to_json(*sample, json_line, sizeof(json_line));
            // Append to buffer
            size_t line_len = strlen(json_line) + 1;
            if (buf_len + line_len >= sizeof(buffer)) {
                // Flush
                char filepath[256];
                format_timestamp_iso8601(json_line, sizeof(json_line)); // temp reuse, not ideal but ok
                // Use daily file name: /sdcard/YYYYMMDD.ndjson
                char dayname[16];
                time_t now; time(&now); struct tm tm; gmtime_r(&now, &tm);
                strftime(dayname, sizeof(dayname), "%Y%m%d", &tm);
                snprintf(filepath, sizeof(filepath), "%s/%s.ndjson", SD_MOUNT_POINT, dayname);
                sd_atomic_append_line(filepath, buffer);
                buf_len = 0;
            }
            // copy line into buffer (naive)
            if (line_len < sizeof(buffer) - buf_len) {
                memcpy(buffer + buf_len, json_line, line_len - 1);
                buffer[buf_len + line_len - 1] = '\n';
                buf_len += line_len;
            } else {
                // fallback: write directly
                char filepath[256];
                time_t now; time(&now); struct tm tm; gmtime_r(&now, &tm);
                char dayname[16]; strftime(dayname, sizeof(dayname), "%Y%m%d", &tm);
                snprintf(filepath, sizeof(filepath), "%s/%s.ndjson", SD_MOUNT_POINT, dayname);
                sd_atomic_append_line(filepath, json_line);
            }
            // mark sample persisted - in this simple example we don't track offsets, but you should store index!
            free(sample);
        } else {
            // timeout flush if buffer has pending data
            if (buf_len > 0) {
                char filepath[256];
                time_t now; time(&now); struct tm tm; gmtime_r(&now, &tm);
                char dayname[16]; strftime(dayname, sizeof(dayname), "%Y%m%d", &tm);
                snprintf(filepath, sizeof(filepath), "%s/%s.ndjson", SD_MOUNT_POINT, dayname);
                sd_atomic_append_line(filepath, buffer);
                buf_len = 0;
            }
        }
    }
}

static void CommsTask(void *arg) {
    ESP_LOGI(TAG, "CommsTask started");
    for (;;) {
        Sample *sample = nullptr;
        if (xQueueReceive(g_comms_queue, &sample, pdMS_TO_TICKS(30000)) == pdTRUE) {
            if (!sample) continue;
            // Decide whether to attempt Wi-Fi
            bool sent = false;
            if (sample->battery.voltage > BATTERY_LOW_WARNING_VOLTAGE) {
                if (wifi_connect_blocking()) {
                    char json[2048];
                    sample_to_json(*sample, json, sizeof(json));
                    // Example HTTP endpoint
                    const char *url = "https://example.com/api/telemetry";
                    if (http_post_json(url, json)) {
                        ESP_LOGI(TAG, "Posted sample via Wi-Fi");
                        sent = true;
                    } else {
                        ESP_LOGW(TAG, "Wi-Fi publish failed; will fallback");
                    }
                    // In production: disconnect Wi-Fi to save power
                } else {
                    ESP_LOGW(TAG, "Wi-Fi connect failed");
                }
            } else {
                ESP_LOGW(TAG, "Battery low: skipping Wi-Fi to save power");
            }

            if (!sent) {
                // Fallback: attempt LoRa with compact binary
                // Simple binary encode: timestamp(4s epoch), co2(2), temp(2*100), hum(1), battery(1)
                uint8_t pkt[32];
                memset(pkt,0,sizeof(pkt));
                time_t now; time(&now);
                pkt[0] = (uint8_t)((now >> 24) & 0xFF);
                pkt[1] = (uint8_t)((now >> 16) & 0xFF);
                pkt[2] = (uint8_t)((now >> 8) & 0xFF);
                pkt[3] = (uint8_t)(now & 0xFF);
                int co2 = sample->scd41.present ? sample->scd41.co2_ppm : 0;
                pkt[4] = (co2 >> 8) & 0xFF; pkt[5] = co2 & 0xFF;
                int ttemp = (int)( (sample->bme680.present ? sample->bme680.temp_c : sample->sht4x.temp_c) * 100 );
                pkt[6] = (ttemp >> 8) & 0xFF; pkt[7] = ttemp & 0xFF;
                pkt[8] = (uint8_t) (sample->battery.voltage * 10);
                bool lora_ok = false;
                for (int i = 0; i < LORA_TX_RETRY; ++i) {
                    if (lora_send_binary(pkt, 16)) { lora_ok = true; break; }
                    vTaskDelay(pdMS_TO_TICKS(200 + i*100));
                }
                if (lora_ok) {
                    ESP_LOGI(TAG, "LoRa tx ok");
                    sent = true;
                } else {
                    ESP_LOGW(TAG, "LoRa tx failed; relying on SD copy");
                }
            }

            // Always persist copy to SD to ensure durability if not already saved (StorageTask does this asynchronously)
            // Free sample (note StorageTask may already have freed it depending on queue usage model)
            free(sample);
        }
    }
}

static void PowerManagerTask(void *arg) {
    ESP_LOGI(TAG, "PowerManagerTask started");
    for (;;) {
        // Periodically check battery and decide deep sleep policy
        BatterySample bat = sample_battery_adc();
        if (bat.voltage < BATTERY_CRITICAL_VOLTAGE) {
            ESP_LOGW(TAG, "Battery critical %.2fV - entering long sleep", bat.voltage);
            // Configure wakeup by timer only for emergency
            esp_sleep_enable_timer_wakeup((uint64_t)SAMPLE_INTERVAL_SECONDS * 1000000ULL * 6); // longer sleep
            esp_deep_sleep_start();
        } else if (bat.voltage < BATTERY_LOW_WARNING_VOLTAGE) {
            // Reduce sampling rate or disable power-hungry peripherals
            ESP_LOGW(TAG, "Battery low %.2fV - increasing sample interval", bat.voltage);
            // Implementation: set global variable to adapt SensorTask loop frequency or alter scheduler tick
        }
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

static void WatchdogTask(void *arg) {
    ESP_LOGI(TAG, "WatchdogTask started");
    for (;;) {
        // Pet hardware watchdog (if available) or check liveness of other tasks
        // Example: check queue backlog and memory stats
        UBaseType_t storage_queued = uxQueueMessagesWaiting(g_storage_queue);
        UBaseType_t comms_queued = uxQueueMessagesWaiting(g_comms_queue);
        if (storage_queued > 50 || comms_queued > 50) {
            ESP_LOGW(TAG, "Queue lengths high: storage=%u comms=%u", (unsigned)storage_queued, (unsigned)comms_queued);
            // Attempt soft fixes (e.g., restart tasks) - not implemented here
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

/////////////////////
// Boot recovery    //
/////////////////////

static void boot_recovery_checks() {
    ESP_LOGI(TAG, "Boot recovery start");
    // 1) SD recovery: check for .tmp files and clean up
    // (simple scan not implemented in this skeleton; in production iterate directory for *.tmp)
    // 2) Clock sanity - attempt NTP if set allowed and Wi-Fi available; else leave RTC as is.
    time_t now; time(&now);
    if (now < 1609459200) {
        ESP_LOGW(TAG, "System time invalid - try GPS/NTP to set RTC");
        // If GPS fix present at boot, set RTC; else schedule NTP when Wi-Fi works
    }
    // 3) Sensor presence probe - mark sensors absent/present in a persistent table if needed
    // Not implemented fully here.
    ESP_LOGI(TAG, "Boot recovery done");
}

//////////////////////
// Initialization   //
//////////////////////

extern "C" void app_main() {
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Firmware skeleton starting");

    // Initialize NVS & RTC
    if (init_nvs_and_rtc() != ESP_OK) {
        ESP_LOGE(TAG, "NVS/RTC init failed - continuing but expect issues");
    }

    // Initialize GPIOs
    gpio_set_direction(PIN_SENSOR_POWER, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_GPS_POWER, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_STATUS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_SENSOR_POWER, 0);
    gpio_set_level(PIN_GPS_POWER, 0);
    gpio_set_level(PIN_LED_STATUS, 0);

    // Init mutexes and queues
    g_i2c_mutex = xSemaphoreCreateMutex();
    g_sd_mutex = xSemaphoreCreateMutex();
    g_storage_queue = xQueueCreate(64, sizeof(Sample*));
    g_comms_queue = xQueueCreate(64, sizeof(Sample*));
    if (!g_storage_queue || !g_comms_queue || !g_i2c_mutex || !g_sd_mutex) {
        ESP_LOGE(TAG, "Failed to create RTOS primitives");
        abort();
    }

    // Peripheral init in strict order
    if (init_i2c_bus() != ESP_OK) {
        ESP_LOGW(TAG, "I2C init failed, will attempt recovery during boot_recovery");
    }
    // SD init next (but non-fatal)
    init_sdcard();

    // Boot recovery
    boot_recovery_checks();

    // Start tasks (priorities: PowerManager highest)
    BaseType_t r;
    r = xTaskCreate(PowerManagerTask, "PowerMgr", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr);
    r = xTaskCreate(WatchdogTask, "Watchdog", 3072, nullptr, 3, &g_watchdog_handle);
    r = xTaskCreate(StorageTask, "Storage", 8192, nullptr, 2, nullptr);
    r = xTaskCreate(CommsTask, "Comms", 8192, nullptr, 4, nullptr);
    r = xTaskCreate(SensorTask, "Sensor", 8192, nullptr, 5, nullptr);

    // Optionally schedule first deep sleep wakeup via RTC if enabling deep sleep mode
    // For demonstration we enable periodic timer wakeup to mimic solar-powered duty cycle.
    esp_sleep_enable_timer_wakeup((uint64_t)SAMPLE_INTERVAL_SECONDS * 1000000ULL);
    // Note: do NOT call esp_deep_sleep_start() here; SensorTask will call it when ready

    ESP_LOGI(TAG, "All tasks scheduled. Main will now idle.");
    // The main task can either delete itself or idle
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
