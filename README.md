# ESP32-C6 All-Inclusive Weather Station

A comprehensive environmental monitoring system designed for autonomous outdoor deployment. This weather station integrates meteorological sensors, air quality monitoring, GPS tracking, and solar power management into a single robust PCB design.


<img width="784" height="560" alt="Screenshot 2025-11-29 at 9 49 39 PM" src="https://github.com/user-attachments/assets/a9a9ceb8-e459-468b-9033-2f630ab46786" />
<img width="847" height="590" alt="Screenshot 2025-11-29 at 9 50 54 PM" src="https://github.com/user-attachments/assets/da6bb370-c7f4-49a7-84ae-6a2df201defb" />
<img width="626" height="547" alt="Screenshot 2025-11-29 at 9 50 24 PM" src="https://github.com/user-attachments/assets/1ea0e025-81eb-43bf-8b84-14139492d735" />
<img width="541" height="433" alt="Screenshot 2025-11-29 at 9 52 07 PM" src="https://github.com/user-attachments/assets/798035f8-af06-425f-8ddf-ccbe0df4cc02" />

---

## Features

- **Microcontroller:** ESP32-C6-MINI (Wi-Fi 6, Bluetooth 5.2, low-power deep sleep)
- **Environmental Sensors:**
  - BME680: Temperature, Humidity, Pressure, VOC
  - SHT4x: Temperature and Humidity
  - SCD41: CO2, Temperature, Humidity
  - TSL25911: Ambient Light
- **GPS Module:** NEO-M8N with external U.FL antenna
- **Air Quality & Power Monitoring:**
  - Battery voltage, charge state, solar charging support
  - AQI estimation
- **Storage:** MicroSD card (DM3D-SF)
- **Connectivity:** Wi-Fi (primary), LoRa (fallback)
- **Power Management:** Deep sleep with peripheral power gating
- **Security:** TLS for Wi-Fi communications, optional HMAC payload signatures, secure key storage in NVS
- **Optional LEDs:** WS2812B RGB for status indicators

---

## Architecture

The firmware uses an RTOS-based task structure:

| Task               | Priority | Description |
|-------------------|----------|-------------|
| SensorTask         | High     | Wake peripherals, read sensors, push measurements to queues |
| CommsTask          | High     | Wi-Fi/MQTT/HTTP upload, fallback SD logging, LoRa backup |
| StorageTask        | Medium   | Buffered SD writes, CRC/JSON formatting |
| PowerManagerTask   | Highest  | Sensor power control, deep sleep, battery monitoring |
| WatchdogTask       | Low      | Detects stuck tasks or I²C bus lock and resets if necessary |

---

## Data Model

```json
{
  "device_id":"mjohnson-weather-v1",
  "timestamp":"2025-11-28T12:52:24Z",
  "sensors": {
    "bme680": {"temp_c": 23.54, "hum_pct": 45.2, "press_hpa": 1013.2, "gas_ohms": 120},
    "sht4x": {"temp_c": 23.50, "hum_pct": 45.1},
    "scd41": {"co2_ppm": 420, "temp_c": 23.4, "rh": 45.0},
    "tsl25911": {"lux": 3200},
    "gps": {"lat": 40.123456, "lon": -73.123456, "fix": true, "hdop": 0.9},
    "battery": {"voltage": 7.40, "charge_pct": 85, "charging": true},
    "aqi_est": 35
  },
  "signature": "xxxxx"  // CRC-8 checksum
} 
