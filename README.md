# LD2450 Zigbee Radar

ESP32-C6 firmware that bridges a Hi-Link **LD2450 mmWave radar sensor** to a Zigbee network, reporting occupancy to a coordinator such as Zigbee2MQTT or Home Assistant.

## Overview

The device acts as a **Zigbee End Device** (ZED). It reads up to 3 targets from the LD2450 over UART and reports binary occupancy via the standard ZCL Occupancy Sensing cluster. A custom cluster (`0xFF00`) exposes runtime-configurable radar parameters.

```
LD2450 sensor  --UART-->  ESP32-C6  --Zigbee-->  Coordinator (Z2M / HA)
```

## Hardware

| Signal | ESP32-C6 GPIO |
|--------|--------------|
| UART TX (to LD2450 RX) | GPIO 22 |
| UART RX (from LD2450 TX) | GPIO 23 |

UART: 256000 baud, 8N1, no flow control.

## Project Structure

```
├── main/
│   └── main.c                     Entry point
├── components/
│   ├── LD2450_driver/             UART driver for the LD2450 sensor
│   │   ├── LD2450_driver.c
│   │   └── include/LD2450_driver.h
│   └── Zigbee_handler/            Zigbee stack integration
│       ├── Zigbee_handler.c
│       └── include/Zigbee_handler.h
├── z2m_converter/
│   └── new.js                     Zigbee2MQTT external converter
├── partitions.csv                 Custom partition table
└── CMakeLists.txt
```

## Zigbee Device Profile

- **Vendor**: JUREK
- **Model**: JP0
- **Endpoint**: 10
- **Profile**: Home Automation (HA)

### Clusters

| Cluster | ID | Direction | Description |
|---|---|---|---|
| Basic | `0x0000` | Server | Manufacturer: JUREK, Model: JP0 |
| Identify | `0x0003` | Server + Client | |
| Occupancy Sensing | `0x0406` | Server | Occupancy attribute, auto-reported on change |
| Radar Config | `0xFF00` | Server (custom) | Runtime radar parameters |

### Custom Cluster Attributes (`0xFF00`)

| Attr ID | Name | Type | Range | Default | Description |
|---|---|---|---|---|---|
| `0x0000` | `range` | U16 | 10–10000 | 300 | Detection range (cm) |
| `0x0001` | `timeout` | U16 | 1–3600 | 20 | Occupancy off-delay (s) |
| `0x0002` | `update_rate` | U8 | 1–100 | 20 | Update interval |
| `0x0003` | `angle` | U16 | 0–180 | 0 | Installation angle (°) |

## Zigbee2MQTT Integration

Add the external converter from `z2m_converter/new.js` to your Z2M configuration:

```yaml
# configuration.yaml
external_converters:
  - new.js
```

The converter exposes `occupancy`, `range`, `timeout`, `update_rate`, and `angle` as readable/writable entities.

## Pairing

On first boot (or after a factory reset) the device enters network steering automatically. In Zigbee2MQTT, enable joining and the device will appear as model **JP0**.

On subsequent reboots the device rejoins its previous network and reports current attribute values.

## LD2450 Driver Configuration

Default values set in `LD2450_Init()`:

| Parameter | Default |
|---|---|
| Max detection distance | 600 cm |
| Installation angle | 0° |
| Inverse mounting | false |
| Update interval | 300 ms |
| Aggressive target clearing | false |
| Aggressive timeout | 3000 ms |
