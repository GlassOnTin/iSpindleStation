# Hardware Architecture

`iSpindleStation` targets ESP8266 relay station hardware (4 MB SPI flash) equipped with a round 240×240 GC9A01 RGB565 TFT display, a 1-Wire DS18B20 temperature probe, an analog vessel-pressure transducer, and a piezo alarm buzzer.

## Board & Peripheral Topology

```mermaid
flowchart LR
    subgraph Power["Power & Analog Front-End"]
        USB["5 V DC Supply"] --> REG["3.3 V LDO Regulator"]
        PT["Vessel Pressure Transducer"] -->|"0.5 - 4.5 V ratiometric"| DIV["Resistor Divider (4.5 V FS)"]
    end

    subgraph MCU["ESP8266EX Module (80/160 MHz, 80 KB RAM)"]
        CORE["Xtensa LX106 CPU & Dual WiFi (AP + STA)"]
        HSPI["Hardware SPI1 (HSPI @ 20 MHz)"]
        OW["1-Wire Bus Controller"]
        PWM["Timer PWM / Tone Generator"]
        ADC["10-bit SAR ADC (0 - 1.0 V)"]
        QIO["SPI0 Flash Interface"]
    end

    subgraph Storage["On-Module Flash (4 MB)"]
        FW["Firmware Sketch (~537 KB)"]
        OTA["OTA Staging Slot (~1 MB)"]
        FS["SPIFFS (~2 MB: /config.json, /0..2.jsonl, /ispindel_fw.bin)"]
    end

    subgraph Display["GC9A01 240x240 Round TFT"]
        PANEL["GC9A01 SPI Controller (RGB565)"]
        BL["LED Backlight"]
    end

    subgraph Sensors["External Probes & Audio"]
        DS["DS18B20 Thermometer Probe (11-bit)"]
        BUZ["Piezo Buzzer (1800 Hz / 2000 Hz)"]
    end

    DIV -->|"A0 / ADC0 (Pin 17)"| ADC
    QIO <-->|"SPI0"| Storage

    HSPI -->|"GPIO 14 (D5) SCK"| PANEL
    HSPI -->|"GPIO 13 (D7) MOSI"| PANEL
    CORE -->|"GPIO 0 (D3) CS"| PANEL
    CORE -->|"GPIO 4 (D2) DC"| PANEL
    CORE -->|"GPIO 5 (D1) RST"| PANEL
    CORE -->|"GPIO 15 (D8) BLK"| BL

    OW <-->|"GPIO 2 (D4) Data"| DS
    PWM -->|"GPIO 12 (D6) Drive"| BUZ
```

## Radio & Network Dataflow

The ESP8266 radio operates in simultaneous `WIFI_AP_STA` mode so battery-powered iSpindel hydrometers can post to a dedicated local access point (`192.168.4.1/24`) while the station serves its web dashboard, syncs with the `iSpindlePlotter` Android app over mDNS, and forwards telemetry to cloud or LAN endpoints.

```mermaid
flowchart TD
    subgraph Vessels["Fermentation Vessels"]
        ISP1["iSpindel Slot 1 (1-*)"]
        ISP2["iSpindel Slot 2 (2-*)"]
        PROBE["Onboard DS18B20 & Pressure Sensor"]
    end

    subgraph Relay["iSpindleStation (ESP8266)"]
        AP["Soft-AP Interface (192.168.4.1/24)"]
        INGEST["HTTP Server (:80) POST /gravity"]
        RING["RAM Ring Buffer (3 x 300 pts) & SPIFFS JSONL"]
        TFT["GC9A01 Dual-Hemisphere Display & Buzzer Alarms"]
        STA["Station Interface (Home WiFi LAN)"]
    end

    subgraph Clients["LAN & Cloud Consumers"]
        APP["iSpindlePlotter Android App (mDNS _ispindle-proxy._tcp)"]
        WEB["Browser Web UI (Gompertz Fit, 7-Phase & Calibration)"]
        CLOUD["12 Forwarding Services (HTTP / Home Assistant MQTT)"]
    end

    ISP1 -->|"POST /gravity (JSON)"| AP
    ISP2 -->|"POST /gravity (JSON)"| AP
    AP -->|"HTTP 200 {'status':'ok', 'cfg':{...}} (POLY, sleep, WiFi, OTA)"| ISP1
    AP -->|"HTTP 200 {'status':'ok', 'cfg':{...}} (POLY, sleep, WiFi, OTA)"| ISP2

    AP --> INGEST
    PROBE -->|"10 s / 15 s Poll"| RING
    INGEST --> RING
    RING --> TFT
    RING --> STA

    STA <-->|"GET /readings?since=N & PUT /cfg"| APP
    STA <-->|"GET / & /api/status"| WEB
    STA -->|"On-Receipt & Periodic Uploads"| CLOUD
```

## GPIO Pin Assignment Summary

| ESP8266 Pin | NodeMCU Label | Direction | Connected Subsystem | Notes |
| :--- | :--- | :--- | :--- | :--- |
| `GPIO 14` | `D5` | Output | GC9A01 `SCK` | Hardware HSPI clock (`20 MHz`) |
| `GPIO 13` | `D7` | Output | GC9A01 `MOSI` | Hardware HSPI data |
| `GPIO 0` | `D3` | Output | GC9A01 `CS` | Display chip select (active low; boot strap high) |
| `GPIO 4` | `D2` | Output | GC9A01 `DC` | Display data/command select |
| `GPIO 5` | `D1` | Output | GC9A01 `RST` | Display hardware reset |
| `GPIO 15` | `D8` | Output | GC9A01 `BLK` | Backlight enable (`ledState`: `1` = On, `0` = Off; boot strap low) |
| `GPIO 12` | `D6` | Output | Piezo Buzzer | Driven low at idle; `1800 Hz` save tone (`1 s`), `2000 Hz` alarm pulse (`1 s` every `5 s`) |
| `GPIO 2` | `D4` | Bidir | DS18B20 `DQ` | 1-Wire temperature bus (`11-bit` non-blocking conversion, `10 s` poll) |
| `ADC0 (17)` | `A0` | Input (Analog) | Pressure Sensor | 10-bit ADC (`0..1023` mapped to `0..4.5 V`), linear calibration via `minpressure`, `maxpressure`, `maxpressure_bar` (`15 s` poll) |
