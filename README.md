# iSpindleStation

Open-source ESP8266 firmware for iSpindel relay stations (including GC9A01 240×240 round-TFT relay hardware). It receives readings over WiFi (`POST /gravity`), buffers per-spindle history in flash, renders a dual-hemisphere instrument display on the round TFT, serves an interactive fermentation analytics web UI, syncs with the [iSpindlePlotter](https://github.com/GlassOnTin/iSpindlePlotter) Android app, and forwards readings to cloud or local logging services.

- **Hardware Architecture**: [docs/hardware.md](docs/hardware.md)
- **Feature Specification**: [docs/features.md](docs/features.md)
- **HTTP & Configuration Surface**: [docs/protocol.md](docs/protocol.md)

## Highlights

- **GC9A01 240×240 Round TFT (`src/display.h`)**:
  - Simultaneous upper/lower hemisphere layout for two iSpindels (Specific Gravity or °Plato, temperature, battery voltage, perimeter status/alarm arcs, and center heartbeat dot).
  - Live onboard relay DS18B20 temperature and vessel pressure readout in the idle hemisphere or equator pill, plus runtime backlight (`GPIO 15`) and rotation (`0..3`) control.
- **Hardware Peripherals & Alarms (`src/sensors.h`)**:
  - Piezo buzzer (`GPIO 12`, 1800 Hz save confirmation and 2000 Hz alarm tone), onboard DS18B20 1-Wire thermometer (`GPIO 2`, 11-bit non-blocking 10 s poll with 6-strike fault debounce), onboard analog pressure transducer (`A0`, 15 s poll with linear Bar/Psi calibration), and 10 configurable temperature/gravity/pressure alarm bounds.
- **Buffering Proxy & Zero-Touch Hydrometer Management (`src/proxy.h`)**:
  - Native `ispindle-proxy` compatibility: mDNS advertisement (`_ispindle-proxy._tcp` on port `80`), monotonic cursor catch-up feed (`GET /readings?since=N`), and piggybacked `POST /gravity` response configuration/OTA push (`GET/PUT/DELETE /cfg`, `/fw`, `/fw.bin`).
- **On-Device & Browser Fermentation Analytics (`src/state.h`, `src/webui.h`)**:
  - 5th-sample OG auto-lock, ABV %, apparent attenuation %, episode segmentation, bad-data exclusions (`/exclusions`), krausen-clamped OG, Levenberg-Marquardt modified-Gompertz curve fitting with 95% credible band and ETA, diauxic plateau shading, 7-phase state machine (`Lag`, `Active`, `Slowing`, `Conditioning`, `Clarifying`, `Stuck`, `ColdCrash`), degree 1-3 polynomial calibration calculator, and spunding-pressure guidance paired with live `A0` vessel pressure.
- **Cloud & Local Forwarding (`src/forwarders.h`)**:
  - 12 forwarding services (`0` UbiDots, `1` Brewfather, `2` BrewSpy, `3` Monitor.Beer, `4` Grainfather, `5` BrewPiless, `6` Little Bock, `7` Brewers Friend, `8` BrewBench, `9` MQTT + 17 Home Assistant Auto-Discovery sensors, `10` Generic HTTP `host:port`, `11` NAUTILIS Tapboard) across both on-receipt iSpindel forwarding and periodic onboard relay sensor uploads.

## Build & Flash

```bash
# Compile firmware (.pio/build/relay/firmware.bin)
pio run

# Flash over USB serial
pio run -t upload

# Or upload over WiFi OTA to a running station
curl -F "update=@.pio/build/relay/firmware.bin" http://<station-ip>/update
```

## Hardware Mock Test Suite

`test/mock_test_forwarders.py` starts a local mock HTTP server and mock MQTT v3.1.1 broker to test all 12 forwarding services (both on-receipt and periodic relay-sensor uploads) against a live station without altering its stored `/config.json`:

```bash
python3 test/mock_test_forwarders.py
```

## License

AGPL-3.0. Vendored: `lib/WiFiManagerKT` — MIT, Ken Taylor's fork of WiFiManager.
