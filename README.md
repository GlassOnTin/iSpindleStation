# iSpindleStation

Open-source host for iSpindel hydrometers on ESP8266. It receives readings
over WiFi (`POST /gravity` with the classic generic-HTTP JSON), keeps a
per-spindle history, serves live data and SG graphs from its own web page,
and will forward readings to the usual log services.

It provides an open web and display firmware for ESP8266 iSpindel
relay hardware. The on-device web surface ([docs/protocol.md](docs/protocol.md))
and the unified feature specification ([docs/features.md](docs/features.md))
blend the stock relay hardware and cloud features with the buffering proxy,
piggybacked hydrometer config/OTA push, and fermentation analytics from
[iSpindlePlotter](https://github.com/GlassOnTin/iSpindlePlotter).

## Build

```
pio run
```

## Status

- **v0.4.0** (verified on iRelay NXT hardware, `531 KB` flash / `56.1%` static RAM):
  - **GC9A01 240×240 round TFT (`TFT_eSPI`)**: dual-hemisphere layout (SG / °P, temperature, battery, perimeter status/alarm arcs, center heartbeat dot, plus live onboard Relay DS18B20 & pressure readout in idle hemisphere or equator pill), runtime `ledState` (`GPIO 15`) and `tft_rotate` (`0..3`).
  - **Hardware peripherals & alarms**: piezo buzzer (`GPIO 12`, 1800 Hz save beep & 2000 Hz alarm beep), onboard DS18B20 (`GPIO 2`, 11-bit non-blocking, 10 s poll, 6-strike fault debounce), onboard analog pressure transducer (`A0`, 15 s poll, linear Bar/Psi calibration), and all 10 stock alarm thresholds.
  - **Native `ispindle-proxy` & zero-touch hydrometer management**: mDNS advertisement (`_ispindle-proxy._tcp` on `:80`), monotonic cursor catch-up feed (`GET /readings?since=N`), and piggybacked `POST /gravity` response config/OTA push (`GET/PUT/DELETE /cfg`, `/fw`, `/fw.bin`).
  - **On-device & browser-side `iSpindlePlotter` analytics (`GET /`, `/app.js`)**: 5th-sample OG auto-lock, ABV %, apparent attenuation %, episode segmentation (`FermentSegmenter`), bad-data exclusions (`/exclusions`), krausen-clamped OG, Levenberg-Marquardt modified-Gompertz fitter with 95% credible band and ETA, diauxic plateau shading (`PlateauDetector`), 7-phase classifier (`Lag`, `Active`, `Slowing`, `Conditioning`, `Clarifying`, `Stuck`, `ColdCrash`), degree 1-3 polynomial calibration tool, and spunding-pressure readiness paired with live `A0` vessel pressure.
  - **Cloud & local forwarding**: all 12 stock services (`0` UbiDots, `1` Brewfather, `2` BrewSpy, `3` Monitor.Beer, `4` Grainfather, `5` BrewPiless, `6` Little Bock, `7` Brewers Friend, `8` BrewBench, `9` MQTT + 17 Home Assistant Auto-Discovery sensors, `10` Generic HTTP `host:port`, `11` NAUTILIS Tapboard) plus periodic onboard relay sensor uploads (`custom_period` min).

## License

AGPL-3.0. Vendored: `lib/WiFiManagerKT` — MIT, Ken Taylor's fork of
WiFiManager.
