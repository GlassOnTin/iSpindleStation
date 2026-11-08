<h1 align="center">iSpindleStation</h1>

<p align="center">
  Open-source ESP8266 firmware &amp; fermentation analytics station for iSpindel hydrometers —<br/>
  GC9A01 240×240 round TFT · onboard DS18B20 &amp; vessel pressure transducer · piezo alarms · buffering <code>ispindle-proxy</code> with zero-touch hydrometer config/OTA push · Bayesian Gompertz curve fitting · 12 cloud &amp; Home Assistant MQTT forwarders
</p>

<p align="center">
  <a href="https://github.com/GlassOnTin/iSpindleStation/commits/main"><img src="https://img.shields.io/badge/version-v0.4.0-blue?style=flat-square" alt="Version v0.4.0" /></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-MIT-green?style=flat-square" alt="License MIT" /></a>
  <img src="https://img.shields.io/badge/ESP8266-4MB_Flash-e7352c?style=flat-square&logo=espressif&logoColor=white" alt="ESP8266 4MB Flash" />
  <img src="https://img.shields.io/badge/TFT-GC9A01_240%C3%97240-38dbff?style=flat-square" alt="GC9A01 240x240" />
  <img src="https://img.shields.io/badge/Home_Assistant-MQTT_Discovery-41bdf5?style=flat-square&logo=homeassistant&logoColor=white" alt="Home Assistant MQTT Discovery" />
  <a href="https://github.com/GlassOnTin/iSpindlePlotter"><img src="https://img.shields.io/badge/companion-iSpindlePlotter-20e060?style=flat-square&logo=android&logoColor=white" alt="iSpindlePlotter Companion" /></a>
  <a href="https://ko-fi.com/glassontin"><img src="https://img.shields.io/badge/Ko--fi-support-ff5e5b?style=flat-square&logo=ko-fi&logoColor=white" alt="Ko-fi" /></a>
</p>

<p align="center">
  <a href="docs/hardware.md">Hardware Architecture</a> &bull;
  <a href="docs/features.md">Feature Specification</a> &bull;
  <a href="docs/protocol.md">HTTP &amp; Config Surface</a> &bull;
  <a href="https://github.com/GlassOnTin/iSpindlePlotter">iSpindlePlotter Android App</a>
</p>

---

### Round GC9A01 240×240 TFT Display

<p align="center">
  <img src="docs/images/lcd_single.png" width="230" alt="Single iSpindel + Onboard Relay Sensors" />
  &nbsp;&nbsp;
  <img src="docs/images/lcd_dual.png" width="230" alt="Dual iSpindel Hemispheres + Equator Relay Pill" />
  &nbsp;&nbsp;
  <img src="docs/images/lcd_hardware.jpg" width="305" alt="Live iRelay NXT Hardware Running iSpindleStation v0.4.0" />
</p>

<p align="center">
  <sub><b>Left:</b> Single iSpindel in upper hemisphere with onboard DS18B20 temperature &amp; vessel pressure in lower hemisphere. &nbsp;|&nbsp; <b>Center:</b> Simultaneous dual-iSpindel view with onboard relay sensors in the equator pill. &nbsp;|&nbsp; <b>Right:</b> Live iRelay NXT hardware running <code>v0.4.0</code>.</sub>
</p>

---

### On-Device Web UI — Bayesian Gompertz Fitting, Phase Guidance &amp; Zero-Touch Calibration

<p align="center">
  <img src="docs/images/web_analytics.png" width="920" alt="Dashboard, Spunding Guidance, and Bayesian Gompertz Curve Fit" />
</p>

<p align="center">
  <sub><b>Dashboard &amp; Bayesian Modified-Gompertz Model:</b> Dual-slot telemetry cards, onboard DS18B20 &amp; vessel pressure card, 7-phase state classification (<code>Slowing</code>), live spunding-valve pressure target paired with the <code>A0</code> transducer, shaded <code>Lag</code> / <code>Mid</code> (diauxic shift) / <code>Tail</code> plateaus, and dashed Gompertz curve with 95% credible band.</sub>
</p>

<p align="center">
  <img src="docs/images/web_diauxic_pause.png" width="452" alt="Stuck / Diauxic Stall Detection and Rousing Guidance" />
  &nbsp;
  <img src="docs/images/web_calibration.png" width="452" alt="Degree 1-3 Polynomial Calibration and Piggybacked Config Push" />
</p>

<p align="center">
  <sub><b>Left:</b> Mid-ferment stall / stuck fermentation detection with rousing &amp; temperature guidance and spunding lockout. &nbsp;|&nbsp; <b>Right:</b> Degree 1–3 OLS polynomial calibration (<code>SG = f(tilt)</code>) and zero-touch piggybacked <code>POST /gravity</code> config push (<code>/cfg</code>).</sub>
</p>

---

## At a Glance

- **[GC9A01 240×240 Round TFT](src/display.h)** — Simultaneous upper/lower hemisphere layout for two iSpindels (Specific Gravity or °Plato, temperature, battery voltage, perimeter status/alarm arcs, and center heartbeat dot), plus live onboard relay DS18B20 temperature and vessel pressure readout, runtime backlight (`GPIO 15`), and 4-way rotation (`0..3`).
- **[Hardware Peripherals & Alarms](src/sensors.h)** — Piezo buzzer (`GPIO 12`, 1800 Hz save confirmation and 2000 Hz alarm tone), onboard DS18B20 1-Wire thermometer (`GPIO 2`, 11-bit non-blocking 10 s poll with 6-strike fault debounce), onboard analog pressure transducer (`A0`, 15 s poll with linear Bar/Psi calibration), and 10 configurable temperature/gravity/pressure alarm bounds.
- **[Buffering Proxy & Zero-Touch Hydrometer Management](src/proxy.h)** — Native `ispindle-proxy` compatibility: mDNS advertisement (`_ispindle-proxy._tcp` on port `80`), monotonic cursor catch-up feed (`GET /readings?since=N`), and piggybacked `POST /gravity` response configuration/OTA push (`GET/PUT/DELETE /cfg`, `/fw`, `/fw.bin`).
- **[On-Device & Browser Fermentation Analytics](src/webui.h)** — 5th-sample OG auto-lock, ABV %, apparent attenuation %, episode segmentation, bad-data exclusions (`/exclusions`), krausen-clamped OG, Levenberg-Marquardt modified-Gompertz curve fitting with 95% credible band and ETA, diauxic plateau shading, 7-phase state machine (`Lag`, `Active`, `Slowing`, `Conditioning`, `Clarifying`, `Stuck`, `ColdCrash`), degree 1–3 polynomial calibration calculator, and spunding-pressure guidance paired with live `A0` vessel pressure.
- **[Cloud & Local Forwarding](src/forwarders.h)** — 12 forwarding services (`0` UbiDots, `1` Brewfather, `2` BrewSpy, `3` Monitor.Beer, `4` Grainfather, `5` BrewPiless, `6` Little Bock, `7` Brewers Friend, `8` BrewBench, `9` MQTT + 17 Home Assistant Auto-Discovery sensors, `10` Generic HTTP `host:port`, `11` NAUTILIS Tapboard) across both on-receipt iSpindel forwarding and periodic onboard relay sensor uploads.

See [docs/features.md](docs/features.md) for the full feature specification, [docs/hardware.md](docs/hardware.md) for the hardware architecture and Mermaid diagrams, and [docs/protocol.md](docs/protocol.md) for the HTTP and `/config.json` schema.

## Build &amp; Flash

```bash
# Compile firmware (.pio/build/relay/firmware.bin)
pio run

# Flash over USB serial
pio run -t upload

# Or upload over WiFi OTA to a running station
curl -F "update=@.pio/build/relay/firmware.bin" http://<station-ip>/update
```

## Hardware Mock Test Suite

[`test/mock_test_forwarders.py`](test/mock_test_forwarders.py) starts a local mock HTTP server and mock MQTT v3.1.1 broker to test all 12 forwarding services (both on-receipt and periodic relay-sensor uploads) against a live station without altering its stored `/config.json`:

```bash
python3 test/mock_test_forwarders.py
```

## License

MIT (`LICENSE`). Vendored: `lib/WiFiManagerKT` — MIT, Ken Taylor's fork of WiFiManager.
