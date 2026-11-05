# iSpindleStation

Open-source host for iSpindel hydrometers on ESP8266. It receives readings
over WiFi (`POST /gravity` with the classic generic-HTTP JSON), keeps a
per-spindle history, serves live data and SG graphs from its own web page,
and will forward readings to the usual log services.

It provides an open web and display firmware for ESP8266 iSpindel
relay hardware. The on-device web surface ([docs/protocol.md](docs/protocol.md))
was mapped from observed behaviour so existing workflows keep working; the
fermentation analytics from the
[iSpindlePlotter](https://github.com/GlassOnTin/iSpindlePlotter) app run in
the served browser page (Kotlin/JS) rather than in firmware.

## Build

```
pio run
```

v0 assumes a d1_mini-class board and is verified by compilation only;
bench flashing and the board pinout come next.

## Status

- v0.1.0 — receive `/gravity`, capped per-slot JSONL history in SPIFFS,
  live page, SVG SG graphs, WiFi config portal.
- Not yet: service forwarding, analytics page (Kotlin/JS), OTA, hardware
  drivers (TFT, buzzer, relay output, pressure sensor).

## License

AGPL-3.0. Vendored: `lib/WiFiManagerKT` — MIT, Ken Taylor's fork of
WiFiManager.
