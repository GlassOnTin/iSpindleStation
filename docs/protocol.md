# Device Web Surface

Specification of the HTTP, mDNS, and configuration endpoints served by `iSpindleStation`.

## Received Payload (`POST /gravity`)

`POST /gravity` (as well as any unregistered `POST` path) accepts the standard iSpindel generic-HTTP JSON payload:
- Standard keys: `name`, `ID`, `angle`, `temperature`, `temp_units`, `battery`, `gravity`, `interval`, `RSSI`, plus optional `token`.
- Extended `ispindle-proxy` acknowledgment keys: `cf` (`"trial"`, `"reverted"`, or `"refused"` for pending WiFi trials) and `fa` (`"err"` or `"skip"` for OTA firmware trials).

Slot routing follows the relay naming convention:
- Payload names starting with `1` route to Slot 1 (`/1.jsonl`).
- Payload names starting with `2` route to Slot 2 (`/2.jsonl`).
- Any other hydrometer name routes to Slot 0 (`/0.jsonl`) when Slots 1 and 2 are occupied.

Response:
- `{"status":"ok"}` by default.
- `{"status":"ok","cfg":{...}}` when a piggybacked hydrometer configuration or OTA update is armed via `PUT /cfg`.

## HTTP Endpoints

| Route | Method | Description |
| :--- | :--- | :--- |
| `/` (`/status`, `/hidro`) | `GET` | Live dashboard with dual-slot cards, relay sensor status, fermentation analytics, interactive chart, and calibration calculator |
| `/api/status` | `GET` | Full JSON state snapshot for browser polling |
| `/gravity` | `POST` | Ingest iSpindel reading, persist to JSONL, update TFT display, and trigger configured forwarder |
| `/readings?since=N` | `GET` | Monotonic cursor catch-up feed (`{"cursor":M,"readings":[{id,ts,ip,payload},...]}`) for the `iSpindlePlotter` Android app |
| `/cfg` | `GET`, `PUT`, `DELETE` | View, arm, or clear piggybacked iSpindel config (`POLY`, `sleep`, one-shot `SSID`+`PSK`, one-shot `fw` OTA) |
| `/fw`, `/fw.bin` | `GET` | Metadata (`{size, md5}`) and binary stream for staged iSpindel firmware (`/ispindel_fw.bin`) |
| `/exclusions` | `GET`, `PUT` | Load or save bad-data exclusion time ranges (`/exclusions.json`) |
| `/data/0.jsonl`..`/data/2.jsonl` | `GET` | Per-slot append-only NDJSON history |
| `/graph/0.svg`..`/graph/2.svg` | `GET` | Server-rendered fallback SVG specific-gravity polyline |
| `/network`, `/config`, `/managment`, `/info` | `GET` | Configuration pages for Network & Cloud, Relay Hardware, iSpindel Calibration/Alarms, and System Info |
| `/get`, `/save_config`, `/save_management`, `/i_get` | `GET` | Merge-save query parameters into `/config.json` and apply runtime settings |
| `/new_fermentation` | `GET`, `POST` | Reset OG lock and fermentation state for `ispindel=1\|2` (or `isp=1\|2`) |
| `/update` | `GET`, `POST` | Web OTA firmware upload for the relay station |
| `/reboot`, `/reset` | `GET`, `POST` | Schedule a clean reboot (`1 s`) or factory-reset `/config.json` and logs |
| `/health`, `/debug` | `GET` | JSON diagnostics (firmware version, SNTP epoch, heap, SPIFFS state, forwarder status, slot ages) |

## Configuration Fields (`/config.json`)

- **Network & Cloud (`/network`)**: `ssid`, `password`, `ap_ssid`, `ap_pass`, `service` (`0..11`, `255` = None), `TOKEN`, `server_address`, `uri`, `custom_period`, `uri_custom`, `grainfather_uri1`, `grainfather_uri2`, `littlebock_uri1`, `littlebock_uri2`, `mqtt_user`, `mqtt_pass`.
- **Relay Hardware (`/config`)**: `tft_rotate` (`0..3`), `ledState` (`0..1`), `buzz_yesno` (`1` = Off, `2` = On), `temp_yesno` (`1` = Off, `2` = On), `temp_unit` (`1` = °C, `2` = °F), `TempCorrection1`, `alarm_3`, `alarm_4`, `pressure_unit` (`1` = Bar, `2` = Psi, `3` = Off), `minpressure`, `maxpressure`, `maxpressure_bar`, `pressure_alarm_low`, `pressure_alarm_high`.
- **Per-iSpindel (`/managment`)**: `ispindel_num` (`1..2`), `unit_1`, `unit_2` (`1` = SG, `2` = Plato), `SGkorekcija1`, `SGkorekcija2`, `Tempkorekcija1`, `Tempkorekcija2`, `OG1`, `OG2`, `alarm_1`, `alarm_2`, `alarm_5`, `alarm_6`, `alarm_SG1`, `alarm_SG2`.

Note: The companion soft-AP runs on `192.168.4.1/24`, so the upstream home WiFi network must use a subnet other than `192.168.4.0/24` for cloud/LAN forwarding to route properly.
