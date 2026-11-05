# Device web surface

The replacement's spec, kept from observed behaviour.

## Received payload

`POST /gravity` accepts the classic iSpindel generic-HTTP JSON. Keys
observed: `name`, `ID`, `angle`, `temperature`, `temp_units`, `battery`,
`gravity`, `interval`, `RSSI`, plus a `token` when the sender includes one
(ignored here).

Slot routing follows the original relay's rule: payload names starting
`1-` go to slot 1 and `2-` to slot 2 (its firmware validates "iSpindel
name must start with 1 or 2"). This firmware sends anything else to slot
0 instead of dropping it.

Response: `{"status":"ok"}`.

## Served by iSpindleStation (v0)

| Route | Method | What |
|---|---|---|
| `/` | GET | live page; last reading per slot; 30 s meta refresh |
| `/gravity` | POST | receive (any unregistered POST path also lands here) |
| `/data/0.jsonl`…`/data/2.jsonl` | GET | full history, one `{"ts":…,"payload":{…}}` line per reading |
| `/graph/0.svg`…`/graph/2.svg` | GET | server-rendered SG polyline, last 400 points |
| `/health` | GET | firmware version, epoch, sntp state |

Timestamps are epoch seconds from SNTP; records stamped before SNTP has
reached the box carry uptime seconds instead (flagged, not silently
mixed — `/health` reports `sntp`).

## Original relay surface (observed on hardware)

Routes: `/network` (network + cloud), `/managment` (per-iSpindel), `/config`
(device), `/hidro` (live data), `/update` (manual `.bin` upload), `/reboot`,
`/reset` (`POST`, factory reset), plus `/get` (network save), `/save_config`,
`/save_management`, `/new_fermentation` (`POST`, `isp=1/2` with `OG1`/`OG2`,
"0 = auto detect" — `OG1` inferred from `OG2`'s form).

Network & Cloud form fields: `ssid`, `password`, `ap_ssid`, `ap_pass`,
`service`; with service HTTP the section adds `TOKEN`, `server_address`,
`uri`, `custom_period`. Device-config fields: `tft_rotate`, `ledState`,
`pressure_unit` (Bar=1, Psi=2), `minpressure` (ADC zero), `maxpressure`
(ADC max), `maxpressure_bar`, `buzz_yesno`, `temp_yesno`, `temp_unit`,
`alarm_3`, `alarm_4`, `TempCorrection1`. Per-iSpindel fields: `i_broj`,
`unit_1/2` (SG=1, Plato=2), `alarm_1`..`alarm_4`, `alarm_SG1/SG2`,
`SGkorekcija1/2`, `Tempkorekcija1/2`; per-spindle URI fields exist for
Grainfather and Little Bock (`grainfather_uri1/2`, `littlebock_uri1/2`,
placeholder `/api/log/ispindle/...`).

Forwarders: ubidots (`industrial.api.ubidots.com`,
`POST /api/v1.6/devices/Nautilis_Thermometer`), brewfather, brewspy,
grainfather, brewersfriend, littlebock, brewbench, monitor.beer, MQTT.

Known template bug: text-input `value="` attributes are unterminated; in
the HTTP service section the TOKEN input then swallows the markup that
should follow, so the Server Address input never renders (both 7.2.4 and
7.5.0). The `/get` endpoint accepts all fields regardless, which is the
working configuration path.

Design note carried from the relay's AP configuration: a home LAN in
192.168.4.x conflicts with the relay AP subnet and breaks forwarding.
