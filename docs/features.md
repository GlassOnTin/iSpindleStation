# Feature Specification: Stock Relay + iSpindlePlotter Blend

`iSpindleStation` combines two lineages into one open firmware and web application for the ESP8266 relay hardware:

1. **Stock Nautilis iRelay hardware and cloud surface**: GC9A01 240×240 round TFT, piezo buzzer (`GPIO 12`), onboard DS18B20 thermometer (`GPIO 2`), onboard analog pressure transducer (`A0`), drop-in `/config.json` compatibility, and 12 cloud/local forwarding services.
2. **iSpindlePlotter and `ispindle-proxy` capabilities** (`~/Code/iSpindlePlotter` `v0.5.24`): mDNS-discoverable buffering proxy (`GET /readings?since=N`), piggybacked iSpindel configuration and OTA push (`{"status":"ok","cfg":{...}}` on `POST /gravity` responses), multi-brew episode segmentation, outlier and bad-data exclusion, Bayesian modified-Gompertz and biphasic two-component curve fitting, 7-phase fermentation state classification (`Lag`, `Active`, `Slowing`, `Conditioning`, `Clarifying`, `Stuck`, `ColdCrash`), diauxic plateau detection, degree 1-3 polynomial calibration, and spunding-pressure guidance tied directly to the relay's physical pressure sensor.

---

## Unified Feature Matrix

| Subsystem | Stock Relay (`v7.2.4` / `v7.5.0`) | `iSpindlePlotter` + `ispindle-proxy` (`v0.5.24`) | Blended Target in `iSpindleStation` | Status (`v0.4.0`) |
| :--- | :--- | :--- | :--- | :--- |
| **GC9A01 240×240 TFT** | 3-screen 5 s carousel (iSpindel 1, Relay Sensors, iSpindel 2) | Phone UI only | Simultaneous 2-hemisphere dual-spindle view + relay temp/pressure display, runtime `ledState` (`GPIO 15`) and `tft_rotate` (`0..3`) | Implemented (`src/display.h`) |
| **Audio Buzzer (`GPIO 12`)** | Save beep (1800 Hz, 1 s) and alarm beep (2000 Hz, 1 s / 5 s), `buzz_yesno` | Phone notifications | Save beep + configurable alarm buzzer for temp, SG, and pressure bounds | Implemented (`src/sensors.h`) |
| **Onboard DS18B20 (`GPIO 2`)** | 11-bit resolution, 10 s poll, °C/°F, `TempCorrection1`, 6-strike fault debounce, low/high alarms | Uses iSpindel onboard temp | Poll `GPIO 2` every 10 s with 6-strike debounce; display alongside iSpindel wort temp | Implemented & verified on hardware (`12.25 °C` live) |
| **Onboard Pressure (`A0`)** | 15 s poll, linear ADC calibration (`minpressure`, `maxpressure`, `maxpressure_bar`), Bar/Psi/Off, low/high alarms | `Spunding.kt` computes SG gates for `Hold`, `~10 psi` (`0.69 bar`), and `~15 psi` (`1.03 bar`) | Poll `A0` every 15 s and compare live vessel pressure against the active fermentation phase's spunding target | Implemented (`src/sensors.h`, `src/state.h`, `src/webui.h`) |
| **Buffering Proxy & Phone Sync** | None (readings are ephemeral in RAM except OG) | `ispindle-proxy`: append-only JSONL, monotonic `id`, `GET /readings?since=N`, mDNS `_ispindle-proxy._tcp` | Serve `GET /readings?since=N` and advertise `_ispindle-proxy._tcp` on ESP8266 so the Android app syncs directly from the relay | Implemented & verified (`avahi-browse` + `/readings?since=N`) |
| **iSpindel Config & OTA Push** | None (requires opening iSpindel AP portal) | `PUT /cfg` attaches `cfg` (`POLY`, `sleep`, one-shot `SSID`+`PSK` with `cf` ack, one-shot `fw` OTA with `fa` ack) to `POST` response | Attach armed `/cfg_push.json` delta to `POST /gravity` responses and serve `/fw` + `/fw.bin` for zero-touch iSpindel updates | Implemented & verified (`src/proxy.h`) |
| **Calibration** | Constant offsets `SGkorekcija1/2` and `Tempkorekcija1/2` | Degree 1-3 OLS polynomial fit (`SG = f(tilt)`), R², sugar/water mixer, import from device, push `POLY` back to iSpindel | Support both scalar offsets (`SGkorekcija`, `Tempkorekcija`) and browser-side polynomial fitting pushed to the spindle via `/cfg` | Implemented (`src/state.h`, `src/webui.h`) |
| **OG & Pre-Filtering** | Locks OG on 5th sample (`sample_count == 5`) or manual `OG1`/`OG2` | `FermentSegmenter` (auto-splits brews), `SeriesClean.robustOg` (MAD spike filter + lag-plateau median to bypass krausen rise), `Exclusions` (bad-data spans) | Auto-lock OG on device for TFT/Tapboard compatibility, while the browser UI runs `FermentSegmenter`, krausen-clamped `robustOg`, and `/exclusions` | Implemented (`src/state.h`, `src/webui.h`, `src/proxy.h`) |
| **Fermentation Phase Model** | 3 states: `Waiting for OG`, `Fermenting`, `Complete` (24 intervals without >0.001 SG drop) | 7 phases: `Lag` -> `Active` -> `Slowing` -> `Conditioning` -> `Clarifying` (`SettlingDetector`) / `Stuck` -> `ColdCrash` + `PlateauDetector` (diauxic pauses) | Run the 7-phase classifier and diauxic plateau detector in the served web app and expose summary phase + ABV/attenuation on device | Implemented (`src/state.h`, `src/webui.h`) |
| **Curve Fitting & ETA** | None (linear ABV and apparent attenuation formulas only) | Bayesian MAP modified-Gompertz (`AttenuationFit`) + credible bands and ETA | Render Gompertz curve, 95% credible band, diauxic pause bands, and ETA in the served web UI | Implemented (`src/webui.h` `/app.js`) |
| **Cloud & Local Forwarding** | 12 services (1 active at a time) + periodic timer (`custom_period` min) for Relay DS18B20/Pressure + 17 HA MQTT discovery topics | None (phone/LAN only) | Preserve `POST /gravity` on-receipt forwarding, all 12 service presets, 17 HA MQTT auto-discovery topics, and periodic relay sensor publishing | Implemented (`src/forwarders.h`) |

---

## 1. Hardware Peripherals & Pinout

| Pin | NodeMCU | Peripheral | Behavior |
| :--- | :--- | :--- | :--- |
| `GPIO 14` | `D5` | TFT `SCK` | Hardware HSPI clock (`TFT_eSPI`) |
| `GPIO 13` | `D7` | TFT `MOSI` | Hardware HSPI data (`TFT_eSPI`) |
| `GPIO 0` | `D3` | TFT `CS` | Chip Select |
| `GPIO 4` | `D2` | TFT `DC` | Data / Command |
| `GPIO 5` | `D1` | TFT `RST` | Reset |
| `GPIO 15` | `D8` | TFT `BLK` | Backlight (`digitalWrite(15, ledState)`). Controlled by `ledState` (`0` = OFF, `1` = ON, default `1`). |
| `GPIO 12` | `D6` | Piezo Buzzer | Active when `buzz_yesno == 2` (`1` = OFF, `2` = ON). Settings save: `tone(12, 1800, 1000)`. Alarm: `tone(12, 2000, 1000)` rate-limited to once per 5000 ms. |
| `GPIO 2` | `D4` | DS18B20 1-Wire | `OneWire(2)` + `DallasTemperature` at 11-bit resolution. Polled every 10,000 ms. |
| `ADC0 (17)` | `A0` | Pressure Sensor | `analogRead(17)`. Polled every 15,000 ms. |

### Onboard DS18B20 Thermometer

- Config keys: `temp_yesno` (`1` = OFF, `2` = ON), `temp_unit` (`1` = °C, `2` = °F), `TempCorrection1` (float offset added to raw reading).
- Fault debounce: rejects readings `<= -100.0` (disconnect `-127.0 °C` / `-196.6 °F`), `== 85.0` (power-on reset in °C, or `185.0` in °F), or `== 127.0`. After 6 consecutive failed polls (60 s), sets the published value to sentinel `127.0` (`"Sensor error"` in web UI, `"N/A"` on TFT). Any valid reading resets the error counter to `0` and stores `raw + TempCorrection1`.

### Onboard Analog Pressure Transducer & Spunding Integration

- Config keys: `pressure_unit` (`1` = Bar, `2` = Psi, `3` = Off), `minpressure` (ADC count at 0 gauge pressure, default `115`), `maxpressure` (ADC count at full scale, default `334`), `maxpressure_bar` (full-scale pressure in Bar, default `1.52`).
- Linear conversion:
  - Diagnostic voltage: `V = raw_adc * 4.5 / 1024.0`
  - If `maxpressure <= minpressure` or `raw_adc < minpressure`, `bar = 0.0` and `psi = 0.0`.
  - Otherwise:
    ```
    bar = (raw_adc - minpressure) * maxpressure_bar / (maxpressure - minpressure)
    psi = bar * 14.503768
    ```
- **Blended Spunding Guidance (`Spunding.kt`)**:
  - `iSpindlePlotter` computes three pressure-fermentation readiness gates from `OG`, `current`, and `predictedFg`:
    1. **Hold** (`current > psi10Sg`, where `psi10Sg = OG - max(0.15 * (OG - FG), 0.005)`): fermentation is not yet vigorous; early pressure suppresses yeast growth. Leave relief valve open (`~0 psi` / `0.0 bar`).
    2. **~10 psi / 0.69 bar** (`psi15Sg < current <= psi10Sg`): 15% of expected attenuation is complete (`>= 5 mSG` drop); safe to apply ~10 psi.
    3. **~15 psi / 1.03 bar** (`current <= psi15Sg`, where `psi15Sg = OG - 0.90 * (OG - FG)`, or phase in `{Conditioning, Clarifying, ColdCrash}`): near terminal gravity; bung up to ~15 psi so CO₂ produced in the final 10% of attenuation carbonates the beer in-vessel and stays in solution through cold crash.
    4. **Stuck** (`Phase.Stuck`): spunding guidance is suppressed because pressure further inhibits stalled yeast.
  - When the relay's `A0` pressure sensor is enabled (`pressure_unit` in `{1, 2}`), the dashboard and alarm engine can display both the live vessel pressure and the current phase's recommended spunding target.

---

## 2. Buffering Proxy, Config Push & Remote OTA (`ispindle-proxy` Blend)

Incorporating `~/Code/iSpindlePlotter/proxy/ispindle-proxy.go` directly into `iSpindleStation` turns the ESP8266 relay into a self-contained proxy for the Android app and for patched iSpindel hydrometers (`v7.3.4` with `firmware-config-push.patch`):

### Phone Catch-Up & mDNS Discovery

- **mDNS Service**: Advertise `_ispindle-proxy._tcp.local` on port `80` (with TXT record `app=ispindle-proxy`) via `ESP8266mDNS` so `ProxyDiscovery.kt` in the Android app finds the station automatically on the LAN without manual IP entry.
- **Cursor-Based Replay (`GET /readings?since=<cursor>`)**:
  - Each ingested reading receives a monotonically increasing integer `id` (persisted across reboots by scanning the JSONL tail on boot) and a millisecond timestamp `ts` (`epochNow() * 1000ULL` when SNTP is active) plus peer `ip`:
    ```json
    {"id": 42, "ts": 1730889000000, "ip": "192.168.4.2", "payload": {...}}
    ```
  - `GET /readings?since=N` streams all buffered records with `id > N` and returns `{"cursor": M, "readings": [...]}` so the Android app can sleep or leave the house and catch up on every reading when it reconnects.
  - Existing per-slot raw streams (`/data/0.jsonl`..`/data/2.jsonl`) remain available for direct browser/CLI inspection.

### Piggybacked iSpindel Configuration & Remote OTA (`GET/PUT/DELETE /cfg`, `GET /fw`, `GET /fw.bin`)

When an iSpindel POSTs to `/gravity`, it reads the HTTP response body before entering deep sleep. By attaching a `"cfg"` object to the `{"status":"ok"}` response, `iSpindleStation` reconfigures or updates the hydrometer without zero-touch radio overhead:

```json
{"status":"ok","cfg":{"POLY":"0.95683653 + 0.00156709*tilt + 0.00000376*tilt^2","sleep":900}}
```

- **Sticky fields** (persisted in `/cfg.json` on flash until cleared via `DELETE /cfg`):
  - `POLY`: TinyExpr gravity polynomial (`<= 250` printable ASCII chars, must reference `tilt`). Used when the brewer fits a new degree 1-3 calibration curve in the app or web UI and saves it to the hydrometer.
  - `sleep`: reporting interval in seconds (`10 < sleep < 86400`).
- **One-shot WiFi credential ladder (`SSID` + `PSK`)**:
  - Tried by the iSpindel as *pending* credentials on subsequent wakes before falling back to saved credentials; promoted on the first successful POST and reverted after 3 failed wakes.
  - Retired by `iSpindleStation` as soon as an incoming POST includes a `"cf"` acknowledgment (`"trial"`, `"reverted"`, or `"refused"`), or after `5` unacknowledged deliveries (`maxWifiDeliveries = 5`).
- **One-shot iSpindel OTA firmware push (`fw: {"url", "md5", "ver"}`)**:
  - Instructs the iSpindel to download a new firmware image during the current wake, verify its 32-char hex MD5 and version (`ver != running FIRMWAREVERSION`, `RSSI >= -75 dBm`), and reboot into the new image.
  - Retired on any POST carrying an `"fa"` acknowledgment (`"err"` or `"skip"`) or after `5` deliveries (`maxFwDeliveries = 5`).

---

## 3. Fermentation Analytics: Stock Firmware vs. iSpindlePlotter Pipeline

`iSpindleStation` bridges both analytics models: lightweight on-device metrics for the GC9A01 TFT, buzzer alarms, MQTT, and Nautilis Tapboard, paired with the full `iSpindlePlotter` analytical engine in the browser UI.

### On-Device Metrics

1. **Calibration Offsets**:
   - `corrected_temp = raw_temp + Tempkorekcija1` (or `Tempkorekcija2`)
   - `corrected_gravity = raw_gravity + SGkorekcija1` (or `SGkorekcija2`)
2. **Battery Percentage & Health**:
   - Stock percentage formula (`3.30 V` = `0%`, `4.20 V` = `100%`):
     ```
     bat_pct = clamp(((battery_v - 3.3) / 0.9) * 100.0, 0.0, 100.0)
     ```
   - Browser chart adds linear-regression discharge smoothing, `3.0..4.2 V` lithium-ion cell zones (`Full >= 3.70 V`, `Low 3.45..3.70 V`, `Critical < 3.45 V`), and projected battery runway.
3. **Original Gravity (OG), ABV & Apparent Attenuation**:
   - Auto-locks `OG` on the 5th valid sample (`sample_count == 5`) unless overridden manually via `OG1`/`OG2` or reset via `/new_fermentation?ispindel=1|2`.
   - Standard formulas:
     ```
     ABV         = (OG - corrected_gravity) * 131.25
     Attenuation = (((OG - 1.0) - (corrected_gravity - 1.0)) / (OG - 1.0)) * 100.0
     ```

### Rich Browser-Side Analytics (`iSpindlePlotter/analysis`)

1. **Episode Segmentation (`FermentSegmenter.kt`) & Bad-Data Exclusions (`Exclusions.kt`)**:
   - Automatically splits continuous device logs into distinct `FermentSegment` episodes on time gaps `> 8 h` (while keeping cold-conditioning attached across logging gaps) or single-step SG rises `> 5 mSG` that exceed the prior 6-hour maximum.
   - Rejects initial float drop-in settling spikes before qualifying a segment (`duration >= 6 h`, `max - min SG >= 5 mSG`, downward median trend).
   - Allows the user to mark bad-data time ranges (`ExclusionRange`) to exclude from plots and model fits, or delete false-positive segments.
2. **Krausen-Aware Robust OG (`SeriesClean.robustOg`)**:
   - Computes OG over MAD-filtered inliers.
   - If a `Lag` plateau is detected and the raw maximum sits `> 1.5 mSG` (`KRAUSEN_OG_MARGIN`) above the settled lag median, takes the lag plateau median as the true OG and clamps early krausen false-rise points down to `OG` before fitting the Gompertz curve.
3. **Bayesian Modified-Gompertz & Biphasic Two-Component Fitter (`AttenuationFit.kt`, `TwoComponentAttenuationFit.kt`, `AttenuationModelSelector.kt`)**:
   - Primary curve: 4-parameter modified Gompertz ([Zwietering et al. 1990](https://doi.org/10.1128/aem.56.6.1875-1881.1990)):
     ```
     SG(t) = OG - (OG - FG) * exp(-exp((mu_max * e / (OG - FG)) * (lambda - t) + 1))
     ```
   - Solved by Levenberg-Marquardt with a soft Gaussian prior on attenuation (`atten ~ N(0.75, 0.10²)`) and a posterior FG uncertainty floor scaled by the unobserved fraction of the descent.
   - **Biphasic Promotion**: When `PlateauDetector` finds a mid-ferment diauxic pause with sufficient post-pause data and a decisive BIC improvement, `AttenuationModelSelector` promotes the fit to a 7-parameter two-component Gompertz curve.
   - **Laplace Credible Bands**: Samples from the converged Hessian covariance `Σ = (J^T J)^-1 * σ²` (fixed seed `0x5E1ED`) to render 50% and 95% credible bands on the SG curve and 2.5%..97.5% credible intervals on ETA to terminal gravity (`predictedFg + 0.001`).
4. **Diauxic Plateau Detector (`PlateauDetector.kt`)**:
   - Rolling-window slope test (`|dSG/dt| <= 0.3 mSG/h` for `>= 2.5 h`) classifying flat segments as `Lag`, `Mid` (diauxic shift as yeast switches from glucose/maltose to maltotriose, including pauses that run directly into a cold crash), or `Tail`.
   - Reports plateau level using the **median** (not mean) so brief foam/temp blips do not skew the pause SG.
5. **7-Phase Monotonic State Machine (`Fermentation.kt` + `SettlingDetector.kt`)**:
   - Replaces the stock firmware's 3-state counter with a monotonic timeline (`Timeline.phaseAt(t)` supports interactive cursor scrubbing across historical points without threshold flicker):
     - **`Lag`**: `< 3 mSG` drop from OG and no sustained decline (`ACTIVE_CONFIRM_HOURS = 1.0 h` forward median check prevents single-sample drop-in spikes from erasing the lag phase).
     - **`Active`**: `dSG/dt < -0.0005 SG/h`, or `< 50%` attenuated, or `> 12 mSG` (`SLOWING_NEAR_FG_GAP`) above predicted FG. If currently inside a `Mid` plateau (`current` within `2 mSG` of the plateau level), surfaces diauxic pause duration and rousing/temperature guidance.
     - **`Slowing`**: Decelerating (`-0.0005 <= dSG/dt < -0.00008 SG/h`) while in the final `12 mSG` approach to FG (`>= 50%` attenuated), sustained for `>= 3.0 h`.
     - **`Conditioning`**: Theil-Sen tail rate near zero (`|rate| < 0.00012 SG/h` for `>= 6.0 h`) and median tail SG within `3 mSG` of expected FG. Named `Conditioning` rather than `Complete` because yeast is still scrubbing diacetyl/acetaldehyde and finishing trace sugars.
     - **`Clarifying` (`SettlingDetector.kt`)**: Constant-temperature (`10th..90th` percentile spread `< 1.5 °C`) step-down of `1.5..6.0 mSG` between two flat plateaus after `>= 70%` attenuation across an `>= 8 h` warm tail. Represents yeast flocculating out of suspension (`ρ_bulk` dropping toward clarified beer density) prior to any cold crash; reports both `clarifiedSg` and `fgWithYeast`.
     - **`Stuck`**: Rate near zero for `>= 6.0 h` after `> 5 mSG` initial drop, but stalled `> 5 mSG` above the 75%-attenuation prior FG.
     - **`ColdCrash`**: Sustained temperature drop `>= 4.0 °C` below the pre-crash rolling fermentation temperature for `>= 1.5 h` (with `2.0 °C` hysteresis). Locks `fermentSg` to the pre-crash plateau and trims post-crash thermal density artifacts out of the Gompertz fitter input.

---

## 4. Alarms & Audio Buzzer

Evaluated in `loop()` when `buzz_yesno == 2`. Active alarms pulse the `GPIO 12` buzzer at 2000 Hz (1 s every 5 s) and flag the web UI and TFT perimeter arc:

| Config Key | Sensor / Condition | Trigger Rule | Default |
| :--- | :--- | :--- | :--- |
| `alarm_1` / `alarm_2` (int) | iSpindel 1 Temp Low / High | `raw_temp1 != 0 && (corr_temp1 < alarm_1 \|\| corr_temp1 > alarm_2)` | `10` / `30` |
| `alarm_SG1` (float) | iSpindel 1 Gravity Low | `raw_grav1 != 0 && corr_grav1 < alarm_SG1` (sets Tapboard `alarm1 = 1`) | `1.000` |
| `alarm_5` / `alarm_6` (int) | iSpindel 2 Temp Low / High | `ispindel_num == 2 && raw_temp2 != 0 && (corr_temp2 < alarm_5 \|\| corr_temp2 > alarm_6)` | `10` / `30` |
| `alarm_SG2` (float) | iSpindel 2 Gravity Low | `ispindel_num == 2 && raw_grav2 != 0 && corr_grav2 < alarm_SG2` (sets `alarm2 = 1`) | `1.000` |
| `alarm_3` / `alarm_4` (int) | Relay DS18B20 Temp Low / High | `temp_yesno == 2 && valid(relay_temp) && (relay_temp < alarm_3 \|\| relay_temp > alarm_4)` | `10` / `30` |
| `pressure_alarm_low` / `high` (float) | Relay Pressure Low / High | `pressure_unit in {1,2} && ((low > 0 && P < low) \|\| (high > 0 && P > high))` | `0.0` / `2.0` |

---

## 5. Cloud & Local Forwarding Services

Stock firmware supports 12 service types (`service = 0..11`, `255` = None) over two channels:
1. **On-receipt iSpindel forwarder**: triggered on `POST /gravity`. (`iSpindleStation` extends `server_address` to accept `host:port` so it can forward to another `ispindle-proxy` on `:9501` as well as port `80`.)
2. **Periodic relay sensor sender**: triggered every `custom_period` minutes (minimum 10 minutes) to upload onboard DS18B20 temperature and `A0` pressure.

| ID | Service | Host : Port | On-Receipt iSpindel | Periodic Relay Sensor |
| :-: | :--- | :--- | :--- | :--- |
| `0` | UbiDots | `industrial.api.ubidots.com:80` | `POST /api/v1.6/devices/<name>?token=<TOKEN>` | `POST /api/v1.6/devices/Nautilis_Thermometer?token=<TOKEN>` with `{"temperature":<T>,"pressure":<P>}` |
| `1` | Brewfather | `log.brewfather.net:80` | `POST <uri>` | `POST <uri_custom>` with `{"name":"Nautilis","temp":<T>,"temp_unit":"C|F","pressure":<P>,"pressure_unit":"BAR|PSI"}` |
| `2` | BrewSpy | `brew-spy.com:80` | `POST <uri>`, injects `"token":"<TOKEN>"` | `POST <uri>` with `{"device":"Nautilis-iRelay","token":"<TOKEN>","temperature":<T>,"pressure":<P>}` |
| `3` | Monitor.Beer | `Monitor.beer:80` | `POST <uri>`, injects `"token":"<TOKEN>"` | None |
| `4` | Grainfather | `community.grainfather.com:80` | `name[0] == '1'` -> `<grainfather_uri1>`; `name[0] == '2'` -> `<grainfather_uri2>` | `POST <uri_custom>` with `{"specific-gravity":0,"temperature":<T>,"unit":"celsius|fahrenheit"}` |
| `5` | BrewPiless | `<server_address>:80` | `POST /gravity`, rewrites `"name":"iSpindel000"` | None |
| `6` | Little Bock | `www.littlebock.fr:80` | `name[0] == '1'` -> `<littlebock_uri1>`; `name[0] == '2'` -> `<littlebock_uri2>` | None |
| `7` | Brewers Friend | `log.brewersfriend.com:80` | `POST <uri>` | None |
| `8` | BrewBench | `sensor.brewbench.co:80` | `POST <uri><TOKEN>` | None |
| `9` | MQTT (HA) | `<uri>:1883` | Publishes state topics + 17 Home Assistant discovery payloads | Publishes `<ap_ssid>/rssi`, `<ap_ssid>/temp`, `<ap_ssid>/pressure` |
| `10` | HTTP | `<server_address>[:port]` | `POST <uri>`, injects `"token":"<TOKEN>"` when non-empty | `POST <uri_custom>` with `{"name":"Nautilis_Thermometer","token":"<TOKEN>","temperature":<T>,"temp_units":"C|F","pressure":<P>}` |
| `11` | NAUTILIS Tapboard | `tapboard.nautilis.eu:80` | `POST /receive.php?token=<TOKEN>` with OG/ABV/atten/alarms; reboots after 50 consecutive failures | Same endpoint with `"name":"Nautilis_Relay"` |

### Home Assistant MQTT Auto-Discovery

On MQTT connect, publishes 17 retained discovery configs under `homeassistant/sensor/<ap_ssid>/<sensor_id>/config` sharing device identifier `<ap_ssid>`, plus retained state topics (QoS 1):
- Relay: `<ap_ssid>/rssi`, `<ap_ssid>/temp`, `<ap_ssid>/pressure`
- iSpindel 1: `<ap_ssid>/I1_rssi`, `<ap_ssid>/I1_temp`, `<ap_ssid>/I1_gravity`, `<ap_ssid>/I1_bat`, `<ap_ssid>/I1_angle`, `<ap_ssid>/I1_interval`, `<ap_ssid>/I1_OG1`
- iSpindel 2: `<ap_ssid>/I2_rssi`, `<ap_ssid>/I2_temp`, `<ap_ssid>/I2_gravity`, `<ap_ssid>/I2_bat`, `<ap_ssid>/I2_angle`, `<ap_ssid>/I2_interval`, `<ap_ssid>/I2_OG2`

---

## 6. Web Surface & Configuration Schema

### Blended HTTP Routes

| Route | Origin | Purpose |
| :--- | :--- | :--- |
| `GET /` | Both | Live dashboard, fermentation phase & spunding card, and interactive SG/temp/battery charts |
| `POST /gravity` (and any `POST` path) | Both | Ingest iSpindel JSON, append to slot JSONL and cursor log, return `{"status":"ok"}` (or `{"status":"ok","cfg":{...}}` when a config/OTA push is armed), redraw TFT, and forward on receipt |
| `GET /readings?since=N` | `ispindle-proxy` | Monotonic cursor catch-up feed (`{"cursor":M,"readings":[{id,ts,ip,payload},...]}`) for the `iSpindlePlotter` Android app |
| `GET /cfg`, `PUT /cfg`, `DELETE /cfg` | `ispindle-proxy` | View, arm, or clear the piggybacked iSpindel config delta (`POLY`, `sleep`, one-shot `SSID`+`PSK`, one-shot `fw` OTA) |
| `GET /fw`, `GET /fw.bin` | `ispindle-proxy` | Report `{size, md5}` and stream staged iSpindel firmware binary for remote hydrometer OTA |
| `GET /data/0.jsonl`..`/data/2.jsonl` | `iSpindleStation` | Raw per-slot NDJSON history stream |
| `GET /graph/0.svg`..`/graph/2.svg` | `iSpindleStation` | Server-rendered fallback SVG SG polyline |
| `GET /status`, `GET /info`, `GET /health`, `GET /debug` | Both | Status pages and JSON diagnostics (heap, SPIFFS usage, SNTP sync, slot ages) |
| `GET /network`, `GET /config`, `GET /managment` | Stock Relay | Configuration forms for WiFi/Cloud, Relay Hardware (TFT/Buzzer/DS18B20/Pressure), and per-iSpindel settings |
| `GET /get`, `GET /save_config`, `GET /save_management`, `GET /i_get` | Stock Relay | Merge-save query parameters into `/config.json` and apply runtime changes (`ledState`, `tft_rotate`, buzzer save beep) |
| `GET /new_fermentation?ispindel=1\|2` | Stock Relay | Reset on-device OG lock and fermentation state for slot 1 or 2 |
| `GET /update`, `POST /update` | Stock Relay | Web OTA firmware upload for the relay station itself |
| `GET /reboot` | Both | Schedule `ESP.restart()` |

### `/config.json` Schema (45 Stock Keys Preserved by Merge-Save)

```json
{
  "ssid": "", "password": "",
  "ap_ssid": "nautilus-ispindel", "ap_pass": "1234567890",
  "service": 255, "TOKEN": "", "uri": "/", "server_address": "",
  "grainfather_uri1": "", "grainfather_uri2": "",
  "littlebock_uri1": "", "littlebock_uri2": "",
  "mqtt_user": "", "mqtt_pass": "",
  "custom_period": 15, "uri_custom": "",
  "ispindel_num": 1, "unit_1": 1, "unit_2": 1,
  "SGkorekcija1": 0.0, "SGkorekcija2": 0.0,
  "Tempkorekcija1": 0.0, "Tempkorekcija2": 0.0,
  "OG1": 0.0, "OG2": 0.0,
  "alarm_1": 10, "alarm_2": 30, "alarm_5": 10, "alarm_6": 30,
  "alarm_SG1": 1.000, "alarm_SG2": 1.000,
  "temp_yesno": 1, "temp_unit": 1, "TempCorrection1": 0.0,
  "alarm_3": 10, "alarm_4": 30,
  "pressure_unit": 3, "minpressure": 115, "maxpressure": 334, "maxpressure_bar": 1.52,
  "pressure_alarm_low": 0.0, "pressure_alarm_high": 2.0,
  "buzz_yesno": 2, "ledState": 1, "tft_rotate": 3
}
```
