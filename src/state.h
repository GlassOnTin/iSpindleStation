#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <time.h>

#define TFT_BL_PIN 15
#define BUZZER_PIN 12
#define ONEWIRE_PIN 2
#define PRESSURE_PIN A0

static const char *FIRMWARE_VERSION = "0.4.0";
static const char *CONFIG_PATH = "/config.json";
static const char *CFG_PUSH_PATH = "/cfg_push.json";
static const char *EXCLUSIONS_PATH = "/exclusions.json";
static const char *SPINDLE_FW_PATH = "/spindle_fw.bin";
static const char *SLOT_FILE[3] = {"/0.jsonl", "/1.jsonl", "/2.jsonl"};
static const uint32_t MAX_FILE_BYTES = 120000;

static bool fsMounted = false;
static bool rebootScheduled = false;
static unsigned long rebootScheduledAt = 0;

// ---- All 45 stock /config.json keys ----
struct StationConfig
{
  String ssid;
  String password;
  String apSsid = "nautilus-ispindel";
  String apPass = "1234567890";

  int service = 255; // 0..11, 255 = None
  String token;
  String uri = "/";
  String serverAddress;
  String grainfatherUri1;
  String grainfatherUri2;
  String littlebockUri1;
  String littlebockUri2;
  String mqttUser;
  String mqttPass;
  int customPeriod = 15; // minutes (min 10 for periodic relay sensor upload)
  String uriCustom;

  int ispindelNum = 2; // 1 or 2 (stock key: ispindel_num or i_broj)
  int unit[3] = {1, 1, 1}; // 1 = SG, 2 = Plato (unit_1, unit_2)
  float sgOffset[3] = {0.0f, 0.0f, 0.0f};   // SGkorekcija1, SGkorekcija2
  float tempOffset[3] = {0.0f, 0.0f, 0.0f}; // Tempkorekcija1, Tempkorekcija2
  float og[3] = {0.0f, 0.0f, 0.0f};         // OG1, OG2

  // iSpindel 1 & 2 alarms
  int alarmTempLow[3] = {10, 10, 10};  // alarm_1 (slot 1), alarm_5 (slot 2)
  int alarmTempHigh[3] = {30, 30, 30}; // alarm_2 (slot 1), alarm_6 (slot 2)
  float alarmSgLow[3] = {1.000f, 1.000f, 1.000f}; // alarm_SG1, alarm_SG2

  // Onboard DS18B20 relay thermometer (GPIO 2)
  int tempYesNo = 1; // 1 = OFF, 2 = ON
  int tempUnit = 1;  // 1 = C, 2 = F
  float tempCorrection1 = 0.0f;
  int alarmRelayTempLow = 10;  // alarm_3
  int alarmRelayTempHigh = 30; // alarm_4

  // Onboard analog pressure transducer (A0)
  int pressureUnit = 3; // 1 = Bar, 2 = Psi, 3 = Off
  int minPressure = 115;
  int maxPressure = 334;
  float maxPressureBar = 1.52f;
  float pressureAlarmLow = 0.0f;
  float pressureAlarmHigh = 2.0f;

  // Buzzer, Backlight, TFT Rotation
  int buzzYesNo = 2; // 1 = OFF, 2 = ON
  int ledState = 1;  // 0 = OFF, 1 = ON
  int tftRotate = 3; // 0..3
};

static StationConfig cfg;

// ---- Graph Ring Buffer (3 slots x 120 points x 8 B = 2.88 KB static) ----
static const uint16_t GRAPH_POINTS = 120;
struct GraphPoint
{
  uint32_t ts;
  uint16_t sgTenThou; // (sg - 0.9000) * 10000
  int16_t tempCent;   // temp * 100
};
static GraphPoint graphRing[3][GRAPH_POINTS];
static uint16_t graphCount[3] = {0, 0, 0};
static uint16_t graphNext[3] = {0, 0, 0};

static inline float unpackGraphSg(const GraphPoint &p)
{
  return 0.9000f + (float)p.sgTenThou * 0.0001f;
}
static inline float unpackGraphTemp(const GraphPoint &p)
{
  return (float)p.tempCent * 0.01f;
}
static inline void packGraphPoint(GraphPoint &p, uint32_t ts, float sg, float temp)
{
  p.ts = ts;
  float s = (sg > 1.5f) ? (1.0f + (sg / (258.6f - ((sg / 258.2f) * 227.1f)))) : sg;
  int32_t sv = (int32_t)roundf((s - 0.9000f) * 10000.0f);
  if (sv < 0)
    sv = 0;
  if (sv > 65535)
    sv = 65535;
  p.sgTenThou = (uint16_t)sv;
  p.tempCent = (int16_t)roundf(temp * 100.0f);
}

// ---- Per-Slot Runtime State & Analytics ----
enum FermPhase : uint8_t
{
  PHASE_INSUFFICIENT = 0,
  PHASE_LAG = 1,
  PHASE_ACTIVE = 2,
  PHASE_SLOWING = 3,
  PHASE_CONDITIONING = 4,
  PHASE_CLARIFYING = 5,
  PHASE_STUCK = 6,
  PHASE_COLD_CRASH = 7
};

enum SpundingStage : uint8_t
{
  SPUND_NONE = 0,
  SPUND_HOLD = 1,
  SPUND_10PSI = 2,
  SPUND_15PSI = 3
};

struct SlotState
{
  bool active = false;
  String name;
  uint32_t deviceId = 0;
  float rawSg = 0.0f;
  float corrSg = 0.0f;
  float prevRawSg = 0.0f;
  float rawTemp = 0.0f;
  float corrTemp = 0.0f;
  float batt = 0.0f;
  float batPct = 0.0f;
  float angle = 0.0f;
  int rssi = 0;
  uint32_t interval = 900;
  char tempUnit = 'C';
  unsigned long lastSeenMillis = 0;
  uint32_t lastSeenEpoch = 0;
  String lastIp;

  // Stock fermentation state
  uint16_t sampleCount = 0;
  bool ogLocked = false;
  bool ogAutoDone = false;
  float abv = 0.0f;
  float attenuation = 0.0f;
  uint8_t stockFermState = 0; // 0 = Waiting for OG, 1 = Fermenting, 2 = Complete
  unsigned long lastDropMillis = 0;

  // iSpindlePlotter phase & spunding state
  FermPhase phase = PHASE_INSUFFICIENT;
  float ratePerHour = 0.0f;
  float predictedFg = 1.010f;
  float robustOgVal = 0.0f;
  SpundingStage spundStage = SPUND_NONE;
  float spundTargetSg = 0.0f;

  // Alarm flags
  bool alarmTemp = false;
  bool alarmSg = false;
};

static SlotState slots[3];
static uint32_t globalSeqId = 0;
static bool configDirty = false;

// ---- Onboard Relay Sensor Runtime State ----
struct RelaySensorState
{
  float temp = 127.0f; // 127.0 = disconnected / sensor error sentinel
  uint8_t tempErrorCount = 0;
  bool tempAlarm = false;

  int rawAdc = 0;
  float voltage = 0.0f;
  float bar = 0.0f;
  float psi = 0.0f;
  bool pressureAlarm = false;
};

static RelaySensorState relaySensors;

// ---- Helper Functions ----
static bool sntpActive()
{
  time_t t = time(nullptr);
  return t > 1600000000;
}

static uint32_t epochNow()
{
  if (sntpActive())
    return (uint32_t)time(nullptr);
  return millis() / 1000UL;
}

static uint32_t slotAgeSeconds(int s)
{
  if (!slots[s].active)
    return 0;
  if (slots[s].lastSeenMillis > 0)
    return (uint32_t)((millis() - slots[s].lastSeenMillis) / 1000UL);
  if (sntpActive() && slots[s].lastSeenEpoch > 1600000000UL)
  {
    uint32_t now = (uint32_t)time(nullptr);
    if (now >= slots[s].lastSeenEpoch)
      return now - slots[s].lastSeenEpoch;
  }
  return (uint32_t)(millis() / 1000UL);
}

// Convert SG (1.000..1.150) to Plato (°P), or return as-is if already in Plato
static float sgToPlato(float v)
{
  if (v <= 0.5f)
    return 0.0f;
  if (v > 1.5f)
    return v; // already Plato
  return -616.868f + 1111.14f * v - 630.272f * v * v + 135.997f * v * v * v;
}

// Ensure value is SG (1.000..1.150) even if hydrometer sent Plato
static float ensureSg(float v)
{
  if (v <= 0.5f)
    return 0.0f;
  if (v <= 1.5f)
    return v;
  return 1.0f + (v / (258.6f - ((v / 258.2f) * 227.1f)));
}

static const char *phaseLabel(FermPhase p)
{
  switch (p)
  {
  case PHASE_LAG:
    return "Lag";
  case PHASE_ACTIVE:
    return "Active";
  case PHASE_SLOWING:
    return "Slowing";
  case PHASE_CONDITIONING:
    return "Conditioning";
  case PHASE_CLARIFYING:
    return "Clarifying";
  case PHASE_STUCK:
    return "Stuck";
  case PHASE_COLD_CRASH:
    return "Cold Crash";
  default:
    return "Waiting";
  }
}

static const char *stockFermLabel(uint8_t st)
{
  switch (st)
  {
  case 1:
    return "Fermenting";
  case 2:
    return "Fermentation complete";
  default:
    return "Waiting for OG";
  }
}

// ---- Spunding & On-Device Phase Classifier (ported from iSpindlePlotter) ----
static void updateSlotAnalytics(int s)
{
  SlotState &st = slots[s];
  float sgCorr = st.rawSg + cfg.sgOffset[s];
  float tempCorr = st.rawTemp + cfg.tempOffset[s];
  st.corrSg = sgCorr;
  st.corrTemp = tempCorr;

  // Battery percentage: ((V - 3.3) / 0.9) * 100, clamped [0, 100]
  float pct = ((st.batt - 3.3f) / 0.9f) * 100.0f;
  if (pct < 0.0f)
    pct = 0.0f;
  if (pct > 100.0f)
    pct = 100.0f;
  st.batPct = pct;

  float sgNorm = ensureSg(sgCorr);

  // Stock 5th-sample OG auto-lock (for slots 1 and 2, and slot 0 fallback)
  if (!st.ogAutoDone && sgNorm > 0.980f)
  {
    st.sampleCount++;
    if (st.sampleCount >= 5)
    {
      cfg.og[s] = sgCorr;
      st.ogLocked = true;
      st.ogAutoDone = true;
      configDirty = true;
      Serial.printf("OG%d locked (5th sample): %.4f\n", s, cfg.og[s]);
    }
  }

  float ogNorm = ensureSg(cfg.og[s]);
  if (st.ogLocked && ogNorm > 1.000f && sgNorm > 0.980f)
  {
    st.abv = (ogNorm - sgNorm) * 131.25f;
    if (st.abv < 0.0f)
      st.abv = 0.0f;
    st.attenuation = (((ogNorm - 1.0f) - (sgNorm - 1.0f)) / (ogNorm - 1.0f)) * 100.0f;
    if (st.attenuation < 0.0f)
      st.attenuation = 0.0f;

    // 3-state fermentation counter
    if (st.prevRawSg > 0.5f && (st.prevRawSg - st.rawSg) > 0.001f)
    {
      st.stockFermState = 1; // Fermenting
      st.lastDropMillis = millis();
    }
    else if (st.stockFermState == 1 && st.lastDropMillis > 0)
    {
      unsigned long timeoutMs = (unsigned long)st.interval * 24000UL;
      if (millis() - st.lastDropMillis > timeoutMs)
      {
        st.stockFermState = 2; // Fermentation complete
      }
    }
  }
  st.prevRawSg = st.rawSg;

  // iSpindlePlotter phase & Spunding readiness from graphRing[s]
  uint16_t n = graphCount[s];
  if (n < 6)
  {
    st.phase = PHASE_INSUFFICIENT;
    st.spundStage = SPUND_NONE;
    return;
  }

  uint16_t start = (n == GRAPH_POINTS) ? graphNext[s] : 0;
  uint32_t firstTs = graphRing[s][start].ts;
  uint32_t lastTs = graphRing[s][(start + n - 1) % GRAPH_POINTS].ts;
  float spanH = (lastTs > firstTs) ? ((float)(lastTs - firstTs) / 3600.0f) : 0.0f;
  if (spanH < 1.0f)
  {
    st.phase = PHASE_INSUFFICIENT;
    st.spundStage = SPUND_NONE;
    return;
  }

  // Robust OG from early window / max inlier
  float maxSg = 0.0f;
  float maxTemp = -100.0f;
  for (uint16_t i = 0; i < n; i++)
  {
    GraphPoint &gp = graphRing[s][(start + i) % GRAPH_POINTS];
    float g = ensureSg(unpackGraphSg(gp) + cfg.sgOffset[s]);
    float t = unpackGraphTemp(gp);
    if (g > maxSg)
      maxSg = g;
    if (t > maxTemp)
      maxTemp = t;
  }
  float refOg = (st.ogLocked && ogNorm > 1.000f) ? ogNorm : maxSg;
  st.robustOgVal = refOg;

  // OLS slope over last 6 hours
  uint32_t tailCutoff = (lastTs > 21600U) ? (lastTs - 21600U) : firstTs;
  double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
  int tailN = 0;
  for (uint16_t i = 0; i < n; i++)
  {
    GraphPoint &gp = graphRing[s][(start + i) % GRAPH_POINTS];
    if (gp.ts >= tailCutoff)
    {
      double x = (double)(gp.ts - tailCutoff) / 3600.0;
      double y = (double)ensureSg(unpackGraphSg(gp) + cfg.sgOffset[s]);
      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumXX += x * x;
      tailN++;
    }
  }
  float slope = 0.0f;
  if (tailN >= 3)
  {
    double denom = tailN * sumXX - sumX * sumX;
    if ( fabs(denom) > 1e-9 )
      slope = (float)((tailN * sumXY - sumX * sumY) / denom);
  }
  st.ratePerHour = slope;

  // 75% attenuation prior FG
  float priorFg = 1.000f + 0.25f * (refOg - 1.000f);
  if (priorFg < 0.998f)
    priorFg = 0.998f;
  st.predictedFg = priorFg;

  float drop = refOg - sgNorm;
  bool rateNearZero = fabsf(slope) < 0.00012f;

  // Cold crash check (>= 4 C drop from warm peak)
  float curTempC = (st.tempUnit == 'F' || st.tempUnit == 'f')
                       ? ((st.corrTemp - 32.0f) * 5.0f / 9.0f)
                       : st.corrTemp;
  float maxTempC = (st.tempUnit == 'F' || st.tempUnit == 'f')
                       ? ((maxTemp - 32.0f) * 5.0f / 9.0f)
                       : maxTemp;

  if (maxTemp > 0.0f && (maxTempC - curTempC) >= 4.0f && drop > 0.005f)
  {
    st.phase = PHASE_COLD_CRASH;
  }
  else if (drop < 0.003f && slope > -0.00008f)
  {
    st.phase = PHASE_LAG;
  }
  else if (rateNearZero && drop > 0.005f && sgNorm <= priorFg + 0.003f)
  {
    st.phase = PHASE_CONDITIONING;
  }
  else if (rateNearZero && drop > 0.005f && sgNorm > priorFg + 0.005f)
  {
    st.phase = PHASE_STUCK;
  }
  else if (slope < -0.0005f || (drop / fmaxf(refOg - priorFg, 0.001f)) < 0.50f ||
           (sgNorm - priorFg) > 0.012f)
  {
    st.phase = PHASE_ACTIVE;
  }
  else
  {
    st.phase = PHASE_SLOWING;
  }

  // Spunding evaluation (ported from Spunding.kt)
  if (st.phase == PHASE_INSUFFICIENT || st.phase == PHASE_STUCK || refOg <= 1.000f || priorFg >= refOg)
  {
    st.spundStage = SPUND_NONE;
    st.spundTargetSg = 0.0f;
  }
  else if (st.phase == PHASE_CONDITIONING || st.phase == PHASE_CLARIFYING || st.phase == PHASE_COLD_CRASH)
  {
    st.spundStage = SPUND_15PSI;
    st.spundTargetSg = priorFg;
  }
  else
  {
    float target15 = refOg - 0.90f * (refOg - priorFg);
    float drop10 = fmaxf(0.15f * (refOg - priorFg), 0.005f);
    float target10 = fmaxf(refOg - drop10, target15);
    if (sgNorm <= target15)
    {
      st.spundStage = SPUND_15PSI;
      st.spundTargetSg = target15;
    }
    else if (sgNorm <= target10)
    {
      st.spundStage = SPUND_10PSI;
      st.spundTargetSg = target15;
    }
    else
    {
      st.spundStage = SPUND_HOLD;
      st.spundTargetSg = target10;
    }
  }
}

// ---- Config Load & Save ----
static void defaultConfig()
{
  cfg = StationConfig();
}

static void loadConfig()
{
  defaultConfig();
  if (!SPIFFS.exists(CONFIG_PATH))
    return;
  File f = SPIFFS.open(CONFIG_PATH, "r");
  if (!f)
    return;
  DynamicJsonDocument doc(3072);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err)
  {
    Serial.printf("config.json unreadable: %s\n", err.c_str());
    return;
  }
  cfg.ssid = doc["ssid"] | "";
  cfg.password = doc["password"] | "";
  cfg.apSsid = doc["ap_ssid"] | cfg.apSsid;
  cfg.apPass = doc["ap_pass"] | cfg.apPass;
  cfg.service = doc["service"] | 255;
  cfg.token = doc["TOKEN"] | "";
  cfg.uri = doc["uri"] | "/";
  cfg.serverAddress = doc["server_address"] | "";
  cfg.grainfatherUri1 = doc["grainfather_uri1"] | "";
  cfg.grainfatherUri2 = doc["grainfather_uri2"] | "";
  cfg.littlebockUri1 = doc["littlebock_uri1"] | "";
  cfg.littlebockUri2 = doc["littlebock_uri2"] | "";
  cfg.mqttUser = doc["mqtt_user"] | "";
  cfg.mqttPass = doc["mqtt_pass"] | "";
  cfg.customPeriod = doc["custom_period"] | 15;
  cfg.uriCustom = doc["uri_custom"] | "";

  if (doc.containsKey("ispindel_num"))
    cfg.ispindelNum = doc["ispindel_num"] | 2;
  else if (doc.containsKey("i_broj"))
    cfg.ispindelNum = doc["i_broj"] | 2;

  cfg.unit[1] = doc["unit_1"] | 1;
  cfg.unit[2] = doc["unit_2"] | 1;
  cfg.sgOffset[1] = doc["SGkorekcija1"] | 0.0f;
  cfg.sgOffset[2] = doc["SGkorekcija2"] | 0.0f;
  cfg.tempOffset[1] = doc["Tempkorekcija1"] | 0.0f;
  cfg.tempOffset[2] = doc["Tempkorekcija2"] | 0.0f;
  cfg.og[1] = doc["OG1"] | 0.0f;
  cfg.og[2] = doc["OG2"] | 0.0f;

  for (int s = 1; s <= 2; s++)
  {
    if (cfg.og[s] > 0.0f)
    {
      slots[s].ogLocked = true;
      slots[s].ogAutoDone = true;
      slots[s].sampleCount = 5;
    }
  }

  cfg.alarmTempLow[1] = doc["alarm_1"] | 10;
  cfg.alarmTempHigh[1] = doc["alarm_2"] | 30;
  cfg.alarmTempLow[2] = doc["alarm_5"] | 10;
  cfg.alarmTempHigh[2] = doc["alarm_6"] | 30;
  cfg.alarmSgLow[1] = doc["alarm_SG1"] | 1.000f;
  cfg.alarmSgLow[2] = doc["alarm_SG2"] | 1.000f;

  cfg.tempYesNo = doc["temp_yesno"] | 1;
  cfg.tempUnit = doc["temp_unit"] | 1;
  cfg.tempCorrection1 = doc["TempCorrection1"] | 0.0f;
  cfg.alarmRelayTempLow = doc["alarm_3"] | 10;
  cfg.alarmRelayTempHigh = doc["alarm_4"] | 30;

  cfg.pressureUnit = doc["pressure_unit"] | 3;
  cfg.minPressure = doc["minpressure"] | 115;
  cfg.maxPressure = doc["maxpressure"] | 334;
  cfg.maxPressureBar = doc["maxpressure_bar"] | 1.52f;
  cfg.pressureAlarmLow = doc["pressure_alarm_low"] | 0.0f;
  cfg.pressureAlarmHigh = doc["pressure_alarm_high"] | ((cfg.pressureUnit == 2) ? 29.0f : 2.0f);
  // Guard against unscaled 2.00 Bar default left over when switching to Psi
  if (cfg.pressureUnit == 2 && fabsf(cfg.pressureAlarmHigh - 2.0f) < 0.01f)
    cfg.pressureAlarmHigh = 29.0f;

  cfg.buzzYesNo = doc["buzz_yesno"] | 2;
  cfg.ledState = doc["ledState"] | 1;
  cfg.tftRotate = doc["tft_rotate"] | 3;
}

static bool saveConfigToDisk()
{
  DynamicJsonDocument doc(3072);
  if (SPIFFS.exists(CONFIG_PATH))
  {
    File f = SPIFFS.open(CONFIG_PATH, "r");
    if (f)
    {
      deserializeJson(doc, f);
      f.close();
    }
  }
  doc["ssid"] = cfg.ssid;
  doc["password"] = cfg.password;
  doc["ap_ssid"] = cfg.apSsid;
  doc["ap_pass"] = cfg.apPass;
  doc["service"] = cfg.service;
  doc["TOKEN"] = cfg.token;
  doc["uri"] = cfg.uri;
  doc["server_address"] = cfg.serverAddress;
  doc["grainfather_uri1"] = cfg.grainfatherUri1;
  doc["grainfather_uri2"] = cfg.grainfatherUri2;
  doc["littlebock_uri1"] = cfg.littlebockUri1;
  doc["littlebock_uri2"] = cfg.littlebockUri2;
  doc["mqtt_user"] = cfg.mqttUser;
  doc["mqtt_pass"] = cfg.mqttPass;
  doc["custom_period"] = cfg.customPeriod;
  doc["uri_custom"] = cfg.uriCustom;
  doc["ispindel_num"] = cfg.ispindelNum;
  doc["unit_1"] = cfg.unit[1];
  doc["unit_2"] = cfg.unit[2];
  doc["SGkorekcija1"] = cfg.sgOffset[1];
  doc["SGkorekcija2"] = cfg.sgOffset[2];
  doc["Tempkorekcija1"] = cfg.tempOffset[1];
  doc["Tempkorekcija2"] = cfg.tempOffset[2];
  doc["OG1"] = cfg.og[1];
  doc["OG2"] = cfg.og[2];
  doc["alarm_1"] = cfg.alarmTempLow[1];
  doc["alarm_2"] = cfg.alarmTempHigh[1];
  doc["alarm_5"] = cfg.alarmTempLow[2];
  doc["alarm_6"] = cfg.alarmTempHigh[2];
  doc["alarm_SG1"] = cfg.alarmSgLow[1];
  doc["alarm_SG2"] = cfg.alarmSgLow[2];
  doc["temp_yesno"] = cfg.tempYesNo;
  doc["temp_unit"] = cfg.tempUnit;
  doc["TempCorrection1"] = cfg.tempCorrection1;
  doc["alarm_3"] = cfg.alarmRelayTempLow;
  doc["alarm_4"] = cfg.alarmRelayTempHigh;
  doc["pressure_unit"] = cfg.pressureUnit;
  doc["minpressure"] = cfg.minPressure;
  doc["maxpressure"] = cfg.maxPressure;
  doc["maxpressure_bar"] = cfg.maxPressureBar;
  doc["pressure_alarm_low"] = cfg.pressureAlarmLow;
  doc["pressure_alarm_high"] = cfg.pressureAlarmHigh;
  doc["buzz_yesno"] = cfg.buzzYesNo;
  doc["ledState"] = cfg.ledState;
  doc["tft_rotate"] = cfg.tftRotate;

  File out = SPIFFS.open(CONFIG_PATH, "w");
  if (!out)
    return false;
  serializeJson(doc, out);
  out.close();
  configDirty = false;
  return true;
}
