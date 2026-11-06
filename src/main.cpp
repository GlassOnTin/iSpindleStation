// iSpindleStation v0.4.0 — Unified stock Nautilis iRelay + iSpindlePlotter firmware.
//
// Combines:
// - GC9A01 240x240 round TFT display (HSPI SCK=14, MOSI=13, CS=0, DC=4, RST=5, BL=15)
// - Piezo Buzzer (GPIO 12: 1800 Hz save beep, 2000 Hz alarm beep)
// - Onboard DS18B20 thermometer (GPIO 2: 11-bit non-blocking, 10 s poll, 6-strike debounce)
// - Onboard analog pressure sensor (A0: 15 s poll, linear Bar/Psi calibration)
// - ispindle-proxy compatibility: mDNS (_ispindle-proxy._tcp), GET /readings?since=N,
//   and piggybacked POST /gravity config/OTA push (GET/PUT/DELETE /cfg, /fw, /fw.bin)
// - On-device & browser-side iSpindlePlotter analytics (Bayesian Gompertz fit,
//   episode segmentation, diauxic plateau detection, 7-phase state machine, and
//   spunding-pressure readiness paired with live A0 vessel pressure)
// - All 12 stock cloud/local forwarding services + 17 Home Assistant MQTT
//   Auto-Discovery sensors + periodic relay sensor uploads.

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <time.h>
#include <WiFiManager.h>

#include "state.h"
#include "sensors.h"
#include "display.h"
#include "proxy.h"
#include "forwarders.h"
#include "webui.h"

ESP8266WebServer server(80);
static DNSServer dnsServer;

static int slotForName(const String &name)
{
  if (name.startsWith("1"))
    return 1;
  if (name.startsWith("2"))
    return 2;
  if (slots[1].active && slots[1].name == name)
    return 1;
  if (slots[2].active && slots[2].name == name)
    return 2;
  if (!slots[1].active)
    return 1;
  if (!slots[2].active && cfg.ispindelNum >= 2)
    return 2;
  return 0;
}

static void pushGraphPoint(int slot, uint32_t ts, float sg, float temp)
{
  GraphPoint &p = graphRing[slot][graphNext[slot]];
  packGraphPoint(p, ts, sg, temp);
  graphNext[slot] = (graphNext[slot] + 1) % GRAPH_POINTS;
  if (graphCount[slot] < GRAPH_POINTS)
    graphCount[slot]++;
}

static void loadHistoricalSlots()
{
  DynamicJsonDocument doc(1024);
  uint32_t maxSeq = 0;
  for (int s = 0; s < 3; s++)
  {
    if (!SPIFFS.exists(SLOT_FILE[s]))
      continue;
    File f = SPIFFS.open(SLOT_FILE[s], "r");
    if (!f)
      continue;
    while (f.available())
    {
      String line = f.readStringUntil('\n');
      line.trim();
      if (line.isEmpty())
        continue;
      if (deserializeJson(doc, line) == DeserializationError::Ok)
      {
        uint32_t id = doc["id"] | 0U;
        if (id > maxSeq)
          maxSeq = id;
        else
          maxSeq++;
        uint32_t ts = doc["ts"] | 0U;
        JsonObject payload = doc["payload"];
        if (!payload.isNull())
        {
          float sg = payload["gravity"] | 0.0f;
          float temp = payload["temperature"] | 0.0f;
          if (sg > 0.5f)
            pushGraphPoint(s, ts, sg, temp);
          SlotState &st = slots[s];
          st.active = true;
          st.name = payload["name"] | (String("Slot ") + String(s));
          st.deviceId = payload["ID"] | 0U;
          st.rawSg = sg;
          st.rawTemp = temp;
          st.batt = payload["battery"] | 0.0f;
          st.angle = payload["angle"] | 0.0f;
          st.rssi = payload["RSSI"] | 0;
          uint32_t iv = payload["interval"] | 900U;
          st.interval = iv > 0 ? iv : 900U;
          const char *tu = payload["temp_units"] | "C";
          st.tempUnit = (tu && tu[0]) ? tu[0] : 'C';
          st.lastSeenEpoch = ts;
          updateSlotAnalytics(s);
        }
      }
      yield();
    }
    f.close();
  }
  globalSeqId = maxSeq;
  if (configDirty)
    saveConfigToDisk();
}

static bool appendJsonl(const char *path, const String &line)
{
  File out = SPIFFS.open(path, "a");
  if (!out)
    return false;
  out.write((const uint8_t *)line.c_str(), line.length());
  out.write((const uint8_t *)"\n", 1);
  uint32_t size = out.size();
  out.close();
  if (size <= MAX_FILE_BYTES)
    return true;

  File src = SPIFFS.open(path, "r");
  if (!src)
    return false;
  src.seek(size / 2, SeekSet);
  while (src.available() && src.read() != '\n')
  {
  }
  String tmpPath = String(path) + ".tmp";
  File dst = SPIFFS.open(tmpPath, "w");
  if (!dst)
  {
    src.close();
    return false;
  }
  uint8_t buf[256];
  size_t got;
  while ((got = src.read(buf, sizeof(buf))) > 0)
    dst.write(buf, got);
  src.close();
  dst.close();
  return SPIFFS.remove(path) && SPIFFS.rename(tmpPath, path);
}

static void handleConfigSave()
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
  const char *textKeys[] = {
      "ssid", "password", "ap_ssid", "ap_pass", "TOKEN", "server_address",
      "uri", "grainfather_uri1", "grainfather_uri2", "littlebock_uri1",
      "littlebock_uri2", "mqtt_user", "mqtt_pass", "uri_custom"};
  for (const char *k : textKeys)
    if (server.hasArg(k))
      doc[k] = server.arg(k);

  const char *intKeys[] = {
      "service", "custom_period", "ispindel_num", "i_broj", "unit_1", "unit_2",
      "pressure_unit", "ledState", "tft_rotate", "buzz_yesno", "temp_yesno",
      "temp_unit", "alarm_1", "alarm_2", "alarm_3", "alarm_4", "alarm_5",
      "alarm_6", "minpressure", "maxpressure"};
  for (const char *k : intKeys)
    if (server.hasArg(k))
      doc[k] = server.arg(k).toInt();

  if (server.hasArg("i_broj") && !server.hasArg("ispindel_num"))
    doc["ispindel_num"] = server.arg("i_broj").toInt();

  const char *floatKeys[] = {
      "alarm_SG1", "alarm_SG2", "SGkorekcija1", "SGkorekcija2",
      "Tempkorekcija1", "Tempkorekcija2", "OG1", "OG2", "maxpressure_bar",
      "pressure_alarm_low", "pressure_alarm_high", "TempCorrection1"};
  for (const char *k : floatKeys)
    if (server.hasArg(k))
      doc[k] = server.arg(k).toFloat();

  File out = SPIFFS.open(CONFIG_PATH, "w");
  if (!out)
  {
    server.send(500, "text/plain", "config write failed");
    return;
  }
  serializeJson(doc, out);
  out.close();

  int prevRot = cfg.tftRotate;
  loadConfig();

  // Handle manual OG1 / OG2 override vs zeroing
  for (int s = 1; s <= 2; s++)
  {
    String ogKey = (s == 1) ? "OG1" : "OG2";
    if (server.hasArg(ogKey))
    {
      if (cfg.og[s] > 0.0f)
      {
        slots[s].ogLocked = true;
        slots[s].ogAutoDone = true;
        slots[s].sampleCount = 5;
      }
      else
      {
        slots[s].ogLocked = false;
        slots[s].ogAutoDone = false;
        slots[s].sampleCount = 0;
        slots[s].abv = 0.0f;
        slots[s].attenuation = 0.0f;
        slots[s].stockFermState = 0;
      }
    }
    if (slots[s].active)
      updateSlotAnalytics(s);
  }

  digitalWrite(TFT_BL_PIN, cfg.ledState ? HIGH : LOW);
  if (cfg.tftRotate != prevRot)
    tft.setRotation(cfg.tftRotate % 4);

  beepSettingsSaved();
  evaluateAlarms();
  drawDisplay(true);

  server.send(200, "text/html",
              "<!DOCTYPE html><html><head><meta charset='utf-8'><meta http-equiv='refresh' content='1;url=/'>"
              "<link rel='stylesheet' href='/style.css'></head><body style='text-align:center;padding:40px'>"
              "<h2>Settings saved.</h2><p><a href='/' style='color:#38dbff'>Return to Dashboard</a></p></body></html>");
}

static void handleNewFermentation()
{
  int sp = 1;
  if (server.hasArg("ispindel"))
    sp = server.arg("ispindel").toInt();
  else if (server.hasArg("isp"))
    sp = server.arg("isp").toInt();
  if (sp < 1 || sp > 2)
    sp = 1;

  cfg.og[sp] = 0.0f;
  slots[sp].ogLocked = false;
  slots[sp].ogAutoDone = false;
  slots[sp].sampleCount = 0;
  slots[sp].abv = 0.0f;
  slots[sp].attenuation = 0.0f;
  slots[sp].stockFermState = 0;
  slots[sp].lastDropMillis = 0;
  graphCount[sp] = 0;
  graphNext[sp] = 0;
  saveConfigToDisk();
  beepSettingsSaved();
  drawDisplay(true);

  server.sendHeader("Location", "/managment", true);
  server.send(302, "text/plain", "Fermentation reset");
}

static void handleReceive()
{
  String body = server.arg("plain");
  if (body.length() == 0 || body.length() > 2048)
  {
    server.send(400, "application/json", "{\"error\":\"bad body\"}");
    return;
  }
  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, body);
  if (err || !doc.is<JsonObject>())
  {
    server.send(400, "application/json", "{\"error\":\"bad json\"}");
    return;
  }

  // Check for piggybacked config/OTA acks from patched iSpindel 7.3.4 firmware
  String cf = doc["cf"] | "";
  String fa = doc["fa"] | "";
  if (!cf.isEmpty() || !fa.isEmpty())
    noteSpindleAcks(cf, fa);

  String name = doc["name"] | "unknown";
  int slot = slotForName(name);
  float sg = doc["gravity"] | 0.0f;
  float temp = doc["temperature"] | 0.0f;
  float batt = doc["battery"] | 0.0f;
  float angle = doc["angle"] | 0.0f;
  int rssi = doc["RSSI"] | 0;
  uint32_t devId = doc["ID"] | 0U;
  uint32_t interval = doc["interval"] | 900U;
  const char *tu = doc["temp_units"] | "C";
  char tempUnit = (tu && tu[0]) ? tu[0] : 'C';

  uint32_t nowTs = epochNow();
  uint32_t seqId = ++globalSeqId;
  String peerIp = server.client().remoteIP().toString();

  String line = "{\"id\":" + String(seqId) + ",\"ts\":" + String(nowTs) +
                ",\"ip\":\"" + peerIp + "\",\"payload\":" + body + "}";
  if (!appendJsonl(SLOT_FILE[slot], line))
  {
    server.send(500, "application/json", "{\"error\":\"write failed\"}");
    return;
  }

  SlotState &st = slots[slot];
  st.active = true;
  st.name = name;
  st.deviceId = devId;
  st.rawSg = sg;
  st.rawTemp = temp;
  st.batt = batt;
  st.angle = angle;
  st.rssi = rssi;
  st.interval = interval > 0 ? interval : 900U;
  st.tempUnit = tempUnit;
  st.lastSeenMillis = millis();
  st.lastSeenEpoch = nowTs;
  st.lastIp = peerIp;

  if (sg > 0.5f)
    pushGraphPoint(slot, nowTs, sg, temp);

  updateSlotAnalytics(slot);
  if (configDirty)
    saveConfigToDisk();
  evaluateAlarms();

  // Reply to iSpindel (attaches {"cfg":{...}} if a push is armed)
  String resp = buildReceiveResponse();
  server.send(200, "application/json", resp);

  drawDisplay(true);
  forwardOnReceipt(slot, body, COL_OK, COL_WARN, COL_SG);
}

static void serveSlot(int s)
{
  const char *path = SLOT_FILE[s];
  if (!SPIFFS.exists(path))
  {
    server.send(200, "application/x-ndjson", "");
    return;
  }
  File f = SPIFFS.open(path, "r");
  server.streamFile(f, "application/x-ndjson");
  f.close();
}

static void serveGraph(int slot)
{
  uint16_t n = graphCount[slot];
  if (n == 0)
  {
    server.send(200, "image/svg+xml", "<svg xmlns='http://www.w3.org/2000/svg'/>");
    return;
  }
  int cfgIdx = (slot == 2) ? 2 : 1;
  uint16_t start = (n == GRAPH_POINTS) ? graphNext[slot] : 0;
  float lo = INFINITY, hi = -INFINITY;
  for (uint16_t i = 0; i < n; i++)
  {
    float v = ensureSg(unpackGraphSg(graphRing[slot][(start + i) % GRAPH_POINTS]) + cfg.sgOffset[cfgIdx]);
    if (v < lo)
      lo = v;
    if (v > hi)
      hi = v;
  }
  const float pad = (hi - lo) < 0.002f ? 0.001f : (hi - lo) * 0.05f;
  lo -= pad;
  hi += pad;
  const int W = 700, H = 300, M = 44;
  String pts;
  if (!pts.reserve(n * 14))
  {
    server.send(503, "text/plain", "not enough memory for graph");
    return;
  }
  for (uint16_t i = 0; i < n; i++)
  {
    GraphPoint &p = graphRing[slot][(start + i) % GRAPH_POINTS];
    float v = ensureSg(unpackGraphSg(p) + cfg.sgOffset[cfgIdx]);
    float x = M + (W - 2 * M) * (n == 1 ? 0.5f : (float)i / (float)(n - 1));
    float y = H - M - (H - 2 * M) * ((v - lo) / (hi - lo));
    if (i)
      pts += ' ';
    pts += String((int)x) + "," + String(y, 1);
  }
  String strokeCol = (slot == 2) ? "#ffea00" : "#00e676";
  String svg = "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 " +
               String(W) + " " + String(H) + "' style='max-width:100%'>"
               "<rect width='" + String(W) + "' height='" + String(H) + "' fill='#14191e'/>"
               "<polyline fill='none' stroke='" + strokeCol + "' stroke-width='2' points='" + pts + "'/>";
  GraphPoint &oldest = graphRing[slot][start];
  GraphPoint &newest = graphRing[slot][(start + n - 1) % GRAPH_POINTS];
  svg += "<text x='" + String(M) + "' y='" + String(H - 12) + "' fill='#8b949e' font-size='12'>";
  svg += "SG " + String(lo + pad, 4) + " .. " + String(hi - pad, 4) +
         " (" + String(n) + " pts, span " + String((uint32_t)(newest.ts - oldest.ts)) + " s)";
  svg += "</text></svg>";
  server.send(200, "image/svg+xml", svg);
}

static void handleHealth()
{
  DynamicJsonDocument doc(512);
  doc["name"] = "iSpindleStation";
  doc["firmware"] = FIRMWARE_VERSION;
  doc["epoch"] = epochNow();
  doc["sntp"] = sntpActive();
  doc["ap"] = WiFi.softAPIP().toString();
  doc["sta"] = WiFi.localIP().toString();
  doc["cursor"] = globalSeqId;
  String j;
  serializeJson(doc, j);
  server.send(200, "application/json", j);
}

static void handleDebug()
{
  DynamicJsonDocument doc(2048);
  doc["firmware"] = FIRMWARE_VERSION;
  doc["fsMounted"] = fsMounted;
  doc["sta"] = WiFi.localIP().toString();
  doc["apClients"] = WiFi.softAPgetStationNum();
  doc["heap"] = ESP.getFreeHeap();
  doc["ledState"] = cfg.ledState;
  doc["tftRotate"] = cfg.tftRotate;
  doc["service"] = cfg.service;
  doc["cursor"] = globalSeqId;
  doc["relayTemp"] = relaySensors.temp;
  doc["relayBar"] = relaySensors.bar;
  doc["relayPsi"] = relaySensors.psi;
  JsonArray arr = doc.createNestedArray("slots");
  for (int s = 0; s < 3; s++)
  {
    JsonObject so = arr.createNestedObject();
    so["slot"] = s;
    so["active"] = slots[s].active;
    so["name"] = slots[s].name;
    so["sg"] = slots[s].corrSg;
    so["temp"] = slots[s].corrTemp;
    so["batt"] = slots[s].batt;
    so["og"] = cfg.og[(s == 2) ? 2 : 1];
    so["abv"] = slots[s].abv;
    so["atten"] = slots[s].attenuation;
    so["phase"] = phaseLabel(slots[s].phase);
    so["ageSec"] = slotAgeSeconds(s);
    so["points"] = graphCount[s];
  }
  String j;
  serializeJson(doc, j);
  server.send(200, "application/json", j);
}

static void handleReboot()
{
  server.send(200, "text/plain", "rebooting in 1s");
  rebootScheduled = true;
  rebootScheduledAt = millis();
}

static void handleFactoryReset()
{
  SPIFFS.remove(CONFIG_PATH);
  SPIFFS.remove(CFG_PUSH_PATH);
  server.send(200, "text/plain", "config reset, rebooting");
  rebootScheduled = true;
  rebootScheduledAt = millis();
}

static void handleNotFound()
{
  if (server.method() == HTTP_POST)
  {
    handleReceive();
    return;
  }
  // Captive portal redirect when accessed over soft-AP while STA is disconnected
  if (WiFi.status() != WL_CONNECTED)
  {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
    return;
  }
  server.send(404, "text/plain", "not found");
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("\niSpindleStation v0.4.0 boot");

  fsMounted = SPIFFS.begin();
  if (!fsMounted)
  {
    Serial.println("SPIFFS mount failed - formatting");
    delay(100);
    SPIFFS.format();
    fsMounted = SPIFFS.begin();
  }

  loadConfig();
  loadCfgPushFromDisk();
  loadHistoricalSlots();

  // Initialize TFT display before WiFi join
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, cfg.ledState ? HIGH : LOW);
  tft.init();
  initSensorsAndBuzzer();
  tft.setRotation(cfg.tftRotate % 4);
  pollRelayTemperature();
  pollRelayPressure();
  drawDisplay(true);

  WiFi.hostname("iSpindleStation");
  bool joined = false;
  if (cfg.ssid.length() > 0)
  {
    WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
    for (int i = 0; i < 25 && !joined; i++)
    {
      delay(1000);
      joined = (WiFi.status() == WL_CONNECTED);
    }
  }
  if (!joined)
  {
    WiFiManager wm;
    wm.setConfigPortalTimeout(300);
    wm.setBreakAfterConfig(true);
    if (!wm.autoConnect("iSpindleStation"))
    {
      delay(2000);
      ESP.restart();
    }
  }
  Serial.printf("joined %s as %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

  if (!cfg.apSsid.isEmpty())
  {
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));
    WiFi.softAP(cfg.apSsid.c_str(), cfg.apPass.c_str());
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));
  }

  configTime(0, 0, "pool.ntp.org");
  initMdnsProxy();

  // Web UI & Stock Relay Routes
  server.on("/", HTTP_GET, handleDashboard);
  server.on("/status", HTTP_GET, handleDashboard);
  server.on("/hidro", HTTP_GET, handleDashboard);
  server.on("/style.css", HTTP_GET, handleStyleCss);
  server.on("/app.js", HTTP_GET, handleAppJs);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/network", HTTP_GET, handleNetworkPage);
  server.on("/config", HTTP_GET, handleConfigPage);
  server.on("/managment", HTTP_GET, handleManagementPage);
  server.on("/info", HTTP_GET, handleInfoPage);
  server.on("/get", HTTP_GET, handleConfigSave);
  server.on("/save_config", HTTP_GET, handleConfigSave);
  server.on("/save_management", HTTP_GET, handleConfigSave);
  server.on("/i_get", HTTP_GET, handleConfigSave);
  server.on("/new_fermentation", HTTP_ANY, handleNewFermentation);
  server.on("/gravity", HTTP_POST, handleReceive);
  server.on("/reboot", HTTP_ANY, handleReboot);
  server.on("/reset", HTTP_POST, handleFactoryReset);
  server.on("/update", HTTP_GET, handleUpdateGet);
  server.on("/update", HTTP_POST, handleUpdatePostDone, handleUpdatePostUpload);

  // ispindle-proxy & iSpindlePlotter Routes
  server.on("/readings", HTTP_GET, handleReadingsSince);
  server.on("/cfg", HTTP_ANY, handleCfgEndpoint);
  server.on("/fw", HTTP_GET, handleFwInfo);
  server.on("/fw.bin", HTTP_GET, handleFwBinGet);
  server.on("/exclusions", HTTP_ANY, handleExclusionsEndpoint);

  // Raw Data, SVG Graphs & Diagnostics
  server.on("/data/0.jsonl", HTTP_GET, []() { serveSlot(0); });
  server.on("/data/1.jsonl", HTTP_GET, []() { serveSlot(1); });
  server.on("/data/2.jsonl", HTTP_GET, []() { serveSlot(2); });
  server.on("/graph/0.svg", HTTP_GET, []() { serveGraph(0); });
  server.on("/graph/1.svg", HTTP_GET, []() { serveGraph(1); });
  server.on("/graph/2.svg", HTTP_GET, []() { serveGraph(2); });
  server.on("/health", HTTP_GET, handleHealth);
  server.on("/debug", HTTP_GET, handleDebug);
  server.on("/redraw", HTTP_GET, []() {
    if (server.hasArg("rot"))
    {
      cfg.tftRotate = server.arg("rot").toInt() % 4;
      tft.setRotation(cfg.tftRotate);
    }
    drawDisplay(true);
    server.send(200, "application/json", "{\"redrawn\":true,\"rot\":" + String(cfg.tftRotate) + "}");
  });
  server.onNotFound(handleNotFound);
  server.begin();

  drawDisplay(true);
}

static unsigned long lastPingMs = 0;

void loop()
{
  dnsServer.processNextRequest();
  server.handleClient();
  MDNS.update();
  updateBuzzerPin();

  if (rebootScheduled && (millis() - rebootScheduledAt) >= 1000UL)
  {
    delay(100);
    ESP.restart();
  }

  pollRelayTemperature();
  pollRelayPressure();
  evaluateAlarms();
  updatePeriodicRelaySender(COL_OK, COL_WARN, COL_SG);

  if (millis() - lastPingMs >= 30000UL)
  {
    lastPingMs = millis();
    Serial.printf("[loop] t=%lu heap=%u apClients=%d\n", millis() / 1000UL,
                  ESP.getFreeHeap(), WiFi.softAPgetStationNum());
    drawDisplay(false);
  }
}
