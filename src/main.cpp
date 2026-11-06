// iSpindleStation v0.3.0 — receive, store, serve, forward, and GC9A01 display.
//
// On top of v0.2:
// - Bodmer TFT_eSPI driver for the 240x240 round GC9A01 panel on hardware HSPI
//   (SCK=14, MOSI=13, CS=0, DC=4, RST=5, BL=15; GPIO12 kept LOW for buzzer).
// - Two-hemisphere brewery display layout: Slot 1 top, Slot 2 bottom,
//   FreeSansBold18pt7b SG numerals, FreeSansBold9pt7b ice-cyan temperature
//   and colour-coded battery, perimeter status arcs, plus centre heartbeat dot.
// - Boot-time replay of /0.jsonl, /1.jsonl, /2.jsonl so the display and
//   SVG graphs are populated immediately across power cycles.

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <time.h>
#include <WiFiManager.h>
#include <SPI.h>
#include <TFT_eSPI.h>

static ESP8266WebServer server(80);
static const char *FIRMWARE_VERSION = "0.3.0";
static bool fsMounted = false;

#define TFT_BL_PIN 15
#define BUZZER_PIN 12
#define GFXFF 1

static TFT_eSPI tft = TFT_eSPI(240, 240);

// RGB565 colour palette (per display_layout_spec.md)
#define COL_BG       0x0000 // Pure Black (#000000)
#define COL_SURFACE  0x18E4 // Dark Slate (#181C20)
#define COL_DIVIDER  0x31A7 // Muted Steel (#303438)
#define COL_SG       0xFFDF // Warm White (#F8FAF8)
#define COL_TEMP     0x3EDF // Ice Cyan (#38DBFF)
#define COL_NAME     0xB5D8 // Silver Grey (#B0B8C0)
#define COL_OK       0x270C // Emerald Green (#20E060)
#define COL_WARN     0xFDC0 // Amber Gold (#FFB800)
#define COL_ALERT    0xF986 // Coral Red (#FF3030)
#define COL_EMPTY    0x52CC // Dim Charcoal (#505860)

static void drawDisplay(bool fullClear = false);
static void drawHeartbeatDot(uint16_t col);

// ---- config ----
// /config.json on SPIFFS: the same path and key names as the original
// relay's firmware (verified by reading its boot log), so an app image swap
// can keep the operator's settings. Keys this firmware doesn't use are
// preserved on every save.
static const char *CONFIG_PATH = "/config.json";
static struct StationConfig
{
  String ssid, password;
  String apSsid, apPass;
  int service = 255; // 255 = none; forward nothing until configured
  String token;      // accepted, unused (the original's cloud token slot)
  String serverAddress;
  String uri = "/";
  int customPeriod = 900;
  int ledState = 1;
  int tftRotate = 3;
} cfg;

static void defaultConfig()
{
  cfg.apSsid = "nautilus-ispindel";
  cfg.apPass = "1234567890";
  cfg.service = 255;
  cfg.uri = "/";
  cfg.customPeriod = 900;
  cfg.ledState = 1;
  cfg.tftRotate = 3;
}

static void loadConfig()
{
  defaultConfig();
  DynamicJsonDocument doc(2048);
  if (!SPIFFS.exists(CONFIG_PATH))
    return;
  File f = SPIFFS.open(CONFIG_PATH, "r");
  if (!f)
    return;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err)
  {
    Serial.printf("config.json unreadable: %s\n", err.c_str());
    return;
  }
  cfg.ssid = doc["ssid"] | (const char *)"";
  cfg.password = doc["password"] | (const char *)"";
  cfg.apSsid = doc["ap_ssid"] | cfg.apSsid;
  cfg.apPass = doc["ap_pass"] | cfg.apPass;
  cfg.service = doc["service"] | 255;
  cfg.token = doc["TOKEN"] | (const char *)"";
  cfg.serverAddress = doc["server_address"] | (const char *)"";
  cfg.uri = doc["uri"] | (const char *)"/";
  cfg.customPeriod = doc["custom_period"] | 900;
  cfg.ledState = doc["ledState"] | 1;
  cfg.tftRotate = doc["tft_rotate"] | 3;
  Serial.printf("config: service=%d server=%s ap=%s led=%d rot=%d\n",
                cfg.service, cfg.serverAddress.c_str(), cfg.apSsid.c_str(),
                cfg.ledState, cfg.tftRotate);
}

// Merge-save: start from the existing doc, apply the present query args,
// write the whole thing back so untouched keys survive.
static void handleConfigSave()
{
  Serial.println("[get-save] enter");
  DynamicJsonDocument doc(4096);
  if (SPIFFS.exists(CONFIG_PATH))
  {
    File f = SPIFFS.open(CONFIG_PATH, "r");
    if (f)
    {
      deserializeJson(doc, f); // tolerate a damaged file: it may be empty
      f.close();
    }
  }
  const char *textKeys[] = {"ssid", "password", "ap_ssid", "ap_pass", "TOKEN",
                            "server_address", "uri", "mqtt_user", "mqtt_pass",
                            "uri_custom"};
  for (const char *k : textKeys)
    if (server.hasArg(k))
      doc[k] = server.arg(k);
  const char *intKeys[] = {"service", "custom_period", "i_broj", "unit_1",
                           "unit_2", "pressure_unit", "ledState", "tft_rotate",
                           "buzz_yesno", "temp_yesno", "temp_unit", "alarm_1",
                           "alarm_2", "alarm_3", "alarm_4", "alarm_5", "alarm_6",
                           "minpressure", "maxpressure"};
  for (const char *k : intKeys)
    if (server.hasArg(k))
      doc[k] = server.arg(k).toInt();
  const char *floatKeys[] = {"alarm_SG1", "alarm_SG2", "SGkorekcija1",
                             "SGkorekcija2", "Tempkorekcija1", "Tempkorekcija2",
                             "maxpressure_bar", "TempCorrection1"};
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
  digitalWrite(TFT_BL_PIN, cfg.ledState ? HIGH : LOW);
  if (cfg.tftRotate != prevRot)
  {
    tft.setRotation(cfg.tftRotate % 4);
  }
  drawDisplay(true);
  server.send(200, "text/html",
              "<h1>Settings saved. Reboot to apply.</h1><a href='/'>home</a>");
}

// ---- storage ----
static const char *SLOT_FILE[3] = {"/0.jsonl", "/1.jsonl", "/2.jsonl"};
static const uint32_t MAX_FILE_BYTES = 120000;

static bool slotActive[3] = {false, false, false};
static String lastName[3] = {"", "", ""};
static float lastSg[3] = {0, 0, 0};
static float lastTemp[3] = {0, 0, 0};
static float lastBatt[3] = {0, 0, 0};
static uint32_t lastInterval[3] = {900, 900, 900};
static char lastTempUnit[3] = {'C', 'C', 'C'};
static unsigned long lastSeenMillis[3] = {0, 0, 0};
static uint32_t lastSeenEpoch[3] = {0, 0, 0};

static const uint16_t GRAPH_POINTS = 400;
struct GraphPoint
{
  uint32_t ts;
  float sg;
};
static GraphPoint graphRing[3][GRAPH_POINTS]; // 3 * 400 * 8 B static
static uint16_t graphCount[3] = {0, 0, 0};
static uint16_t graphNext[3] = {0, 0, 0};

static bool sntpActive()
{
  time_t t = time(nullptr);
  return t > 1600000000;
}

static uint32_t epochNow()
{
  if (sntpActive())
    return (uint32_t)time(nullptr);
  return millis() / 1000;
}

static uint32_t slotAgeSeconds(int s)
{
  if (!slotActive[s])
    return 0;
  if (lastSeenMillis[s] > 0)
    return (uint32_t)((millis() - lastSeenMillis[s]) / 1000UL);
  if (sntpActive() && lastSeenEpoch[s] > 1600000000UL)
  {
    uint32_t now = (uint32_t)time(nullptr);
    if (now >= lastSeenEpoch[s])
      return now - lastSeenEpoch[s];
  }
  return (uint32_t)(millis() / 1000UL);
}

static int slotForName(const String &name)
{
  if (name.startsWith("1-"))
    return 1;
  if (name.startsWith("2-"))
    return 2;
  return 0;
}

static void pushGraphPoint(int slot, uint32_t ts, float sg)
{
  GraphPoint &p = graphRing[slot][graphNext[slot]];
  p.ts = ts;
  p.sg = sg;
  graphNext[slot] = (graphNext[slot] + 1) % GRAPH_POINTS;
  if (graphCount[slot] < GRAPH_POINTS)
    graphCount[slot]++;
}

static void loadHistoricalSlots()
{
  DynamicJsonDocument doc(1024);
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
        uint32_t ts = doc["ts"] | 0U;
        JsonObject payload = doc["payload"];
        if (!payload.isNull())
        {
          float sg = payload["gravity"] | 0.0f;
          if (sg > 0.5f)
            pushGraphPoint(s, ts, sg);
          slotActive[s] = true;
          lastName[s] = payload["name"] | String("Slot ") + String(s);
          lastSg[s] = sg;
          lastTemp[s] = payload["temperature"] | 0.0f;
          lastBatt[s] = payload["battery"] | 0.0f;
          uint32_t iv = payload["interval"] | 900U;
          lastInterval[s] = iv > 0 ? iv : 900U;
          const char *tu = payload["temp_units"] | "C";
          lastTempUnit[s] = (tu && tu[0]) ? tu[0] : 'C';
          lastSeenEpoch[s] = ts;
        }
      }
      yield();
    }
    f.close();
  }
}

static bool appendJsonl(const char *path, const String &line)
{
  Serial.printf("[append] %s\n", path);
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

// ---- forwarder ----
// Generic HTTP (service 10): one connect per received reading, sent on
// receipt like the original. server_address may carry host:port.
static void forwardOnReceipt(const String &body)
{
  if (cfg.service != 10 || cfg.serverAddress.isEmpty())
    return;
  String host = cfg.serverAddress;
  int port = 80;
  int colon = host.lastIndexOf(':');
  bool hasPort = false;
  if (colon > 0)
  {
    String p = host.substring(colon + 1);
    if (p.toInt() > 0)
    {
      port = p.toInt();
      host = host.substring(0, colon);
      hasPort = true;
    }
  }
  if (host.isEmpty())
    return;
  drawHeartbeatDot(COL_SG); // Flash white while forwarding
  WiFiClient c;
  String path = cfg.uri.isEmpty() ? String("/") : cfg.uri;
  bool ok = c.connect(host.c_str(), port); // 5 s default timeout inside connect
  if (!ok)
  {
    Serial.printf("forward connect failed %s:%d%s\n", host.c_str(), port,
                  hasPort ? "" : " (host only)");
    drawHeartbeatDot(COL_WARN);
    return;
  }
  String req = "POST " + path + " HTTP/1.1\r\nHost: " + host + "\r\n" +
               "Content-Type: application/json\r\n" +
               "Content-Length: " + String(body.length()) + "\r\n" +
               "Connection: close\r\n\r\n" + body;
  c.print(req);
  uint32_t deadline = millis() + 1000;
  while (c.connected() && millis() < deadline)
    delay(10);
  c.stop();
  Serial.printf("forwarded %u bytes to %s:%d%s\n", body.length(), host.c_str(),
                port, path.c_str());
  drawHeartbeatDot((WiFi.status() == WL_CONNECTED) ? COL_OK : COL_ALERT);
}

static bool recordSlot(int slot, const String &name, float sg, float temp,
                       float batt, uint32_t interval, char tempUnit,
                       const String &body)
{
  uint32_t nowTs = epochNow();
  String line = "{\"ts\":" + String(nowTs) + ",\"payload\":" + body + "}";
  if (!appendJsonl(SLOT_FILE[slot], line))
  {
    server.send(500, "application/json", "{\"error\":\"write failed\"}");
    Serial.printf("append failed for %s\n", SLOT_FILE[slot]);
    return false;
  }
  slotActive[slot] = true;
  lastName[slot] = name;
  lastSg[slot] = sg;
  lastTemp[slot] = temp;
  lastBatt[slot] = batt;
  lastInterval[slot] = interval > 0 ? interval : 900U;
  lastTempUnit[slot] = tempUnit;
  lastSeenMillis[slot] = millis();
  lastSeenEpoch[slot] = nowTs;
  server.send(200, "application/json", "{\"status\":\"ok\"}");
  return true;
}

static void handleReceive()
{
  Serial.println("[receive] enter");
  String body = server.arg("plain");
  if (body.length() == 0 || body.length() > 2048)
  {
    server.send(400, "application/json", "{\"error\":\"bad body\"}");
    return;
  }
  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, body);
  if (err)
  {
    server.send(400, "application/json", "{\"error\":\"bad json\"}");
    Serial.printf("rejecting post (%u bytes): %s\n", body.length(), err.c_str());
    return;
  }
  String name = doc["name"] | String("unknown");
  int slot = slotForName(name);
  float sg = doc["gravity"] | 0.0f;
  float temp = doc["temperature"] | 0.0f;
  float batt = doc["battery"] | 0.0f;
  uint32_t interval = doc["interval"] | 900U;
  const char *tu = doc["temp_units"] | "C";
  char tempUnit = (tu && tu[0]) ? tu[0] : 'C';
  if (!recordSlot(slot, name, sg, temp, batt, interval, tempUnit, body))
    return; // disk refused the reading; do not forward what we failed to keep

  if (doc["gravity"].is<float>())
  {
    pushGraphPoint(slot, epochNow(), sg);
  }
  drawDisplay(true);
  forwardOnReceipt(body); // after the reading is stored and answered
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
  uint16_t start = (n == GRAPH_POINTS) ? graphNext[slot] : 0;
  float lo = INFINITY, hi = -INFINITY;
  for (uint16_t i = 0; i < n; i++)
  {
    float v = graphRing[slot][(start + i) % GRAPH_POINTS].sg;
    if (v < lo)
      lo = v;
    if (v > hi)
      hi = v;
  }
  const float pad = (hi - lo) < 0.02f ? 0.01f : (hi - lo) * 0.05f;
  lo -= pad;
  hi += pad;
  const int W = 700, H = 300, M = 40;
  String pts;
  if (!pts.reserve(n * 14))
  {
    server.send(503, "text/plain", "not enough memory for graph");
    return;
  }
  for (uint16_t i = 0; i < n; i++)
  {
    GraphPoint &p = graphRing[slot][(start + i) % GRAPH_POINTS];
    float x = M + (W - 2 * M) * (n == 1 ? 0.5f : (float)i / (float)(n - 1));
    float y = H - M - (H - 2 * M) * ((p.sg - lo) / (hi - lo));
    if (i)
      pts += ' ';
    pts += String((int)x) + "," + String(y, 1);
  }
  String svg = "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 ";
  svg += String(W) + " " + String(H) + "' style='max-width:100%'>"
         "<rect width='" + String(W) + "' height='" + String(H) +
         "' fill='#1f252b'/>"
         "<polyline fill='none' stroke='#7fb5de' stroke-width='2' points='" +
         pts + "'/>";
  GraphPoint &oldest = graphRing[slot][start];
  GraphPoint &newest = graphRing[slot][(start + n - 1) % GRAPH_POINTS];
  svg += "<text x='" + String(M) + "' y='" + String(H - 10) +
         "' fill='#9fb8a5' font-size='12'>";
  if (sntpActive())
    svg += "SG " + String(lo + pad, 3) + "-" + String(hi - pad, 3) +
           ", span " + String((uint32_t)(newest.ts - oldest.ts)) + " s";
  svg += "</text></svg>";
  server.send(200, "image/svg+xml", svg);
}

static void handleLive()
{
  String page = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>iSpindleStation</title>"
                "<meta http-equiv='refresh' content='30'></head>"
                "<body style='font-family:sans-serif;margin:0;padding:15px;text-align:center'>";
  page += "<h2>iSpindleStation</h2>";
  for (int s = 0; s < 3; s++)
  {
    page += "<hr>";
    if (!slotActive[s])
    {
      page += "<p>Slot " + String(s) + ": no reading yet</p>";
      continue;
    }
    uint32_t ago = slotAgeSeconds(s);
    page += "<p><b>Slot " + String(s) + "</b> (" + lastName[s] + ") - seen " +
            String(ago) + " s ago</p>";
    page += "<p>SG: " + String(lastSg[s], 4) +
            " - Temp: " + String(lastTemp[s], 2) + " " + String(lastTempUnit[s]) +
            " - Battery: " + String(lastBatt[s], 2) + " V</p>";
    if (s != 0)
      page += "<p><a href='/graph/" + String(s) + ".svg'>Graph</a> - " +
              "<a href='/data/" + String(s) + ".jsonl'>Data</a></p>";
  }
  page += "<hr><p style='font-size:.8em'>iSpindleStation v" + String(FIRMWARE_VERSION) +
          " - " + WiFi.localIP().toString() + " - ap " + WiFi.softAPIP().toString() +
          "</p></body></html>";
  server.send(200, "text/html", page);
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
  String j;
  serializeJson(doc, j);
  server.send(200, "application/json", j);
}

static void handleReboot()
{
  server.send(200, "text/plain", "rebooting");
  delay(500);
  ESP.restart();
}

static void handleNotFound()
{
  if (server.method() == HTTP_POST)
  {
    // Any unregistered POST path funnels into the receiver, so spindles
    // pointed at any URI on this host still land.
    handleReceive();
    return;
  }
  server.send(404, "text/plain", "not found");
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
  JsonArray slots = doc.createNestedArray("slots");
  for (int s = 0; s < 3; s++)
  {
    JsonObject so = slots.createNestedObject();
    so["slot"] = s;
    so["active"] = slotActive[s];
    so["name"] = lastName[s];
    so["sg"] = lastSg[s];
    so["temp"] = lastTemp[s];
    so["batt"] = lastBatt[s];
    so["ageSec"] = slotAgeSeconds(s);
    so["points"] = graphCount[s];
  }
  String j;
  serializeJson(doc, j);
  server.send(200, "application/json", j);
}

// ---- GC9A01 two-hemisphere display ----
static void drawHeartbeatDot(uint16_t col)
{
  tft.fillSmoothCircle(120, 120, 5, col, COL_BG);
}

static void formatAge(uint32_t ageSec, char *out, size_t outLen)
{
  uint32_t mins = ageSec / 60U;
  if (mins < 60U)
    snprintf(out, outLen, "%lum", (unsigned long)mins);
  else if (mins < 1440U)
    snprintf(out, outLen, "%luh%lum", (unsigned long)((mins / 60U) % 100U),
             (unsigned long)(mins % 60U));
  else
    snprintf(out, outLen, "%lud", (unsigned long)((mins / 1440U) % 1000U));
}

static void drawSlotHemisphere(bool isTop, int dataSlot)
{
  // In TFT_eSPI::drawSmoothArc, 0 deg = 6 o'clock (bottom), clockwise:
  // Top hemisphere arc: 100..260 deg; Bottom hemisphere arc: 280..80 deg.
  uint32_t arcStart = isTop ? 100 : 280;
  uint32_t arcEnd   = isTop ? 260 : 80;

  int yName = isTop ? 35  : 204;
  int ySG   = isTop ? 78  : 156;
  int yTemp = isTop ? 68  : 146;
  int yBatt = isTop ? 93  : 171;
  int xRight = 190;

  // Clear interior of this hemisphere without touching the R=109..113 arc or y=120 divider
  if (isTop)
  {
    tft.fillRect(58, 22, 124, 24, COL_BG);
    tft.fillRect(28, 46, 184, 70, COL_BG);
  }
  else
  {
    tft.fillRect(28, 124, 184, 68, COL_BG);
    tft.fillRect(58, 192, 124, 24, COL_BG);
  }

  if (dataSlot < 0 || !slotActive[dataSlot])
  {
    tft.setTextDatum(MC_DATUM);
    if (isTop)
    {
      tft.setTextColor(COL_EMPTY);
      tft.drawString("SLOT 1", 120, yName, 2);
    }
    else
    {
      String ip = (WiFi.status() == WL_CONNECTED)
                      ? WiFi.localIP().toString()
                      : WiFi.softAPIP().toString();
      tft.setTextColor(COL_NAME);
      tft.drawString(ip, 120, yName, 2);
    }
    tft.setTextColor(COL_EMPTY);
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.setTextDatum(ML_DATUM);
    tft.drawString("-.---", 36, ySG, GFXFF);
    tft.setTextDatum(MR_DATUM);
    tft.drawString(isTop ? "WAITING" : "IDLE", xRight, ySG, 2);
    tft.drawSmoothArc(120, 120, 113, 109, arcStart, arcEnd, COL_DIVIDER, COL_BG, true);
    return;
  }

  float sg = lastSg[dataSlot];
  float temp = lastTemp[dataSlot];
  float batt = lastBatt[dataSlot];
  uint32_t ageSec = slotAgeSeconds(dataSlot);
  uint32_t intervalSec = lastInterval[dataSlot] > 0 ? lastInterval[dataSlot] : 900U;

  uint16_t battCol = (batt >= 3.70f) ? COL_OK : (batt >= 3.45f ? COL_WARN : COL_ALERT);
  bool timeout = (ageSec > intervalSec * 4U + 120U);
  bool stale   = (ageSec > intervalSec * 2U + 60U);
  uint16_t arcCol = (timeout || batt < 3.45f)
                        ? COL_ALERT
                        : ((stale || batt < 3.70f) ? COL_WARN : COL_OK);

  // 1. Name + Age Badge (with subtle pill background, inside R=106)
  char ageBuf[16];
  formatAge(ageSec, ageBuf, sizeof(ageBuf));
  char hdr[28];
  snprintf(hdr, sizeof(hdr), "%.8s  %s", lastName[dataSlot].c_str(), ageBuf);
  tft.fillSmoothRoundRect(62, yName - 10, 116, 20, 8, COL_SURFACE, COL_BG);
  tft.setTextColor(timeout ? COL_ALERT : (stale ? COL_WARN : COL_NAME));
  tft.setTextDatum(MC_DATUM);
  tft.drawString(hdr, 120, yName, 2);

  // 2. Hero Specific Gravity (FreeSansBold18pt7b = 24 px cap height, ~84 px wide)
  char sgBuf[10];
  snprintf(sgBuf, sizeof(sgBuf), "%.3f", sg);
  tft.setFreeFont(&FreeSansBold18pt7b);
  tft.setTextColor(COL_SG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(sgBuf, 34, ySG, GFXFF);

  // 3. Temperature (FreeSansBold9pt7b = 12 px cap height, Ice Cyan, with degree ring)
  char unitStr[2] = {lastTempUnit[dataSlot], 0};
  char numBuf[10];
  snprintf(numBuf, sizeof(numBuf), "%.1f", temp);
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(COL_TEMP);
  tft.setTextDatum(MR_DATUM);
  int uW = tft.drawString(unitStr, xRight, yTemp, GFXFF);
  int degX = xRight - uW - 4;
  tft.drawSmoothCircle(degX, yTemp - 3, 2, COL_TEMP, COL_BG);
  tft.drawString(numBuf, degX - 4, yTemp, GFXFF);

  // 4. Battery Voltage (FreeSansBold9pt7b = 12 px cap height, colour-coded)
  char batBuf[12];
  snprintf(batBuf, sizeof(batBuf), "%.2fV", batt);
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(battCol);
  tft.setTextDatum(MR_DATUM);
  tft.drawString(batBuf, xRight, yBatt, GFXFF);

  // 5. Perimeter Status Arc (drawn last so text boxes never clip it)
  tft.drawSmoothArc(120, 120, 113, 109, arcStart, arcEnd, arcCol, COL_BG, true);
}

static void drawDisplay(bool fullClear)
{
  if (fullClear)
    tft.fillScreen(COL_BG);

  // Top hemisphere shows Slot 1 (or Slot 0 if Slot 1 is unused and Slot 0 is active)
  int topSlot = slotActive[1] ? 1 : (slotActive[0] ? 0 : -1);
  int botSlot = slotActive[2] ? 2 : -1;

  drawSlotHemisphere(true, topSlot);
  drawSlotHemisphere(false, botSlot);

  // Center horizontal divider line + Relay Wi-Fi heartbeat dot
  tft.drawFastHLine(18, 120, 204, COL_DIVIDER);
  tft.drawFastHLine(18, 121, 204, COL_DIVIDER);
  uint16_t wifiCol = (WiFi.status() == WL_CONNECTED) ? COL_OK : COL_ALERT;
  drawHeartbeatDot(wifiCol);
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("\niSpindleStation boot");
  fsMounted = SPIFFS.begin();
  if (!fsMounted)
  {
    Serial.println("SPIFFS mount FAILED - formatting");
    delay(100);
    SPIFFS.format();
    fsMounted = SPIFFS.begin();
    if (!fsMounted)
      Serial.println("SPIFFS mount failed after format");
  }
  if (fsMounted)
  {
    Dir rd = SPIFFS.openDir("/");
    Serial.println("FS contents:");
    while (rd.next())
      Serial.printf("  %-24s %u B\n", rd.fileName().c_str(), (unsigned)rd.fileSize());
  }

  loadConfig();
  loadHistoricalSlots();

  // Bring up GC9A01 display before Wi-Fi join so the screen wakes immediately
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, cfg.ledState ? HIGH : LOW);
  tft.init();
  // Keep buzzer pin (GPIO12) LOW after SPI.begin() sets HSPI MISO
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  tft.setRotation(cfg.tftRotate % 4);
  drawDisplay(true);

  WiFi.hostname("iSpindleStation");

  bool joined = false;
  if (cfg.ssid.length() > 0)
  {
    WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());
    for (int i = 0; i < 25 && !joined; i++)
    {
      delay(1000);
      joined = WiFi.status() == WL_CONNECTED;
    }
  }
  if (!joined)
  { // fall back to the AP portal, bounded like the original's
    WiFiManager wm;
    wm.setConfigPortalTimeout(300);
    wm.setBreakAfterConfig(true);
    if (!wm.autoConnect("iSpindleStation"))
    {
      Serial.println("no connection after portal, restarting");
      delay(2000);
      ESP.restart();
    }
  }
  Serial.printf("joined %s as %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

  // Companion AP for spindles already paired to the box; the radio shares
  // the STA's channel, so the AP follows whatever the join landed on.
  if (!cfg.apSsid.isEmpty())
  {
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));
    WiFi.softAP(cfg.apSsid.c_str(), cfg.apPass.c_str());
    Serial.printf("soft AP %s at %s\n", cfg.apSsid.c_str(),
                  WiFi.softAPIP().toString().c_str());
  }

  configTime(0, 0, "pool.ntp.org");

  server.on("/", HTTP_GET, handleLive);
  server.on("/gravity", HTTP_POST, handleReceive);
  server.on("/get", HTTP_GET, handleConfigSave);
  server.on("/reboot", HTTP_ANY, handleReboot);
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

  // Redraw now that Wi-Fi is connected (updates heartbeat dot + IP footer)
  drawDisplay(true);
}

static unsigned long lastPing = 0;

void loop()
{
  server.handleClient();
  if (millis() - lastPing > 30000)
  {
    lastPing = millis();
    Serial.printf("[loop] t=%lu heap=%u apClients=%d\n", millis() / 1000,
                  ESP.getFreeHeap(), WiFi.softAPgetStationNum());
    drawDisplay(false);
  }
}
