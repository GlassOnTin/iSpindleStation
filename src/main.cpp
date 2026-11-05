// iSpindleStation v0 — receive, store, serve.
//
// Receives iSpindel POSTs (the generic-HTTP JSON payload), keeps a capped
// JSONL history per spindle slot in SPIFFS, and serves a live page plus SVG
// SG graphs. Forwarding and the analytics page come later; the analytics
// run in the browser (Kotlin/JS), not in firmware.
//
// Timestamps: each record gets epoch seconds from SNTP; while no time has
// reached the box the stamp is uptime seconds and the graph label says so.
//
// Hardware drivers (TFT, buzzer, relay output, pressure sensor) are
// deliberately absent in v0 — they land after the board bring-up.

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <time.h>
#include <WiFiManager.h> // tzapu, MIT (registry)

static ESP8266WebServer server(80);
static const char *FIRMWARE_VERSION = "0.1.0";

// ---- storage ----
// One JSONL file per slot. Slots follow the vendor-compatible name rule: a
// payload name starting "1-" goes to slot 1, "2-" to slot 2, anything else
// to slot 0 so no reading is silently dropped.
static const char *SLOT_FILE[3] = {"/0.jsonl", "/1.jsonl", "/2.jsonl"};

// Ring-cap per slot. The bench flash-id run pins the real SPIFFS budget;
// until then this stays conservative. Graph reads never hold the file in
// RAM (the display ring is fed at receive time), so a big file costs time,
// not heap.
static const uint32_t MAX_FILE_BYTES = 120000;

// ---- last-known state per slot (RAM copy for the live page) ----
static String lastName[3] = {"", "", ""};
static float lastSg[3] = {0, 0, 0};
static float lastTemp[3] = {0, 0, 0};
static float lastBatt[3] = {0, 0, 0};
static unsigned long lastSeenMillis[3] = {0, 0, 0};

// ---- display ring for the SVG graph (last N SG points per slot) ----
static const uint16_t GRAPH_POINTS = 400;
struct GraphPoint
{
  uint32_t ts;
  float sg;
};
static GraphPoint graphRing[3][GRAPH_POINTS]; // 3 * 400 * 8 B = 9.6 KB static
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

static int slotForName(const String &name)
{
  if (name.startsWith("1-"))
    return 1;
  if (name.startsWith("2-"))
    return 2;
  return 0;
}

// Append one line; when the file passes the cap, stream its newest half
// into a temp file and swap, never holding the file in RAM.
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
  while (src.available() && src.read() != '\n') // snap to a line start
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

static void recordSlot(int slot, const String &name, float sg, float temp,
                       float batt, DynamicJsonDocument &doc, const String &body)
{
  String line = "{\"ts\":" + String(epochNow()) + ",\"payload\":" + body + "}";
  if (!appendJsonl(SLOT_FILE[slot], line))
  {
    server.send(500, "application/json", "{\"error\":\"write failed\"}");
    Serial.printf("append failed for %s\n", SLOT_FILE[slot]);
    return;
  }
  lastName[slot] = name;
  lastSg[slot] = sg;
  lastTemp[slot] = temp;
  lastBatt[slot] = batt;
  lastSeenMillis[slot] = millis();

  if (doc["gravity"].is<float>())
  {
    GraphPoint &p = graphRing[slot][graphNext[slot]];
    p.ts = epochNow();
    p.sg = doc["gravity"];
    graphNext[slot] = (graphNext[slot] + 1) % GRAPH_POINTS;
    if (graphCount[slot] < GRAPH_POINTS)
      graphCount[slot]++;
  }
  server.send(200, "application/json", "{\"status\":\"ok\"}");
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
  if (err)
  {
    server.send(400, "application/json", "{\"error\":\"bad json\"}");
    Serial.printf("rejecting post (%u bytes): %s\n", body.length(), err.c_str());
    return;
  }
  String name = doc["name"] | String("unknown");
  float sg = doc["gravity"] | 0.0f;
  float temp = doc["temperature"] | 0.0f;
  float batt = doc["battery"] | 0.0f;
  recordSlot(slotForName(name), name, sg, temp, batt, doc, body);
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
  // Ring order: when full, graphNext points at the oldest entry; while
  // filling, slot 0 holds the first reading.
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
  { // ~14 chars per point; skip the build if RAM is short
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
    svg += "SG " + String(lo + pad, 3) + "–" + String(hi - pad, 3);
  svg += "</text>";
  svg += "<text x='" + String(W - 110) + "' y='20' fill='#9fb8a5' font-size='12'>";
  if (sntpActive())
    svg += "span " + String((uint32_t)(newest.ts - oldest.ts)) + " s";
  svg += "</text>";
  svg += "</svg>";
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
    if (lastSeenMillis[s] == 0)
    {
      page += "<p>Slot " + String(s) + ": no reading yet</p>";
      continue;
    }
    long ago = (long)((millis() - lastSeenMillis[s]) / 1000);
    page += "<p><b>Slot " + String(s) + "</b> (" + lastName[s] + ") — seen " +
            String(ago) + " s ago</p>";
    page += "<p>SG: " + String(lastSg[s], 4) +
            " · Temp: " + String(lastTemp[s], 2) + " °C" +
            " · Battery: " + String(lastBatt[s], 2) + " V</p>";
    if (s != 0)
      page += "<p><a href='/graph/" + String(s) + ".svg'>Graph</a> · " +
              "<a href='/data/" + String(s) + ".jsonl'>Data</a></p>";
  }
  page += "<hr><p style='font-size:.8em'>iSpindleStation v" + String(FIRMWARE_VERSION) +
          " · " + WiFi.localIP().toString() + "</p></body></html>";
  server.send(200, "text/html", page);
}

static void handleHealth()
{
  String j = "{\"name\":\"iSpindleStation\",\"firmware\":\"" + String(FIRMWARE_VERSION) +
             "\",\"epoch\":" + String(epochNow()) + ",\"sntp\":" +
             String(sntpActive() ? "true" : "false") + "}";
  server.send(200, "application/json", j);
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

void setup()
{
  Serial.begin(115200);
  if (!SPIFFS.begin())
    Serial.println("SPIFFS mount failed");

  WiFi.hostname("iSpindleStation");
  WiFiManager wm;
  wm.setConfigPortalTimeout(300); // bounded like the original's portal
  wm.setBreakAfterConfig(true);
  if (!wm.autoConnect("iSpindleStation"))
  { // falls back to the AP portal on failure
    Serial.println("no connection after portal, restarting");
    delay(2000);
    ESP.restart();
  }
  Serial.printf("joined %s as %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

  // SNTP for record timestamps; harmless when the net has no uplink.
  configTime(0, 0, "pool.ntp.org");

  server.on("/", HTTP_GET, handleLive);
  server.on("/gravity", HTTP_POST, handleReceive);
  server.on("/data/0.jsonl", HTTP_GET, []() { serveSlot(0); });
  server.on("/data/1.jsonl", HTTP_GET, []() { serveSlot(1); });
  server.on("/data/2.jsonl", HTTP_GET, []() { serveSlot(2); });
  server.on("/graph/0.svg", HTTP_GET, []() { serveGraph(0); });
  server.on("/graph/1.svg", HTTP_GET, []() { serveGraph(1); });
  server.on("/graph/2.svg", HTTP_GET, []() { serveGraph(2); });
  server.on("/health", HTTP_GET, handleHealth);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop()
{
  server.handleClient();
}
