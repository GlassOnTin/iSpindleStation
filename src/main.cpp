// iSpindleStation v0.2.0 — receive, store, serve, forward.
//
// On top of v0: config.json load/merge/save via the GET /get surface (the
// vendor's field names, drop-in with their flash), a companion soft-AP so
// paired iSpindels keep their link after the swap, and a generic-HTTP
// forwarder that sends each received reading onward on receipt, accepting
// host:port in server_address (the original locks the port to 80).
//
// Slots, file naming, stamping and the live/graph pages are as in v0.
// Hardware drivers (TFT, buzzer, relay output, pressure sensor) are
// deliberately absent; they land after board bring-up.

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <time.h>
#include <WiFiManager.h>

static ESP8266WebServer server(80);
static const char *FIRMWARE_VERSION = "0.2.2";

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
} cfg;

static void defaultConfig()
{
  // The original's factory AP pair, so a spindle already paired to the box
  // keeps posting while the flash swap settles. Empty server => forwarding
  // off, which is the safe state for a fresh filesystem.
  cfg.apSsid = "nautilus-ispindel";
  cfg.apPass = "1234567890";
  cfg.service = 255;
  cfg.uri = "/";
  cfg.customPeriod = 900;
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
  Serial.printf("config: service=%d server=%s ap=%s\n", cfg.service,
                cfg.serverAddress.c_str(), cfg.apSsid.c_str());
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
  Serial.println("[get-save] args merged");
  struct KV
  {
    const char *key;
  };
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
  if (server.hasArg("TempCorrection1"))
    doc["TempCorrection1"] = server.arg("TempCorrection1").toFloat();
  Serial.println("[get-save] writing config");
  File out = SPIFFS.open(CONFIG_PATH, "w");
  if (!out)
  {
    server.send(500, "text/plain", "config write failed");
    return;
  }
  serializeJson(doc, out);
  out.close();
  Serial.println("[get-save] done");
  loadConfig();
  server.send(200, "text/html",
              "<h1>Settings saved. Reboot to apply.</h1><a href='/'>home</a>");
}

// ---- storage ----
static const char *SLOT_FILE[3] = {"/0.jsonl", "/1.jsonl", "/2.jsonl"};
static const uint32_t MAX_FILE_BYTES = 120000;

static String lastName[3] = {"", "", ""};
static float lastSg[3] = {0, 0, 0};
static float lastTemp[3] = {0, 0, 0};
static float lastBatt[3] = {0, 0, 0};
static unsigned long lastSeenMillis[3] = {0, 0, 0};

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

static int slotForName(const String &name)
{
  if (name.startsWith("1-"))
    return 1;
  if (name.startsWith("2-"))
    return 2;
  return 0;
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
  WiFiClient c;
  String path = cfg.uri.isEmpty() ? String("/") : cfg.uri;
  bool ok = c.connect(host.c_str(), port); // 5 s default timeout inside connect
  if (!ok)
  {
    Serial.printf("forward connect failed %s:%d%s\n", host.c_str(), port,
                  hasPort ? "" : " (host only)");
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
}

static bool recordSlot(int slot, const String &name, float sg, float temp,
                       float batt, const String &body)
{
  String line = "{\"ts\":" + String(epochNow()) + ",\"payload\":" + body + "}";
  if (!appendJsonl(SLOT_FILE[slot], line))
  {
    server.send(500, "application/json", "{\"error\":\"write failed\"}");
    Serial.printf("append failed for %s\n", SLOT_FILE[slot]);
    return false;
  }
  lastName[slot] = name;
  lastSg[slot] = sg;
  lastTemp[slot] = temp;
  lastBatt[slot] = batt;
  lastSeenMillis[slot] = millis();
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
  int slot = slotForName(doc["name"] | String("unknown"));
  String name = doc["name"] | String("unknown");
  float sg = doc["gravity"] | 0.0f;
  float temp = doc["temperature"] | 0.0f;
  float batt = doc["battery"] | 0.0f;
  if (!recordSlot(slot, name, sg, temp, batt, body))
    return; // disk refused the reading; do not forward what we failed to keep

  if (doc["gravity"].is<float>())
  {
    GraphPoint &p = graphRing[slot][graphNext[slot]];
    p.ts = epochNow();
    p.sg = sg;
    graphNext[slot] = (graphNext[slot] + 1) % GRAPH_POINTS;
    if (graphCount[slot] < GRAPH_POINTS)
      graphCount[slot]++;
  }
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
    if (lastSeenMillis[s] == 0)
    {
      page += "<p>Slot " + String(s) + ": no reading yet</p>";
      continue;
    }
    long ago = (long)((millis() - lastSeenMillis[s]) / 1000);
    page += "<p><b>Slot " + String(s) + "</b> (" + lastName[s] + ") - seen " +
            String(ago) + " s ago</p>";
    page += "<p>SG: " + String(lastSg[s], 4) +
            " - Temp: " + String(lastTemp[s], 2) + " C" +
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
  fs::FSInfo64 fsinfo;
  SPIFFS.info64(fsinfo);
  String j = "{\"firmware\":\"" + String(FIRMWARE_VERSION) +
             "\",\"fsTotal\":" + String((uint32_t)fsinfo.totalBytes) +
             ",\"fsUsed\":" + String((uint32_t)fsinfo.usedBytes) +
             ",\"fsMounted\":" + (SPIFFS.begin() ? "true" : "false") +
             ",\"sta\":\"" + WiFi.localIP().toString() +
             "\",\"apClients\":" + String(WiFi.softAPgetStationNum()) +
             ",\"heap\":" + String(ESP.getFreeHeap()) +
             ",\"files\":[";
  Dir rd = SPIFFS.openDir("/");
  bool first = true;
  while (rd.next())
  {
    if (!first)
      j += ",";
    first = false;
    j += "{\"name\":\"" + rd.fileName() + "\",\"size\":" +
         String((unsigned)rd.fileSize()) + "}";
  }
  j += "]";
  if (SPIFFS.exists(CONFIG_PATH))
  {
    File f = SPIFFS.open(CONFIG_PATH, "r");
    if (f)
    {
      String cfgText = f.readString(); // config is < 1 KB; heap can hold it
      f.close();
      if (cfgText.length() > 600)
        cfgText = cfgText.substring(0, 600);
      j += ",\"configText\":" + cfgText;
    }
  }
  j += "}";
  server.send(200, "application/json", j);
}




// ---- display probe (benching, not production) ----
// Candidate pins: runtime-safe GPIOs on this board family. GPIO 0/15 are
// boot straps, so every probe restores inputs before returning.
static const uint8_t PROBE_PINS[8] = {0, 2, 4, 5, 12, 13, 14, 16};

static void pinFloat(uint8_t p) { pinMode(p, INPUT); }
static void pinLow(uint8_t p) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }

static void pinWrite(uint8_t p, bool v, bool od)
{
  if (od && !v)
  {
    digitalWrite(p, LOW);
    pinMode(p, OUTPUT);
    return;
  }
  pinMode(p, p == 16 ? INPUT : INPUT_PULLUP); // released high unless GPIO16
  digitalWrite(p, HIGH);
}

static bool i2cReadSda(uint8_t sda) { return digitalRead(sda); }

// One I2C START with given levels on the pair.
static void i2cStartBit(uint8_t sda, uint8_t scl)
{
  pinWrite(sda, true, true);
  pinWrite(scl, true, true);
  delayMicroseconds(4);
  pinWrite(sda, false, true);
  delayMicroseconds(4);
  pinWrite(scl, false, true);
}

static void i2cStopBit(uint8_t sda, uint8_t scl)
{
  pinWrite(sda, false, true);
  pinWrite(scl, true, true);
  delayMicroseconds(4);
  pinWrite(sda, true, true);
  delayMicroseconds(4);
}

static void i2cBit(uint8_t sda, uint8_t scl, bool bit)
{
  pinWrite(sda, bit, true);
  delayMicroseconds(2);
  pinWrite(scl, true, true);
  delayMicroseconds(4);
  pinWrite(scl, false, true);
}

static bool i2cTryAddress(uint8_t sda, uint8_t scl, uint8_t addr)
{
  i2cStartBit(sda, scl);
  // address byte with W bit; sample the ACK after it
  for (int i = 7; i >= 0; i--)
    i2cBit(sda, scl, (addr >> i) & 1);
  // release SDA, clock one bit, read ACK
  pinWrite(sda, true, true);
  delayMicroseconds(2);
  pinWrite(scl, true, true);
  delayMicroseconds(4);
  bool ack = i2cReadSda(sda) == false; // ACK = SDA low
  pinWrite(scl, false, true);
  i2cStopBit(sda, scl);
  return ack;
}

static String probeI2c()
{
  String out = "{\"i2c\":[";
  bool first = true;
  for (uint8_t s : PROBE_PINS)
  {
    for (uint8_t c : PROBE_PINS)
    {
      if (s == c)
        continue;
      pinFloat(s);
      pinFloat(c);
      for (uint8_t addr : {0x3C, 0x3D})
      {
        if (i2cTryAddress(s, c, addr))
        {
          if (!first)
            out += ",";
          first = false;
          out += "{\"sda\":" + String(s) + ",\"scl\":" + String(c) +
                 ",\"addr\":\"0x" + String(addr, 16) + "\"}";
        }
      }
      pinFloat(s);
      pinFloat(c);
    }
  }
  out += "]}";
  return out;
}

// ---- bit-banged GC9A01 fill on one candidate pin group ----
static void bitLow(uint8_t p) { digitalWrite(p, LOW); }
static void bitHigh(uint8_t p) { digitalWrite(p, HIGH); }

// group pins are pre-set OUTPUT in fillGroup() (no pinMode in the hot loop)
static void bbClock(uint8_t sck, uint8_t mosi, bool d)
{
  digitalWrite(mosi, d);
  digitalWrite(sck, HIGH);
  digitalWrite(sck, LOW);
}

static void bbSpiByte(uint8_t sck, uint8_t mosi, uint8_t v, bool dc, uint8_t dcpin)
{
  digitalWrite(dcpin, dc);
  for (int i = 7; i >= 0; i--)
    bbClock(sck, mosi, (v >> i) & 1);
}

static void bbCmd(uint8_t sck, uint8_t mosi, uint8_t dcpin, uint8_t cmd)
{
  bbSpiByte(sck, mosi, cmd, false, dcpin);
}
static void bbData(uint8_t sck, uint8_t mosi, uint8_t dcpin, uint8_t v)
{
  bbSpiByte(sck, mosi, v, true, dcpin);
}

struct PinGroup
{
  uint8_t sck, mosi, dc, cs;
};
static const PinGroup GROUPS[4] = {
    {14, 13, 12, 16},
    {14, 5, 4, 16},
    {2, 0, 4, 5},
    {14, 13, 4, 5},
};

static String fillGroup(uint8_t gi)
{
  const PinGroup &g = GROUPS[gi];
  // set the four group pins as outputs once, DC/CS start low
  pinMode(g.sck, OUTPUT); digitalWrite(g.sck, LOW);
  pinMode(g.mosi, OUTPUT); digitalWrite(g.mosi, LOW);
  pinMode(g.dc, OUTPUT); digitalWrite(g.dc, LOW);
  pinMode(g.cs, OUTPUT); digitalWrite(g.cs, LOW);
  delay(2);
  bbCmd(g.sck, g.mosi, g.dc, 0x01); // SW reset
  delay(50);
  bbCmd(g.sck, g.mosi, g.dc, 0x11); // sleep out
  delay(120);
  bbCmd(g.sck, g.mosi, g.dc, 0x3A);
  bbData(g.sck, g.mosi, g.dc, 0x05); // 16-bit color
  bbCmd(g.sck, g.mosi, g.dc, 0x29);  // display on
  bbCmd(g.sck, g.mosi, g.dc, 0x2A);  // column: 0..239
  bbData(g.sck, g.mosi, g.dc, 0x00);
  bbData(g.sck, g.mosi, g.dc, 0x00);
  bbData(g.sck, g.mosi, g.dc, 0x00);
  bbData(g.sck, g.mosi, g.dc, 0xEF);
  bbCmd(g.sck, g.mosi, g.dc, 0x2B); // row: 0..239
  bbData(g.sck, g.mosi, g.dc, 0x00);
  bbData(g.sck, g.mosi, g.dc, 0x00);
  bbData(g.sck, g.mosi, g.dc, 0x00);
  bbData(g.sck, g.mosi, g.dc, 0xEF);
  bbCmd(g.sck, g.mosi, g.dc, 0x2C); // RAM write
  for (uint16_t n = 0; n < 240 * 240; n++)
  {
    bbData(g.sck, g.mosi, g.dc, 0xF8); // red high byte
    bbData(g.sck, g.mosi, g.dc, 0x00);
    if ((n & 2047) == 0)
      delay(0); // feed the soft WDT
  }
  pinWrite(g.cs, true, false);
  // restore: straps (0/15) float, others released with pullup
  for (uint8_t p : {g.sck, g.mosi, g.dc, g.cs})
    pinWrite(p, true, true);
  return "{\"group\":" + String(gi) + ",\"pins\":" +
         String(g.sck) + "/" + String(g.mosi) + "/" + String(g.dc) + "/" +
         String(g.cs) + "}";
}

static void handleProbe()
{
  String what = server.arg("what");
  String out;
  if (what == "fill")
  {
    uint8_t gi = server.arg("g").toInt();
    if (gi > 3) gi = 0;
    server.send(200, "application/json",
                "{\"started\":true,\"group\":" + String(gi) + ",\"pins\":\"14/13/12/16,14/5/4/16,2/0/4/5,14/13/4/5\"}");
    const String doneNote = fillGroup(gi);
    Serial.println("[probe] fill done " + doneNote);
    return;
  }
  else
  {
    out = probeI2c();
    // include group list so the fill experiment is self-describing
    out.replace("{\"i2c\":[", "{\"groups\":[{\"i\":0,\"pins\":\"14/13/12/16\"},{\"i\":1,\"pins\":\"14/5/4/16\"},{\"i\":2,\"pins\":\"2/0/4/5\"},{\"i\":3,\"pins\":\"14/13/4/5\"}],\"i2c\":[");
  }
  server.send(200, "application/json", out);
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
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("\niSpindleStation boot");
  if (!SPIFFS.begin())
  {
    Serial.println("SPIFFS mount FAILED - formatting");
    delay(100);
    SPIFFS.format();
    if (!SPIFFS.begin())
      Serial.println("SPIFFS mount failed after format");
  }
  {
    Dir rd = SPIFFS.openDir("/");
    Serial.println("FS contents:");
    while (rd.next())
      Serial.printf("  %-24s %u B\n", rd.fileName().c_str(), (unsigned)rd.fileSize());
  }

  loadConfig();
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
  server.on("/probe", HTTP_GET, handleProbe);
  server.onNotFound(handleNotFound);
  server.begin();
}





