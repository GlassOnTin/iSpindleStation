#pragma once

#include "state.h"
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <MD5Builder.h>

extern ESP8266WebServer server;

// ---- Piggybacked Config & OTA Push Store (ported from ispindle-proxy.go) ----
static const int MAX_WIFI_DELIVERIES = 5;
static const int MAX_FW_DELIVERIES = 5;

struct DeviceCfgPush
{
  String poly;
  int sleepSec = 0;
  String ssid;
  String psk;
  String fwUrl;
  String fwMd5;
  String fwVer;
  int wifiDeliveries = 0;
  int fwDeliveries = 0;

  bool empty() const
  {
    return poly.isEmpty() && sleepSec == 0 && ssid.isEmpty() && psk.isEmpty() && fwUrl.isEmpty();
  }
};

static DeviceCfgPush cfgPush;

static bool isPrintableAscii(const String &s)
{
  for (size_t i = 0; i < s.length(); i++)
  {
    char c = s[i];
    if (c < 0x20 || c > 0x7e)
      return false;
  }
  return true;
}

static bool isHex32(const String &s)
{
  if (s.length() != 32)
    return false;
  for (size_t i = 0; i < 32; i++)
  {
    char c = s[i];
    bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (!ok)
      return false;
  }
  return true;
}

static String validateCfgPush(const DeviceCfgPush &c)
{
  if (c.empty())
    return "cfg: POLY, sleep and/or SSID+PSK or fw required";
  if (!c.poly.isEmpty())
  {
    if (c.poly.length() > 250)
      return "cfg: POLY exceeds 250 chars";
    if (c.poly.indexOf("tilt") < 0)
      return "cfg: POLY must reference the \"tilt\" variable";
    if (!isPrintableAscii(c.poly))
      return "cfg: POLY must be printable ASCII";
  }
  if (c.sleepSec != 0 && (c.sleepSec <= 10 || c.sleepSec >= 86400))
    return "cfg: sleep must be > 10 and < 86400 seconds";
  if (c.ssid.isEmpty() != c.psk.isEmpty())
    return "cfg: SSID and PSK go together";
  if (!c.ssid.isEmpty())
  {
    if (c.ssid.length() > 32)
      return "cfg: SSID exceeds 32 chars";
    if (c.psk.length() > 63)
      return "cfg: PSK exceeds 63 chars";
    if (!isPrintableAscii(c.ssid) || !isPrintableAscii(c.psk))
      return "cfg: SSID/PSK must be printable ASCII";
  }
  if (!c.fwUrl.isEmpty() || !c.fwMd5.isEmpty() || !c.fwVer.isEmpty())
  {
    if (c.fwUrl.isEmpty() || c.fwMd5.isEmpty() || c.fwVer.isEmpty())
      return "cfg: fw requires url, md5 and ver";
    if (c.fwUrl.length() > 127)
      return "cfg: fw url exceeds 127 chars";
    if (!c.fwUrl.startsWith("http://") && !c.fwUrl.startsWith("https://"))
      return "cfg: fw url must start with http(s)://";
    if (!isHex32(c.fwMd5))
      return "cfg: fw md5 must be a 32-char hex digest";
    if (c.fwVer.length() > 24)
      return "cfg: fw ver exceeds 24 chars";
    if (!isPrintableAscii(c.fwUrl) || !isPrintableAscii(c.fwMd5) || !isPrintableAscii(c.fwVer))
      return "cfg: fw fields must be printable ASCII";
  }
  return "";
}

static void saveCfgPushToDisk()
{
  if (cfgPush.empty())
  {
    if (SPIFFS.exists(CFG_PUSH_PATH))
      SPIFFS.remove(CFG_PUSH_PATH);
    return;
  }
  DynamicJsonDocument doc(1024);
  if (!cfgPush.poly.isEmpty())
    doc["POLY"] = cfgPush.poly;
  if (cfgPush.sleepSec > 0)
    doc["sleep"] = cfgPush.sleepSec;
  if (!cfgPush.ssid.isEmpty())
  {
    doc["SSID"] = cfgPush.ssid;
    doc["PSK"] = cfgPush.psk;
  }
  if (!cfgPush.fwUrl.isEmpty())
  {
    JsonObject fw = doc.createNestedObject("fw");
    fw["url"] = cfgPush.fwUrl;
    fw["md5"] = cfgPush.fwMd5;
    fw["ver"] = cfgPush.fwVer;
  }
  File f = SPIFFS.open(CFG_PUSH_PATH, "w");
  if (f)
  {
    serializeJson(doc, f);
    f.close();
  }
}

static void loadCfgPushFromDisk()
{
  cfgPush = DeviceCfgPush();
  if (!SPIFFS.exists(CFG_PUSH_PATH))
    return;
  File f = SPIFFS.open(CFG_PUSH_PATH, "r");
  if (!f)
    return;
  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, f) == DeserializationError::Ok)
  {
    cfgPush.poly = doc["POLY"] | "";
    cfgPush.sleepSec = doc["sleep"] | 0;
    cfgPush.ssid = doc["SSID"] | "";
    cfgPush.psk = doc["PSK"] | "";
    if (doc.containsKey("fw") && doc["fw"].is<JsonObject>())
    {
      cfgPush.fwUrl = doc["fw"]["url"] | "";
      cfgPush.fwMd5 = doc["fw"]["md5"] | "";
      cfgPush.fwVer = doc["fw"]["ver"] | "";
    }
    if (!validateCfgPush(cfgPush).isEmpty())
      cfgPush = DeviceCfgPush();
  }
  f.close();
}

static void noteSpindleAcks(const String &cf, const String &fa)
{
  bool changed = false;
  if (!cf.isEmpty() && !cfgPush.ssid.isEmpty())
  {
    Serial.printf("cfg: device cf=%s -> retiring SSID/PSK push\n", cf.c_str());
    cfgPush.ssid = "";
    cfgPush.psk = "";
    cfgPush.wifiDeliveries = 0;
    changed = true;
  }
  if (!fa.isEmpty() && !cfgPush.fwUrl.isEmpty())
  {
    Serial.printf("cfg: device fa=%s -> retiring fw push\n", fa.c_str());
    cfgPush.fwUrl = "";
    cfgPush.fwMd5 = "";
    cfgPush.fwVer = "";
    cfgPush.fwDeliveries = 0;
    changed = true;
  }
  if (changed)
    saveCfgPushToDisk();
}

static String buildReceiveResponse()
{
  if (cfgPush.empty())
    return "{\"status\":\"ok\"}";

  bool changed = false;
  if (!cfgPush.ssid.isEmpty())
  {
    cfgPush.wifiDeliveries++;
    if (cfgPush.wifiDeliveries > MAX_WIFI_DELIVERIES)
    {
      cfgPush.ssid = "";
      cfgPush.psk = "";
      cfgPush.wifiDeliveries = 0;
      changed = true;
    }
  }
  if (!cfgPush.fwUrl.isEmpty())
  {
    cfgPush.fwDeliveries++;
    if (cfgPush.fwDeliveries > MAX_FW_DELIVERIES)
    {
      cfgPush.fwUrl = "";
      cfgPush.fwMd5 = "";
      cfgPush.fwVer = "";
      cfgPush.fwDeliveries = 0;
      changed = true;
    }
  }
  if (changed)
    saveCfgPushToDisk();
  if (cfgPush.empty())
    return "{\"status\":\"ok\"}";

  DynamicJsonDocument doc(1024);
  doc["status"] = "ok";
  JsonObject c = doc.createNestedObject("cfg");
  if (!cfgPush.poly.isEmpty())
    c["POLY"] = cfgPush.poly;
  if (cfgPush.sleepSec > 0)
    c["sleep"] = cfgPush.sleepSec;
  if (!cfgPush.ssid.isEmpty())
  {
    c["SSID"] = cfgPush.ssid;
    c["PSK"] = cfgPush.psk;
  }
  if (!cfgPush.fwUrl.isEmpty())
  {
    JsonObject fw = c.createNestedObject("fw");
    fw["url"] = cfgPush.fwUrl;
    fw["md5"] = cfgPush.fwMd5;
    fw["ver"] = cfgPush.fwVer;
  }
  String out;
  serializeJson(doc, out);
  return out;
}

// ---- Proxy HTTP Handlers (/cfg, /readings, /fw, /fw.bin, /exclusions) ----
static void handleCfgEndpoint()
{
  if (server.method() == HTTP_GET)
  {
    if (cfgPush.empty())
    {
      server.send(200, "application/json", "{}\n");
      return;
    }
    DynamicJsonDocument doc(1024);
    if (!cfgPush.poly.isEmpty())
      doc["POLY"] = cfgPush.poly;
    if (cfgPush.sleepSec > 0)
      doc["sleep"] = cfgPush.sleepSec;
    if (!cfgPush.ssid.isEmpty())
    {
      doc["SSID"] = cfgPush.ssid;
      doc["PSK"] = cfgPush.psk;
    }
    if (!cfgPush.fwUrl.isEmpty())
    {
      JsonObject fw = doc.createNestedObject("fw");
      fw["url"] = cfgPush.fwUrl;
      fw["md5"] = cfgPush.fwMd5;
      fw["ver"] = cfgPush.fwVer;
    }
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
    return;
  }
  if (server.method() == HTTP_PUT || server.method() == HTTP_POST)
  {
    String body = server.arg("plain");
    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, body);
    if (err || !doc.is<JsonObject>())
    {
      server.send(400, "text/plain", String("bad json: ") + err.c_str());
      return;
    }
    DeviceCfgPush next;
    next.poly = doc["POLY"] | "";
    next.sleepSec = doc["sleep"] | 0;
    next.ssid = doc["SSID"] | "";
    next.psk = doc["PSK"] | "";
    if (doc.containsKey("fw") && doc["fw"].is<JsonObject>())
    {
      next.fwUrl = doc["fw"]["url"] | "";
      next.fwMd5 = doc["fw"]["md5"] | "";
      next.fwVer = doc["fw"]["ver"] | "";
    }
    String vErr = validateCfgPush(next);
    if (!vErr.isEmpty())
    {
      server.send(400, "text/plain", vErr);
      return;
    }
    cfgPush = next;
    saveCfgPushToDisk();
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
    return;
  }
  if (server.method() == HTTP_DELETE)
  {
    cfgPush = DeviceCfgPush();
    saveCfgPushToDisk();
    server.send(200, "application/json", "{}\n");
    return;
  }
  server.send(405, "text/plain", "GET/PUT/DELETE only");
}

// Streams {"cursor":M,"readings":[{"id":..,"ts":..,"ip":"..","payload":{...}},...]}
// compatible with ProxyClient.kt in iSpindlePlotter.
static void handleReadingsSince()
{
  uint32_t since = 0;
  if (server.hasArg("since"))
    since = (uint32_t)server.arg("since").toInt();

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  String head = "{\"cursor\":" + String(globalSeqId) + ",\"readings\":[";
  server.sendContent(head);

  bool first = true;
  uint32_t fallbackSeq = 0;
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
        fallbackSeq++;
        uint32_t id = doc["id"] | fallbackSeq;
        if (id > fallbackSeq)
          fallbackSeq = id;
        if (id > since)
        {
          uint32_t tsSec = doc["ts"] | 0U;
          uint64_t tsMs = (uint64_t)tsSec * 1000ULL;
          String ip = doc["ip"] | "";
          String payloadStr;
          serializeJson(doc["payload"], payloadStr);
          String rec;
          if (!first)
            rec += ",";
          first = false;
          char tsBuf[24];
          snprintf(tsBuf, sizeof(tsBuf), "%llu", (unsigned long long)tsMs);
          rec += "{\"id\":" + String(id) + ",\"ts\":" + String(tsBuf) +
                 ",\"ip\":\"" + ip + "\",\"payload\":" + payloadStr + "}";
          server.sendContent(rec);
        }
      }
      yield();
    }
    f.close();
  }
  server.sendContent("]}");
  server.sendContent("");
}

static void handleFwInfo()
{
  if (!SPIFFS.exists(SPINDLE_FW_PATH))
  {
    server.send(404, "application/json", "{\"error\":\"no spindle firmware staged\"}");
    return;
  }
  File f = SPIFFS.open(SPINDLE_FW_PATH, "r");
  if (!f)
  {
    server.send(500, "application/json", "{\"error\":\"read failed\"}");
    return;
  }
  size_t sz = f.size();
  MD5Builder md5;
  md5.begin();
  md5.addStream(f, sz);
  md5.calculate();
  f.close();
  String out = "{\"size\":" + String((uint32_t)sz) + ",\"md5\":\"" + md5.toString() + "\"}";
  server.send(200, "application/json", out);
}

static void handleFwBinGet()
{
  if (!SPIFFS.exists(SPINDLE_FW_PATH))
  {
    server.send(404, "text/plain", "no spindle firmware staged");
    return;
  }
  File f = SPIFFS.open(SPINDLE_FW_PATH, "r");
  server.streamFile(f, "application/octet-stream");
  f.close();
}

static void handleExclusionsEndpoint()
{
  if (server.method() == HTTP_GET)
  {
    if (!SPIFFS.exists(EXCLUSIONS_PATH))
    {
      server.send(200, "application/json", "[]");
      return;
    }
    File f = SPIFFS.open(EXCLUSIONS_PATH, "r");
    server.streamFile(f, "application/json");
    f.close();
    return;
  }
  if (server.method() == HTTP_POST || server.method() == HTTP_PUT)
  {
    String body = server.arg("plain");
    DynamicJsonDocument doc(2048);
    if (deserializeJson(doc, body) != DeserializationError::Ok || !doc.is<JsonArray>())
    {
      server.send(400, "application/json", "{\"error\":\"expected json array\"}");
      return;
    }
    File f = SPIFFS.open(EXCLUSIONS_PATH, "w");
    if (!f)
    {
      server.send(500, "application/json", "{\"error\":\"write failed\"}");
      return;
    }
    serializeJson(doc, f);
    f.close();
    server.send(200, "application/json", body);
    return;
  }
  if (server.method() == HTTP_DELETE)
  {
    if (SPIFFS.exists(EXCLUSIONS_PATH))
      SPIFFS.remove(EXCLUSIONS_PATH);
    server.send(200, "application/json", "[]");
    return;
  }
  server.send(405, "text/plain", "GET/POST/PUT/DELETE only");
}

static void initMdnsProxy()
{
  if (MDNS.begin("ispindlestation"))
  {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ispindle-proxy", "tcp", 80);
    MDNS.addServiceTxt("ispindle-proxy", "tcp", "app", "ispindle-proxy");
    Serial.println("mDNS advertising _ispindle-proxy._tcp on :80");
  }
}
