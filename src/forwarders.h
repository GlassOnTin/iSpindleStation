#pragma once

#include "state.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

static void drawHeartbeatDot(uint16_t col);

static WiFiClient mqttNetClient;
static PubSubClient mqttClient(mqttNetClient);
static bool mqttDiscoveryPublished = false;
static unsigned long lastMqttReconnectAttemptMs = 0;
static unsigned long lastPeriodicRelaySendMs = 0;
static uint8_t tapboardFailCount = 0;

// Optional TCP target override ("host:port") used by /api/test_forward so all 12
// services can be tested live on the ESP8266 against a mock server while keeping
// their exact HTTP Host: header, User-Agent:, path, and JSON payload.
static String mockHttpConnectOverride = "";
static String lastForwardStatus = "Waiting for iSpindel data";
static unsigned long lastForwardAttemptMs = 0;
static unsigned long lastForwardSuccessMs = 0;

static void parseHostPort(const String &input, String &hostOut, int &portOut, int defaultPort = 80)
{
  hostOut = input;
  portOut = defaultPort;
  int colon = hostOut.lastIndexOf(':');
  if (colon > 0)
  {
    int p = hostOut.substring(colon + 1).toInt();
    if (p > 0 && p <= 65535)
    {
      portOut = p;
      hostOut = hostOut.substring(0, colon);
    }
  }
}

static bool sendHttpPost(const String &hostAndPort, const String &path, const String &body,
                         uint16_t colOk, uint16_t colWarn, uint16_t colSg,
                         const char *userAgent = "iSpindel", bool includeAcceptAll = false)
{
  String logicalHost;
  int logicalPort = 80;
  parseHostPort(hostAndPort, logicalHost, logicalPort, 80);
  if (logicalHost.isEmpty())
    return false;

  String connectHost = logicalHost;
  int connectPort = logicalPort;
  if (!mockHttpConnectOverride.isEmpty())
    parseHostPort(mockHttpConnectOverride, connectHost, connectPort, 80);

  String reqPath = path.isEmpty() ? String("/") : path;
  if (!reqPath.startsWith("/"))
    reqPath = "/" + reqPath;

  lastForwardAttemptMs = millis();
  drawHeartbeatDot(colSg);
  WiFiClient c;
  c.setTimeout(3000);
  if (!c.connect(connectHost.c_str(), connectPort))
  {
    Serial.printf("forward connect failed %s:%d (logical %s:%d)\n",
                  connectHost.c_str(), connectPort, logicalHost.c_str(), logicalPort);
    lastForwardStatus = "Connect failed: " + logicalHost;
    drawHeartbeatDot(colWarn);
    return false;
  }

  String req = "POST " + reqPath + " HTTP/1.1\r\nHost: " + logicalHost + "\r\n";
  if (includeAcceptAll)
    req += "Accept: */*\r\n";
  req += "User-Agent: " + String(userAgent) + "\r\n" +
         "Connection: close\r\n" +
         "Content-Type: application/json\r\n" +
         "Content-Length: " + String(body.length()) + "\r\n\r\n" + body;

  c.print(req);
  unsigned long deadline = millis() + 1200UL;
  while (c.connected() && millis() < deadline)
  {
    if (c.available())
      c.readStringUntil('\n');
    delay(5);
  }
  c.stop();
  lastForwardSuccessMs = millis();
  lastForwardStatus = "Success (" + logicalHost + ")";
  Serial.printf("forwarded %u B to %s:%d%s\n", (unsigned)body.length(),
                logicalHost.c_str(), logicalPort, reqPath.c_str());
  drawHeartbeatDot(colOk);
  return true;
}

static String injectTokenIntoJson(const String &rawBody, const String &token)
{
  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, rawBody) != DeserializationError::Ok)
    return rawBody;
  if (!token.isEmpty())
    doc["token"] = token;
  String out;
  serializeJson(doc, out);
  return out;
}

static String rewriteNameInJson(const String &rawBody, const char *newName)
{
  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, rawBody) != DeserializationError::Ok)
    return rawBody;
  doc["name"] = newName;
  String out;
  serializeJson(doc, out);
  return out;
}

// ---- Home Assistant MQTT Auto-Discovery (17 sensors) ----
struct HaSensorSpec
{
  const char *cfgSuffix;
  const char *uniqSuffix;
  const char *name;
  const char *stateSuffix;
  const char *unit;
  const char *devClass;
  uint16_t expireAfter;
  uint8_t slotFilter; // 0 = always, 2 = only when ispindelNum == 2
};

static const HaSensorSpec HA_SENSORS[17] = {
    {"temp", "0", "iRelay temp", "temp", "\xC2\xB0" "C", "temperature", 900, 0},
    {"rssi", "1", nullptr, "rssi", nullptr, "signal_strength", 900, 0},
    {"pressure", "2", "Pressure", "pressure", "bar", "pressure", 900, 0},
    {"I1_temp", "3", "iSpindel 1 temp", "I1_temp", "\xC2\xB0" "C", "temperature", 3000, 0},
    {"I1_gravity", "4", "iSpindel 1 gravity", "I1_gravity", nullptr, nullptr, 3000, 0},
    {"I1_OG1", "12", "iSpindel 1 OG", "I1_OG1", nullptr, nullptr, 3000, 0},
    {"I1rssi", "5", "iSpindel 1 RSSI", "I1_rssi", nullptr, "signal_strength", 3000, 0},
    {"I1_bat", "6", "iSpindel 1 Battery", "I1_bat", "V", "battery", 3000, 0},
    {"I1_angle", "7", "iSpindel 1 angle", "I1_angle", "\xC2\xB0", nullptr, 3000, 0},
    {"I1_interval", "16", "iSpindel 1 interval", "I1_interval", nullptr, nullptr, 3000, 0},
    {"I2_temp", "8", "iSpindel 2 temp", "I2_temp", "\xC2\xB0" "C", "temperature", 3000, 2},
    {"I2_gravity", "9", "iSpindel 2 gravity", "I2_gravity", nullptr, nullptr, 3000, 2},
    {"I2_OG2", "13", "iSpindel 2 OG", "I2_OG2", nullptr, nullptr, 3000, 2},
    {"I2rssi", "10", "iSpindel 2 RSSI", "I2_rssi", nullptr, "signal_strength", 3000, 2},
    {"I2_bat", "11", "iSpindel 2 Battery", "I2_bat", "V", "battery", 3000, 2},
    {"I2_angle", "14", "iSpindel 2 angle", "I2_angle", "\xC2\xB0", nullptr, 3000, 2},
    {"I2_interval", "17", "iSpindel 2 interval", "I2_interval", nullptr, nullptr, 3000, 2}};

static void publishHaDiscovery()
{
  String devId = cfg.apSsid.isEmpty() ? String("nautilis-ispindel") : cfg.apSsid;
  mqttClient.setBufferSize(768);
  for (int i = 0; i < 17; i++)
  {
    const HaSensorSpec &sp = HA_SENSORS[i];
    if (sp.slotFilter == 2 && cfg.ispindelNum < 2)
      continue;

    String cfgTopic = "homeassistant/sensor/" + devId + sp.cfgSuffix + "/config";
    DynamicJsonDocument doc(640);
    if (sp.name)
      doc["name"] = sp.name;
    if (sp.devClass)
      doc["dev_cla"] = sp.devClass;
    if (strcmp(sp.cfgSuffix, "rssi") != 0 && strcmp(sp.cfgSuffix, "I1rssi") != 0 && strcmp(sp.cfgSuffix, "I2rssi") != 0)
      doc["state_class"] = "measurement";

    if (strcmp(sp.cfgSuffix, "temp") == 0 || strcmp(sp.cfgSuffix, "I1_temp") == 0 || strcmp(sp.cfgSuffix, "I2_temp") == 0)
      doc["unit_of_meas"] = (cfg.tempUnit == 2) ? "\xC2\xB0" "F" : "\xC2\xB0" "C";
    else if (strcmp(sp.cfgSuffix, "pressure") == 0)
      doc["unit_of_meas"] = (cfg.pressureUnit == 2) ? "psi" : "bar";
    else if (sp.unit)
      doc["unit_of_meas"] = sp.unit;

    doc["unique_id"] = devId + sp.uniqSuffix;
    doc["state_topic"] = devId + "/" + sp.stateSuffix;
    doc["expire_after"] = sp.expireAfter;

    JsonObject dev = doc.createNestedObject("device");
    JsonArray ids = dev.createNestedArray("ids");
    ids.add(devId + "/device");
    dev["name"] = devId + "/device";
    dev["mf"] = "Nautilis";
    dev["mdl"] = "iRelay";
    dev["sw"] = FIRMWARE_VERSION;

    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(cfgTopic.c_str(), payload.c_str(), true);
    yield();
  }
  mqttDiscoveryPublished = true;
}

static bool ensureMqttConnected(bool forceImmediate = false)
{
  if (cfg.service != 9)
    return false;
  String broker = cfg.uri.isEmpty() ? cfg.serverAddress : cfg.uri;
  if (broker.isEmpty() || broker == "/")
    broker = cfg.serverAddress;
  if (broker.isEmpty() || WiFi.status() != WL_CONNECTED)
    return false;

  if (mqttClient.connected() && !forceImmediate)
    return true;

  unsigned long now = millis();
  if (!forceImmediate && lastMqttReconnectAttemptMs != 0 && (now - lastMqttReconnectAttemptMs) < 10000UL)
    return false;
  lastMqttReconnectAttemptMs = now;

  String host;
  int port = 1883;
  parseHostPort(broker, host, port, 1883);
  mqttClient.setServer(host.c_str(), port);
  mqttClient.setBufferSize(768);

  String clientId = "iSpindleStation-" + String(ESP.getChipId(), HEX);
  bool ok = false;
  if (!cfg.mqttUser.isEmpty())
    ok = mqttClient.connect(clientId.c_str(), cfg.mqttUser.c_str(), cfg.mqttPass.c_str());
  else
    ok = mqttClient.connect(clientId.c_str());

  if (ok)
  {
    Serial.printf("MQTT connected to %s:%d\n", host.c_str(), port);
    publishHaDiscovery();
  }
  return ok;
}

static void publishMqttRelayStates(bool forceConnect = false)
{
  if (!ensureMqttConnected(forceConnect))
    return;
  String devId = cfg.apSsid.isEmpty() ? String("nautilis-ispindel") : cfg.apSsid;
  String rssiStr = String(WiFi.RSSI());
  mqttClient.publish((devId + "/relayrssi").c_str(), rssiStr.c_str(), true);
  mqttClient.publish((devId + "/rssi").c_str(), rssiStr.c_str(), true);
  if (cfg.tempYesNo == 2 && fabsf(relaySensors.temp - 127.0f) > 0.1f)
  {
    float tOut = (cfg.tempUnit == 2) ? (relaySensors.temp * 1.8f + 32.0f) : relaySensors.temp;
    mqttClient.publish((devId + "/temp").c_str(), String(tOut, 2).c_str(), true);
  }
  if (cfg.pressureUnit == 1)
    mqttClient.publish((devId + "/pressure").c_str(), String(relaySensors.bar, 2).c_str(), true);
  else if (cfg.pressureUnit == 2)
    mqttClient.publish((devId + "/pressure").c_str(), String(relaySensors.psi, 2).c_str(), true);
  mqttClient.loop();
  lastForwardSuccessMs = millis();
  lastForwardStatus = "Success (MQTT)";
}

static void publishMqttSpindleSlot(int slot, bool forceConnect = false)
{
  if (!ensureMqttConnected(forceConnect))
    return;
  String devId = cfg.apSsid.isEmpty() ? String("nautilis-ispindel") : cfg.apSsid;
  int idx = (slot == 2) ? 2 : 1;
  String pfx = devId + "/I" + String(idx) + "_";
  SlotState &st = slots[slot];

  float tOut = st.corrTemp;
  if (cfg.tempUnit == 2 && st.tempUnit != 'F' && st.tempUnit != 'f')
    tOut = st.corrTemp * 1.8f + 32.0f;
  else if (cfg.tempUnit == 1 && (st.tempUnit == 'F' || st.tempUnit == 'f'))
    tOut = (st.corrTemp - 32.0f) / 1.8f;

  int gDec = (cfg.unit[idx] == 2) ? 2 : 3;
  float gVal = (cfg.unit[idx] == 2) ? sgToPlato(st.corrSg) : ensureSg(st.corrSg);

  mqttClient.publish((pfx + "rssi").c_str(), String(st.rssi).c_str(), true);
  mqttClient.publish((pfx + "temp").c_str(), String(tOut, 2).c_str(), true);
  mqttClient.publish((pfx + "gravity").c_str(), String(gVal, gDec).c_str(), true);
  mqttClient.publish((pfx + "bat").c_str(), String(st.batt, 2).c_str(), true);
  mqttClient.publish((pfx + "angle").c_str(), String(st.angle, 2).c_str(), true);
  mqttClient.publish((pfx + "interval").c_str(), String(st.interval).c_str(), true);
  if (st.ogLocked || cfg.og[idx] > 0.0f)
  {
    String ogKey = (idx == 2) ? "OG2" : "OG1";
    float ogVal = (cfg.unit[idx] == 2) ? sgToPlato(cfg.og[idx]) : ensureSg(cfg.og[idx]);
    mqttClient.publish((pfx + ogKey).c_str(), String(ogVal, gDec).c_str(), true);
  }
  publishMqttRelayStates(false);
}

// ---- Periodic NAUTILIS Tapboard Relay Sensor Payload ----
static String buildTapboardPeriodicPayload()
{
  DynamicJsonDocument doc(512);
  if (cfg.tempYesNo == 2)
  {
    float tOut = (cfg.tempUnit == 2) ? (relaySensors.temp * 1.8f + 32.0f) : relaySensors.temp;
    doc["temp"] = roundf(tOut * 10.0f) / 10.0f;
    doc["temp_unit"] = (cfg.tempUnit == 2) ? "F" : "C";
    doc["alarm_temp1_low"] = cfg.alarmRelayTempLow;
    doc["alarm_temp1_high"] = cfg.alarmRelayTempHigh;
  }
  if (cfg.pressureUnit == 1)
  {
    doc["pressure"] = relaySensors.bar;
    doc["pressure_unit"] = "BAR";
  }
  else if (cfg.pressureUnit == 2)
  {
    doc["pressure"] = relaySensors.psi;
    doc["pressure_unit"] = "PSI";
  }
  doc["alarm_low"] = cfg.pressureAlarmLow;
  doc["alarm_high"] = cfg.pressureAlarmHigh;
  String out;
  serializeJson(doc, out);
  return out;
}

// ---- On-Receipt iSpindel Forwarder (all 12 services) ----
static bool forwardOnReceipt(int slot, const String &body, uint16_t colOk, uint16_t colWarn, uint16_t colSg, bool forceMqtt = false)
{
  if (cfg.service == 255 || WiFi.status() != WL_CONNECTED)
    return false;

  const String &name = slots[slot].name;
  switch (cfg.service)
  {
  case 0: // UbiDots
    if (!cfg.token.isEmpty())
      return sendHttpPost("industrial.api.ubidots.com",
                          "/api/v1.6/devices/" + name + "?token=" + cfg.token,
                          injectTokenIntoJson(body, ""), colOk, colWarn, colSg, "ESP8266", true);
    lastForwardStatus = "Skipped: UbiDots token missing";
    return false;

  case 1: // Brewfather
    if (!cfg.uri.isEmpty())
      return sendHttpPost("log.brewfather.net", cfg.uri,
                          injectTokenIntoJson(body, ""), colOk, colWarn, colSg, "iSpindel");
    lastForwardStatus = "Skipped: Brewfather URI missing";
    return false;

  case 2: // BrewSpy (default path /api/ispindel)
  {
    String bsPath = (cfg.uri.isEmpty() || cfg.uri == "/") ? String("/api/ispindel") : cfg.uri;
    return sendHttpPost("brew-spy.com", bsPath,
                        injectTokenIntoJson(body, cfg.token), colOk, colWarn, colSg, "iSpindel");
  }

  case 3: // Monitor.Beer
    if (!cfg.uri.isEmpty())
      return sendHttpPost("Monitor.beer", cfg.uri,
                          injectTokenIntoJson(body, cfg.token), colOk, colWarn, colSg, "iSpindel");
    lastForwardStatus = "Skipped: Monitor.Beer URI missing";
    return false;

  case 4: // Grainfather ('1' -> grainfatherUri1, '2' -> grainfatherUri2)
  {
    String gfUri;
    if (name.startsWith("1"))
      gfUri = cfg.grainfatherUri1;
    else if (name.startsWith("2"))
      gfUri = cfg.grainfatherUri2;
    else
    {
      lastForwardStatus = "Skipped: Grainfather name invalid";
      return false;
    }
    if (!gfUri.isEmpty())
      return sendHttpPost("community.grainfather.com", gfUri,
                          injectTokenIntoJson(body, ""), colOk, colWarn, colSg, "iSpindel");
    lastForwardStatus = "Skipped: Grainfather URI missing";
    return false;
  }

  case 5: // BrewPiless (default 192.168.88.55, rewrites "name":"iSpindel000")
  {
    String bplHost = cfg.serverAddress.isEmpty() ? String("192.168.88.55") : cfg.serverAddress;
    return sendHttpPost(bplHost, "/gravity",
                        rewriteNameInJson(body, "iSpindel000"), colOk, colWarn, colSg, "iSpindel");
  }

  case 6: // Little Bock ('1' -> littlebockUri1, '2' -> littlebockUri2)
  {
    String lbUri;
    if (name.startsWith("1"))
      lbUri = cfg.littlebockUri1;
    else if (name.startsWith("2"))
      lbUri = cfg.littlebockUri2;
    else
    {
      lastForwardStatus = "Skipped: LittleBock name invalid";
      return false;
    }
    if (!lbUri.isEmpty())
      return sendHttpPost("www.littlebock.fr", lbUri,
                          injectTokenIntoJson(body, ""), colOk, colWarn, colSg, "iSpindel");
    lastForwardStatus = "Skipped: LittleBock URI missing";
    return false;
  }

  case 7: // Brewers Friend
    if (!cfg.uri.isEmpty())
      return sendHttpPost("log.brewersfriend.com", cfg.uri,
                          injectTokenIntoJson(body, ""), colOk, colWarn, colSg, "iSpindel");
    lastForwardStatus = "Skipped: BrewersFriend URI missing";
    return false;

  case 8: // BrewBench (concatenates uri + TOKEN)
    if (!cfg.uri.isEmpty())
      return sendHttpPost("sensor.brewbench.co", cfg.uri + cfg.token,
                          injectTokenIntoJson(body, ""), colOk, colWarn, colSg, "iSpindel");
    lastForwardStatus = "Skipped: BrewBench URI missing";
    return false;

  case 9: // MQTT + Home Assistant Auto-Discovery
    publishMqttSpindleSlot(slot, forceMqtt);
    return mqttClient.connected();

  case 10: // Generic HTTP
    if (!cfg.serverAddress.isEmpty())
      return sendHttpPost(cfg.serverAddress, cfg.uri,
                          injectTokenIntoJson(body, cfg.token), colOk, colWarn, colSg, "iSpindel");
    lastForwardStatus = "Skipped: HTTP address missing";
    return false;

  case 11: // NAUTILIS Tapboard
    if (!cfg.token.isEmpty())
    {
      bool ok = sendHttpPost("tapboard.nautilis.eu", "/receive.php?token=" + cfg.token,
                             body, colOk, colWarn, colSg, "Nautilis-iRelay");
      if (ok)
        tapboardFailCount = 0;
      else if (mockHttpConnectOverride.isEmpty() && ++tapboardFailCount >= 50)
        ESP.restart();
      return ok;
    }
    lastForwardStatus = "Skipped: Tapboard token missing";
    return false;
  }
  return false;
}

// ---- Execute One Periodic Relay Sensor Upload ----
static bool sendPeriodicRelaySensor(uint16_t colOk, uint16_t colWarn, uint16_t colSg, bool forceMqtt = false)
{
  bool hasSensor = (cfg.tempYesNo == 2) || (cfg.pressureUnit == 1) || (cfg.pressureUnit == 2);
  if (!hasSensor || cfg.service == 255 || WiFi.status() != WL_CONNECTED)
    return false;

  float tVal = 0.0f;
  if (cfg.tempYesNo == 2 && fabsf(relaySensors.temp - 127.0f) > 0.1f)
    tVal = (cfg.tempUnit == 2) ? (relaySensors.temp * 1.8f + 32.0f) : relaySensors.temp;

  const char *tUnitShort = (cfg.tempUnit == 2) ? "F" : "C";
  const char *tUnitLong = (cfg.tempUnit == 2) ? "fahrenheit" : "celsius";
  String devId = cfg.apSsid.isEmpty() ? String("nautilis-ispindel") : cfg.apSsid;

  switch (cfg.service)
  {
  case 0: // UbiDots Nautilis_Thermometer
  {
    String tok = cfg.uriCustom.isEmpty() ? cfg.token : cfg.uriCustom;
    if (!tok.isEmpty() && cfg.tempYesNo == 2)
    {
      DynamicJsonDocument doc(256);
      doc["temperature"] = tVal;
      String j;
      serializeJson(doc, j);
      return sendHttpPost("industrial.api.ubidots.com",
                          "/api/v1.6/devices/Nautilis_Thermometer?token=" + tok,
                          j, colOk, colWarn, colSg, "ESP8266", true);
    }
    return false;
  }

  case 1: // Brewfather Custom Stream
    if (!cfg.uriCustom.isEmpty() && cfg.tempYesNo == 2)
    {
      DynamicJsonDocument doc(256);
      doc["name"] = devId;
      doc["ext_temp"] = tVal;
      doc["temp_unit"] = tUnitShort;
      if (cfg.pressureUnit == 1)
      {
        doc["pressure"] = relaySensors.bar;
        doc["pressure_unit"] = "BAR";
      }
      else if (cfg.pressureUnit == 2)
      {
        doc["pressure"] = relaySensors.psi;
        doc["pressure_unit"] = "PSI";
      }
      String j;
      serializeJson(doc, j);
      return sendHttpPost("log.brewfather.net", cfg.uriCustom, j, colOk, colWarn, colSg, "iSpindel");
    }
    return false;

  case 2: // BrewSpy /api/nautilis
  {
    String tok = cfg.uriCustom.isEmpty() ? cfg.token : cfg.uriCustom;
    if (!tok.isEmpty() && cfg.tempYesNo == 2)
    {
      DynamicJsonDocument doc(256);
      doc["token"] = tok;
      doc["temperature"] = tVal;
      doc["unit"] = tUnitLong;
      if (cfg.pressureUnit == 1)
      {
        doc["pressure"] = relaySensors.bar;
        doc["pressureUnit"] = "bar";
      }
      else if (cfg.pressureUnit == 2)
      {
        doc["pressure"] = relaySensors.psi;
        doc["pressureUnit"] = "psi";
      }
      String j;
      serializeJson(doc, j);
      return sendHttpPost("brew-spy.com", "/api/nautilis", j, colOk, colWarn, colSg, "iSpindel");
    }
    return false;
  }

  case 4: // Grainfather Custom Stream
    if (!cfg.uriCustom.isEmpty() && cfg.tempYesNo == 2)
    {
      DynamicJsonDocument doc(256);
      doc["specific_gravity"] = 0;
      doc["temperature"] = tVal;
      doc["unit"] = tUnitLong;
      String j;
      serializeJson(doc, j);
      return sendHttpPost("community.grainfather.com", cfg.uriCustom, j, colOk, colWarn, colSg, "iSpindel");
    }
    return false;

  case 9: // MQTT
    publishMqttRelayStates(forceMqtt);
    return mqttClient.connected();

  case 10: // Generic HTTP
  {
    String path = cfg.uriCustom.isEmpty() ? cfg.uri : cfg.uriCustom;
    if (!cfg.serverAddress.isEmpty() && !path.isEmpty() && cfg.tempYesNo == 2)
    {
      DynamicJsonDocument doc(256);
      doc["ext_temp"] = tVal;
      doc["temp_unit"] = tUnitShort;
      if (cfg.pressureUnit == 1)
      {
        doc["pressure"] = relaySensors.bar;
        doc["pressure_unit"] = "BAR";
      }
      else if (cfg.pressureUnit == 2)
      {
        doc["pressure"] = relaySensors.psi;
        doc["pressure_unit"] = "PSI";
      }
      String j;
      serializeJson(doc, j);
      return sendHttpPost(cfg.serverAddress, path, j, colOk, colWarn, colSg, "iSpindel");
    }
    return false;
  }

  case 11: // NAUTILIS Tapboard
    if (!cfg.token.isEmpty())
    {
      return sendHttpPost("tapboard.nautilis.eu", "/receive.php?token=" + cfg.token,
                          buildTapboardPeriodicPayload(), colOk, colWarn, colSg, "Nautilis-iRelay");
    }
    return false;
  }
  return false;
}

// ---- Periodic Relay Sensor Loop Hook ----
static void updatePeriodicRelaySender(uint16_t colOk, uint16_t colWarn, uint16_t colSg)
{
  if (cfg.service == 9)
    mqttClient.loop();

  bool hasSensor = (cfg.tempYesNo == 2) || (cfg.pressureUnit == 1) || (cfg.pressureUnit == 2);
  if (!hasSensor || cfg.service == 255 || WiFi.status() != WL_CONNECTED)
    return;

  unsigned long periodMin = (cfg.customPeriod >= 10) ? (unsigned long)cfg.customPeriod : 10UL;
  unsigned long periodMs = periodMin * 60000UL;
  unsigned long now = millis();
  if (lastPeriodicRelaySendMs != 0 && (now - lastPeriodicRelaySendMs) < periodMs)
    return;
  lastPeriodicRelaySendMs = now;

  sendPeriodicRelaySensor(colOk, colWarn, colSg, false);
}
