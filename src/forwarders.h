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

static bool sendHttpPost(const String &hostAndPort, const String &path, const String &body, uint16_t colOk, uint16_t colWarn, uint16_t colSg)
{
  String host;
  int port = 80;
  parseHostPort(hostAndPort, host, port, 80);
  if (host.isEmpty())
    return false;
  String reqPath = path.isEmpty() ? String("/") : path;
  if (!reqPath.startsWith("/"))
    reqPath = "/" + reqPath;

  drawHeartbeatDot(colSg);
  WiFiClient c;
  c.setTimeout(3000);
  if (!c.connect(host.c_str(), port))
  {
    Serial.printf("forward connect failed %s:%d\n", host.c_str(), port);
    drawHeartbeatDot(colWarn);
    return false;
  }
  String req = "POST " + reqPath + " HTTP/1.1\r\nHost: " + host + "\r\n" +
               "User-Agent: Nautilis-iRelay\r\n" +
               "Content-Type: application/json\r\n" +
               "Content-Length: " + String(body.length()) + "\r\n" +
               "Connection: close\r\n\r\n" + body;
  c.print(req);
  unsigned long deadline = millis() + 1200UL;
  while (c.connected() && millis() < deadline)
  {
    if (c.available())
      c.readStringUntil('\n');
    delay(5);
  }
  c.stop();
  Serial.printf("forwarded %u B to %s:%d%s\n", (unsigned)body.length(),
                host.c_str(), port, reqPath.c_str());
  drawHeartbeatDot(colOk);
  return true;
}

static String injectTokenIntoJson(const String &rawBody, const String &token)
{
  if (token.isEmpty())
    return rawBody;
  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, rawBody) != DeserializationError::Ok)
    return rawBody;
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

// ---- Home Assistant MQTT Auto-Discovery ----
struct HaSensorSpec
{
  const char *id;
  const char *name;
  const char *stateSuffix;
  const char *unit;
  const char *devClass;
  const char *icon;
};

static const HaSensorSpec HA_SENSORS[17] = {
    {"rssi", "Relay WiFi Signal", "rssi", "dBm", "signal_strength", nullptr},
    {"temp", "Relay Temperature", "temp", "\xC2\xB0" "C", "temperature", nullptr},
    {"pressure", "Relay Pressure", "pressure", "bar", "pressure", nullptr},
    {"I1_rssi", "iSpindel 1 Signal", "I1_rssi", "dBm", "signal_strength", nullptr},
    {"I1_temp", "iSpindel 1 Temperature", "I1_temp", "\xC2\xB0" "C", "temperature", nullptr},
    {"I1_gravity", "iSpindel 1 Gravity", "I1_gravity", "SG", nullptr, "mdi:scale"},
    {"I1_bat", "iSpindel 1 Battery", "I1_bat", "V", "voltage", nullptr},
    {"I1_angle", "iSpindel 1 Angle", "I1_angle", "\xC2\xB0", nullptr, "mdi:angle-acute"},
    {"I1_interval", "iSpindel 1 Interval", "I1_interval", "s", nullptr, "mdi:timer-outline"},
    {"I1_OG1", "iSpindel 1 OG", "I1_OG1", "SG", nullptr, "mdi:scale-balance"},
    {"I2_rssi", "iSpindel 2 Signal", "I2_rssi", "dBm", "signal_strength", nullptr},
    {"I2_temp", "iSpindel 2 Temperature", "I2_temp", "\xC2\xB0" "C", "temperature", nullptr},
    {"I2_gravity", "iSpindel 2 Gravity", "I2_gravity", "SG", nullptr, "mdi:scale"},
    {"I2_bat", "iSpindel 2 Battery", "I2_bat", "V", "voltage", nullptr},
    {"I2_angle", "iSpindel 2 Angle", "I2_angle", "\xC2\xB0", nullptr, "mdi:angle-acute"},
    {"I2_interval", "iSpindel 2 Interval", "I2_interval", "s", nullptr, "mdi:timer-outline"},
    {"I2_OG2", "iSpindel 2 OG", "I2_OG2", "SG", nullptr, "mdi:scale-balance"}};

static void publishHaDiscovery()
{
  String devId = cfg.apSsid.isEmpty() ? String("iSpindleStation") : cfg.apSsid;
  mqttClient.setBufferSize(768);
  for (int i = 0; i < 17; i++)
  {
    const HaSensorSpec &sp = HA_SENSORS[i];
    String cfgTopic = "homeassistant/sensor/" + devId + "/" + sp.id + "/config";
    DynamicJsonDocument doc(640);
    doc["name"] = sp.name;
    doc["stat_t"] = devId + "/" + sp.stateSuffix;
    doc["uniq_id"] = devId + "_" + sp.id;
    if ( strcmp(sp.id, "temp") == 0 && cfg.tempUnit == 2 )
      doc["unit_of_meas"] = "\xC2\xB0" "F";
    else if ( strcmp(sp.id, "pressure") == 0 && cfg.pressureUnit == 2 )
      doc["unit_of_meas"] = "psi";
    else
      doc["unit_of_meas"] = sp.unit;

    if (sp.devClass)
      doc["dev_cla"] = sp.devClass;
    if (sp.icon)
      doc["ic"] = sp.icon;

    JsonObject dev = doc.createNestedObject("dev");
    JsonArray ids = dev.createNestedArray("ids");
    ids.add(devId);
    dev["name"] = devId;
    dev["mf"] = "Nautilis / iSpindleStation";
    dev["mdl"] = "iSpindel Relay";
    dev["sw"] = FIRMWARE_VERSION;

    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(cfgTopic.c_str(), payload.c_str(), true);
    yield();
  }
  mqttDiscoveryPublished = true;
}

static bool ensureMqttConnected()
{
  if (cfg.service != 9)
    return false;
  String broker = cfg.uri.isEmpty() ? cfg.serverAddress : cfg.uri;
  if (broker.isEmpty() || broker == "/")
    broker = cfg.serverAddress;
  if (broker.isEmpty() || WiFi.status() != WL_CONNECTED)
    return false;

  if (mqttClient.connected())
    return true;

  unsigned long now = millis();
  if (lastMqttReconnectAttemptMs != 0 && (now - lastMqttReconnectAttemptMs) < 10000UL)
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

static void publishMqttRelayStates()
{
  if (!ensureMqttConnected())
    return;
  String devId = cfg.apSsid.isEmpty() ? String("iSpindleStation") : cfg.apSsid;
  mqttClient.publish((devId + "/rssi").c_str(), String(WiFi.RSSI()).c_str(), true);
  if (cfg.tempYesNo == 2 && fabsf(relaySensors.temp - 127.0f) > 0.1f)
    mqttClient.publish((devId + "/temp").c_str(), String(relaySensors.temp, 2).c_str(), true);
  if (cfg.pressureUnit == 1)
    mqttClient.publish((devId + "/pressure").c_str(), String(relaySensors.bar, 2).c_str(), true);
  else if (cfg.pressureUnit == 2)
    mqttClient.publish((devId + "/pressure").c_str(), String(relaySensors.psi, 2).c_str(), true);
}

static void publishMqttSpindleSlot(int slot)
{
  if (!ensureMqttConnected())
    return;
  String devId = cfg.apSsid.isEmpty() ? String("iSpindleStation") : cfg.apSsid;
  int idx = (slot == 2) ? 2 : 1;
  String pfx = devId + "/I" + String(idx) + "_";
  SlotState &st = slots[slot];
  mqttClient.publish((pfx + "rssi").c_str(), String(st.rssi).c_str(), true);
  mqttClient.publish((pfx + "temp").c_str(), String(st.corrTemp, 2).c_str(), true);
  mqttClient.publish((pfx + "gravity").c_str(), String(st.corrSg, 4).c_str(), true);
  mqttClient.publish((pfx + "bat").c_str(), String(st.batt, 2).c_str(), true);
  mqttClient.publish((pfx + "angle").c_str(), String(st.angle, 2).c_str(), true);
  mqttClient.publish((pfx + "interval").c_str(), String(st.interval).c_str(), true);
  String ogKey = (idx == 2) ? "OG2" : "OG1";
  mqttClient.publish((pfx + ogKey).c_str(), String(cfg.og[idx], 4).c_str(), true);
  publishMqttRelayStates();
}

static String buildTapboardPayload(int slot, const String &rawBody)
{
  DynamicJsonDocument doc(1024);
  if (slot >= 0)
  {
    deserializeJson(doc, rawBody);
    int cfgIdx = (slot == 2) ? 2 : 1;
    doc["temp_korekcija"] = cfg.tempOffset[cfgIdx];
    doc["grav_korekcija"] = cfg.sgOffset[cfgIdx];
    doc["OG"] = cfg.og[cfgIdx];
    doc["ABV"] = slots[slot].abv;
    doc["atten"] = slots[slot].attenuation;
    doc["alarm"] = (slots[slot].alarmSg || slots[slot].alarmTemp) ? 1 : 0;
  }
  else
  {
    doc["name"] = "Nautilis_Relay";
  }
  doc["temp_relay"] = (cfg.tempYesNo == 2) ? relaySensors.temp : 127.0f;
  doc["temp_corr_relay"] = cfg.tempCorrection1;
  doc["pressure"] = (cfg.pressureUnit == 2) ? relaySensors.psi : relaySensors.bar;
  doc["pressure_unit"] = cfg.pressureUnit;
  String out;
  serializeJson(doc, out);
  return out;
}

// ---- On-Receipt iSpindel Forwarder ----
static void forwardOnReceipt(int slot, const String &body, uint16_t colOk, uint16_t colWarn, uint16_t colSg)
{
  if (cfg.service == 255 || WiFi.status() != WL_CONNECTED)
    return;

  const String &name = slots[slot].name;
  switch (cfg.service)
  {
  case 0: // UbiDots
    if (!cfg.token.isEmpty())
      sendHttpPost("industrial.api.ubidots.com",
                   "/api/v1.6/devices/" + name + "?token=" + cfg.token,
                   body, colOk, colWarn, colSg);
    break;

  case 1: // Brewfather
    if (!cfg.uri.isEmpty())
      sendHttpPost("log.brewfather.net", cfg.uri, body, colOk, colWarn, colSg);
    break;

  case 2: // BrewSpy
    if (!cfg.uri.isEmpty())
      sendHttpPost("brew-spy.com", cfg.uri, injectTokenIntoJson(body, cfg.token), colOk, colWarn, colSg);
    break;

  case 3: // Monitor.Beer
    if (!cfg.uri.isEmpty())
      sendHttpPost("Monitor.beer", cfg.uri, injectTokenIntoJson(body, cfg.token), colOk, colWarn, colSg);
    break;

  case 4: // Grainfather (prefix '1' -> grainfatherUri1, '2' -> grainfatherUri2)
  {
    String gfUri;
    if (name.startsWith("2") || slot == 2)
      gfUri = cfg.grainfatherUri2;
    else
      gfUri = cfg.grainfatherUri1.isEmpty() ? cfg.uri : cfg.grainfatherUri1;
    if (!gfUri.isEmpty())
      sendHttpPost("community.grainfather.com", gfUri, body, colOk, colWarn, colSg);
    break;
  }

  case 5: // BrewPiless (rewrites "name":"iSpindel000")
    if (!cfg.serverAddress.isEmpty())
      sendHttpPost(cfg.serverAddress, "/gravity", rewriteNameInJson(body, "iSpindel000"), colOk, colWarn, colSg);
    break;

  case 6: // Little Bock (prefix '1' -> littlebockUri1, '2' -> littlebockUri2)
  {
    String lbUri = (name.startsWith("2") || slot == 2) ? cfg.littlebockUri2 : cfg.littlebockUri1;
    if (!lbUri.isEmpty())
      sendHttpPost("www.littlebock.fr", lbUri, body, colOk, colWarn, colSg);
    break;
  }

  case 7: // Brewers Friend
    if (!cfg.uri.isEmpty())
      sendHttpPost("log.brewersfriend.com", cfg.uri, body, colOk, colWarn, colSg);
    break;

  case 8: // BrewBench
    if (!cfg.uri.isEmpty())
      sendHttpPost("sensor.brewbench.co", cfg.uri + cfg.token, body, colOk, colWarn, colSg);
    break;

  case 9: // MQTT + Home Assistant Auto-Discovery
    publishMqttSpindleSlot(slot);
    break;

  case 10: // Generic HTTP (accepts host:port)
    if (!cfg.serverAddress.isEmpty())
      sendHttpPost(cfg.serverAddress, cfg.uri, injectTokenIntoJson(body, cfg.token), colOk, colWarn, colSg);
    break;

  case 11: // NAUTILIS Tapboard
    if (!cfg.token.isEmpty())
    {
      bool ok = sendHttpPost("tapboard.nautilis.eu", "/receive.php?token=" + cfg.token,
                             buildTapboardPayload(slot, body), colOk, colWarn, colSg);
      if (ok)
        tapboardFailCount = 0;
      else if (++tapboardFailCount >= 50)
        ESP.restart();
    }
    break;
  }
}

// ---- Periodic Relay Sensor Sender min) ----
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

  float tVal = (cfg.tempYesNo == 2 && fabsf(relaySensors.temp - 127.0f) > 0.1f) ? relaySensors.temp : 0.0f;
  float pVal = (cfg.pressureUnit == 2) ? relaySensors.psi : relaySensors.bar;
  const char *tUnit = (cfg.tempUnit == 2) ? "F" : "C";

  switch (cfg.service)
  {
  case 0: // UbiDots Nautilis_Thermometer
    if (!cfg.token.isEmpty())
    {
      String j = "{\"temperature\":" + String(tVal, 2) + ",\"pressure\":" + String(pVal, 2) + "}";
      sendHttpPost("industrial.api.ubidots.com",
                   "/api/v1.6/devices/Nautilis_Thermometer?token=" + cfg.token,
                   j, colOk, colWarn, colSg);
    }
    break;

  case 1: // Brewfather Custom Stream (uri_custom)
    if (!cfg.uriCustom.isEmpty())
    {
      String pUnit = (cfg.pressureUnit == 2) ? "PSI" : "BAR";
      String j = "{\"name\":\"Nautilis\",\"temp\":" + String(tVal, 2) +
                 ",\"temp_unit\":\"" + String(tUnit) + "\",\"pressure\":" +
                 String(pVal, 2) + ",\"pressure_unit\":\"" + pUnit + "\"}";
      sendHttpPost("log.brewfather.net", cfg.uriCustom, j, colOk, colWarn, colSg);
    }
    break;

  case 2: // BrewSpy (Nautilis-iRelay)
    if (!cfg.uri.isEmpty())
    {
      String j = "{\"device\":\"Nautilis-iRelay\",\"token\":\"" + cfg.token +
                 "\",\"temperature\":" + String(tVal, 2) +
                 ",\"pressure\":" + String(pVal, 2) + "}";
      sendHttpPost("brew-spy.com", cfg.uri, j, colOk, colWarn, colSg);
    }
    break;

  case 4: // Grainfather Custom URL (uri_custom)
    if (!cfg.uriCustom.isEmpty())
    {
      String uName = (cfg.tempUnit == 2) ? "fahrenheit" : "celsius";
      String j = "{\"specific-gravity\":0,\"temperature\":" + String(tVal, 2) +
                 ",\"unit\":\"" + uName + "\"}";
      sendHttpPost("community.grainfather.com", cfg.uriCustom, j, colOk, colWarn, colSg);
    }
    break;

  case 9: // MQTT
    publishMqttRelayStates();
    break;

  case 10: // Generic HTTP (uri_custom)
    if (!cfg.serverAddress.isEmpty() && !cfg.uriCustom.isEmpty())
    {
      String j = "{\"name\":\"Nautilis_Thermometer\",\"token\":\"" + cfg.token +
                 "\",\"temperature\":" + String(tVal, 2) +
                 ",\"temp_units\":\"" + String(tUnit) +
                 "\",\"pressure\":" + String(pVal, 2) + "}";
      sendHttpPost(cfg.serverAddress, cfg.uriCustom, j, colOk, colWarn, colSg);
    }
    break;

  case 11: // NAUTILIS Tapboard
    if (!cfg.token.isEmpty())
    {
      sendHttpPost("tapboard.nautilis.eu", "/receive.php?token=" + cfg.token,
                   buildTapboardPayload(-1, "{}"), colOk, colWarn, colSg);
    }
    break;
  }
}
