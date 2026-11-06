#pragma once

#include "state.h"
#include <OneWire.h>
#include <DallasTemperature.h>

static OneWire oneWireBus(ONEWIRE_PIN);
static DallasTemperature ds18b20(&oneWireBus);

static unsigned long lastAlarmBeepMs = 0;
static unsigned long toneEndMs = 0;
static unsigned long lastTempPollMs = 0;
static unsigned long lastPressurePollMs = 0;
static bool dsConversionPending = false;
static unsigned long dsConversionStartedMs = 0;

static void initSensorsAndBuzzer()
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  ds18b20.begin();
  ds18b20.setResolution(11);
  ds18b20.setWaitForConversion(false);
}

static void beepSettingsSaved()
{
  if (cfg.buzzYesNo == 2)
  {
    Serial.println("Beep: settings saved");
    tone(BUZZER_PIN, 1800, 1000);
    toneEndMs = millis() + 1020;
  }
}

static void beepAlarm()
{
  if (cfg.buzzYesNo != 2)
    return;
  unsigned long now = millis();
  if (lastAlarmBeepMs == 0 || (now - lastAlarmBeepMs) >= 5000UL)
  {
    lastAlarmBeepMs = now;
    Serial.println("Biiiiip Alarm");
    tone(BUZZER_PIN, 2000, 1000);
    toneEndMs = now + 1020;
  }
}

static void updateBuzzerPin()
{
  if (toneEndMs > 0 && millis() >= toneEndMs)
  {
    noTone(BUZZER_PIN);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    toneEndMs = 0;
  }
}

static void pollRelayTemperature()
{
  if (cfg.tempYesNo != 2)
  {
    relaySensors.temp = 127.0f;
    relaySensors.tempAlarm = false;
    return;
  }

  unsigned long now = millis();
  if (!dsConversionPending && (lastTempPollMs == 0 || (now - lastTempPollMs) >= 10000UL))
  {
    lastTempPollMs = now;
    ds18b20.requestTemperatures();
    dsConversionPending = true;
    dsConversionStartedMs = now;
    return;
  }

  if (dsConversionPending && (now - dsConversionStartedMs) >= 400UL)
  {
    dsConversionPending = false;
    float raw = (cfg.tempUnit == 2) ? ds18b20.getTempFByIndex(0) : ds18b20.getTempCByIndex(0);
    float resetVal = (cfg.tempUnit == 2) ? 185.0f : 85.0f;

    // Stock 6-strike fault debounce
    if (raw <= -100.0f || fabsf(raw - resetVal) < 0.05f || fabsf(raw - 127.0f) < 0.05f)
    {
      if (relaySensors.tempErrorCount < 250)
        relaySensors.tempErrorCount++;
      if (relaySensors.tempErrorCount >= 6)
        relaySensors.temp = 127.0f;
    }
    else
    {
      relaySensors.tempErrorCount = 0;
      relaySensors.temp = raw + cfg.tempCorrection1;
    }
  }
}

static void pollRelayPressure()
{
  unsigned long now = millis();
  if (lastPressurePollMs != 0 && (now - lastPressurePollMs) < 15000UL)
    return;
  lastPressurePollMs = now;

  if (cfg.pressureUnit != 1 && cfg.pressureUnit != 2)
  {
    relaySensors.bar = 0.0f;
    relaySensors.psi = 0.0f;
    relaySensors.pressureAlarm = false;
    return;
  }

  int raw = analogRead(PRESSURE_PIN);
  relaySensors.rawAdc = raw;
  relaySensors.voltage = (float)raw * 4.5f / 1024.0f;

  if (cfg.maxPressure <= cfg.minPressure || raw < cfg.minPressure)
  {
    relaySensors.bar = 0.0f;
    relaySensors.psi = 0.0f;
  }
  else
  {
    relaySensors.bar = ((float)(raw - cfg.minPressure) * cfg.maxPressureBar) /
                       (float)(cfg.maxPressure - cfg.minPressure);
    relaySensors.psi = relaySensors.bar * 14.503768f;
  }
}

static bool evaluateAlarms()
{
  bool anyAlarm = false;

  // Slot 1 & Slot 2 alarms
  for (int s = 1; s <= 2; s++)
  {
    slots[s].alarmTemp = false;
    slots[s].alarmSg = false;
    if (s == 2 && cfg.ispindelNum < 2 && !slots[2].active)
      continue;
    if (!slots[s].active)
      continue;

    if (slots[s].rawTemp != 0.0f)
    {
      if (slots[s].corrTemp < (float)cfg.alarmTempLow[s] ||
          slots[s].corrTemp > (float)cfg.alarmTempHigh[s])
      {
        slots[s].alarmTemp = true;
        anyAlarm = true;
      }
    }
    if (slots[s].rawSg > 0.5f && cfg.alarmSgLow[s] > 0.5f)
    {
      if (slots[s].corrSg < cfg.alarmSgLow[s])
      {
        slots[s].alarmSg = true;
        anyAlarm = true;
      }
    }
  }

  // Onboard DS18B20 relay thermometer alarms (alarm_3, alarm_4)
  relaySensors.tempAlarm = false;
  if (cfg.tempYesNo == 2 && fabsf(relaySensors.temp - 127.0f) > 0.1f)
  {
    float resetCheck = ((cfg.tempUnit == 2) ? 185.0f : 85.0f) + cfg.tempCorrection1;
    if (fabsf(relaySensors.temp - resetCheck) > 0.1f)
    {
      if (relaySensors.temp < (float)cfg.alarmRelayTempLow ||
          relaySensors.temp > (float)cfg.alarmRelayTempHigh)
      {
        relaySensors.tempAlarm = true;
        anyAlarm = true;
      }
    }
  }

  // Onboard pressure alarms (pressure_alarm_low, pressure_alarm_high)
  relaySensors.pressureAlarm = false;
  if (cfg.pressureUnit == 1 || cfg.pressureUnit == 2)
  {
    float pVal = (cfg.pressureUnit == 2) ? relaySensors.psi : relaySensors.bar;
    if ((cfg.pressureAlarmLow > 0.0f && pVal < cfg.pressureAlarmLow) ||
        (cfg.pressureAlarmHigh > 0.0f && pVal > cfg.pressureAlarmHigh))
    {
      relaySensors.pressureAlarm = true;
      anyAlarm = true;
    }
  }

  if (anyAlarm)
    beepAlarm();

  return anyAlarm;
}
