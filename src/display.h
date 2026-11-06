#pragma once

#include "state.h"
#include <SPI.h>
#include <TFT_eSPI.h>

#define GFXFF 1

static TFT_eSPI tft = TFT_eSPI(240, 240);

// RGB565 colour palette (per display_layout_spec.md)
#define COL_BG      0x0000 // Pure Black (#000000)
#define COL_SURFACE 0x18E4 // Dark Slate (#181C20)
#define COL_DIVIDER 0x31A7 // Muted Steel (#303438)
#define COL_SG      0xFFDF // Warm White (#F8FAF8)
#define COL_TEMP    0x3EDF // Ice Cyan (#38DBFF)
#define COL_NAME    0xB5D8 // Silver Grey (#B0B8C0)
#define COL_OK      0x270C // Emerald Green (#20E060)
#define COL_WARN    0xFDC0 // Amber Gold (#FFB800)
#define COL_ALERT   0xF986 // Coral Red (#FF3030)
#define COL_EMPTY   0x52CC // Dim Charcoal (#505860)

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

static void formatRelaySummary(char *buf, size_t len)
{
  buf[0] = '\0';
  bool hasT = (cfg.tempYesNo == 2);
  bool hasP = (cfg.pressureUnit == 1 || cfg.pressureUnit == 2);
  if (!hasT && !hasP)
    return;

  char tPart[14] = "";
  char pPart[14] = "";
  if (hasT)
  {
    if (fabsf(relaySensors.temp - 127.0f) < 0.1f)
      snprintf(tPart, sizeof(tPart), "T:ERR");
    else
      snprintf(tPart, sizeof(tPart), "%.1f%c", relaySensors.temp,
               (cfg.tempUnit == 2) ? 'F' : 'C');
  }
  if (hasP)
  {
    if (cfg.pressureUnit == 2)
      snprintf(pPart, sizeof(pPart), "%.1fpsi", relaySensors.psi);
    else
      snprintf(pPart, sizeof(pPart), "%.2fbar", relaySensors.bar);
  }
  if (hasT && hasP)
    snprintf(buf, len, "%.13s  %.13s", tPart, pPart);
  else if (hasT)
    snprintf(buf, len, "AMB %s", tPart);
  else
    snprintf(buf, len, "PRS %s", pPart);
}

static void drawSlotHemisphere(bool isTop, int dataSlot)
{
  uint32_t arcStart = isTop ? 100 : 280;
  uint32_t arcEnd = isTop ? 260 : 80;

  int yName = isTop ? 35 : 204;
  int ySG = isTop ? 78 : 156;
  int yTemp = isTop ? 68 : 146;
  int yBatt = isTop ? 93 : 171;
  int xRight = 190;

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

  if (dataSlot < 0 || !slots[dataSlot].active)
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

    // If bottom slot is idle and onboard relay sensors are enabled, show them here!
    char relayBuf[36];
    formatRelaySummary(relayBuf, sizeof(relayBuf));
    if (!isTop && relayBuf[0] != '\0')
    {
      uint16_t rCol = (relaySensors.tempAlarm || relaySensors.pressureAlarm) ? COL_ALERT : COL_TEMP;
      tft.setFreeFont(&FreeSansBold9pt7b);
      tft.setTextColor(rCol);
      tft.setTextDatum(MC_DATUM);
      tft.drawString(relayBuf, 120, ySG - 6, GFXFF);
      tft.setTextColor(COL_NAME);
      tft.drawString("RELAY SENSORS", 120, yBatt + 4, 2);
      tft.drawSmoothArc(120, 120, 113, 109, arcStart, arcEnd, rCol, COL_BG, true);
      return;
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

  SlotState &st = slots[dataSlot];
  int cfgIdx = (dataSlot == 2) ? 2 : 1;
  float sg = st.corrSg;
  float temp = st.corrTemp;
  float batt = st.batt;
  uint32_t ageSec = slotAgeSeconds(dataSlot);
  uint32_t intervalSec = st.interval > 0 ? st.interval : 900U;

  uint16_t battCol = (batt >= 3.70f) ? COL_OK : (batt >= 3.45f ? COL_WARN : COL_ALERT);
  bool timeout = (ageSec > intervalSec * 4U + 120U);
  bool stale = (ageSec > intervalSec * 2U + 60U);
  bool hasAlarm = st.alarmSg || st.alarmTemp;
  uint16_t arcCol = (hasAlarm || timeout || batt < 3.45f)
                        ? COL_ALERT
                        : ((stale || batt < 3.70f) ? COL_WARN : COL_OK);

  // 1. Name + Age + Phase Badge (inside R=106)
  char ageBuf[16];
  formatAge(ageSec, ageBuf, sizeof(ageBuf));
  char hdr[28];
  snprintf(hdr, sizeof(hdr), "%.8s  %s", st.name.c_str(), ageBuf);
  tft.fillSmoothRoundRect(62, yName - 10, 116, 20, 8, COL_SURFACE, COL_BG);
  tft.setTextColor(hasAlarm ? COL_ALERT : (timeout ? COL_ALERT : (stale ? COL_WARN : COL_NAME)));
  tft.setTextDatum(MC_DATUM);
  tft.drawString(hdr, 120, yName, 2);

  // 2. Hero Specific Gravity or Plato (FreeSansBold18pt7b = 24 px cap height)
  char sgBuf[12];
  if (cfg.unit[cfgIdx] == 2)
    snprintf(sgBuf, sizeof(sgBuf), "%.1fP", sgToPlato(sg));
  else
    snprintf(sgBuf, sizeof(sgBuf), "%.3f", ensureSg(sg));

  tft.setFreeFont(&FreeSansBold18pt7b);
  tft.setTextColor(st.alarmSg ? COL_ALERT : COL_SG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(sgBuf, 34, ySG, GFXFF);

  // 3. Temperature (FreeSansBold9pt7b = 12 px cap height, Ice Cyan or Alert Red)
  uint16_t tCol = st.alarmTemp ? COL_ALERT : COL_TEMP;
  char unitStr[2] = {st.tempUnit, 0};
  char numBuf[10];
  snprintf(numBuf, sizeof(numBuf), "%.1f", temp);
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(tCol);
  tft.setTextDatum(MR_DATUM);
  int uW = tft.drawString(unitStr, xRight, yTemp, GFXFF);
  int degX = xRight - uW - 4;
  tft.drawSmoothCircle(degX, yTemp - 3, 2, tCol, COL_BG);
  tft.drawString(numBuf, degX - 4, yTemp, GFXFF);

  // 4. Battery Voltage (FreeSansBold9pt7b = 12 px cap height, colour-coded)
  char batBuf[12];
  snprintf(batBuf, sizeof(batBuf), "%.2fV", batt);
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(battCol);
  tft.setTextDatum(MR_DATUM);
  tft.drawString(batBuf, xRight, yBatt, GFXFF);

  // 5. Perimeter Status Arc
  tft.drawSmoothArc(120, 120, 113, 109, arcStart, arcEnd, arcCol, COL_BG, true);
}

static void drawDisplay(bool fullClear = false)
{
  if (fullClear)
    tft.fillScreen(COL_BG);

  int topSlot = slots[1].active ? 1 : (slots[0].active ? 0 : -1);
  int botSlot = slots[2].active ? 2 : (slots[1].active && slots[0].active ? 0 : -1);

  drawSlotHemisphere(true, topSlot);
  drawSlotHemisphere(false, botSlot);

  // Center horizontal divider line + WiFi heartbeat dot (or relay sensor pill if both slots active)
  tft.drawFastHLine(18, 120, 204, COL_DIVIDER);
  tft.drawFastHLine(18, 121, 204, COL_DIVIDER);

  char relayBuf[36];
  formatRelaySummary(relayBuf, sizeof(relayBuf));
  if (botSlot >= 0 && relayBuf[0] != '\0')
  {
    uint16_t rCol = (relaySensors.tempAlarm || relaySensors.pressureAlarm) ? COL_ALERT : COL_TEMP;
    tft.fillSmoothRoundRect(68, 112, 104, 16, 6, COL_SURFACE, COL_BG);
    tft.setTextColor(rCol);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(relayBuf, 120, 120, 1);
  }
  else
  {
    uint16_t wifiCol = (WiFi.status() == WL_CONNECTED) ? COL_OK : COL_ALERT;
    drawHeartbeatDot(wifiCol);
  }
}
