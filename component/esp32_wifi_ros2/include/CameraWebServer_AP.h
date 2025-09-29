/*
 * @Descripttion: ESP32 Camera Web Server - Station Mode Header
 * @version: Modified for existing WiFi connection
 * @Author: Modified from Elegoo original
 * @Date: 2025-07-29
 */

#ifndef _CameraWebServer_STA_H
#define _CameraWebServer_STA_H
#include "esp_camera.h"
#include <WiFi.h>

#include "env.hpp"

class CameraWebServer_STA
{
public:
  void CameraWebServer_STA_Init(void);
  void reconnectWiFi(void);
  String wifi_name;

private:
  // WLAN-Zugangsdaten - HIER DEINE DATEN EINTRAGEN!
  const char *ssid = WIFI_SSID;        // Ersetze mit deinem WLAN-Namen
  const char *password = WIFI_PASS; // Ersetze mit deinem WLAN-Passwort
  
  unsigned long lastReconnectAttempt = 0;
  const unsigned long reconnectInterval = 10000; // 10 Sekunden zwischen Reconnect-Versuchen
};

#endif