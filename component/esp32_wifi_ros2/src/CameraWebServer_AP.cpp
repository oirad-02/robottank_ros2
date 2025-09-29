// WARNING!!! Make sure that you have selected Board ---> ESP32 Dev Module
//            Partition Scheme ---> Huge APP (3MB No OTA/1MB SPIFFS)
//            PSRAM ---> enabled

// Select camera model
#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_AI_THINKER

#include "CameraWebServer_AP.h"
#include "camera_pins.h"
#include "esp_system.h"

void startCameraServer();

void CameraWebServer_STA::CameraWebServer_STA_Init(void)
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000; // 20000000
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Konfiguration basierend auf PSRAM-Verfügbarkeit
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // Kamera initialisieren
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  sensor_t *s = esp_camera_sensor_get();
  // Frame-Größe für bessere Performance reduzieren
  s->set_framesize(s, FRAMESIZE_SVGA); // 800x600 - mittlere Qualität
  
#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 0);
  s->set_hmirror(s, 1);
#endif
  s->set_vflip(s, 0);   // Bild-Orientierung (oben/unten)
  s->set_hmirror(s, 0); // Bild-Orientierung (links/rechts)

  Serial.println("\r\n");

  // WiFi im Station Mode starten
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // Verbindung zum bestehenden WLAN
  Serial.println(":----------------------------:");
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  Serial.println(":----------------------------:");
  
  WiFi.begin(ssid, password);
  
  // Warten auf Verbindung (max. 20 Sekunden)
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    
    // WiFi-Name für Factory-Test setzen
    wifi_name = WiFi.localIP().toString();
    
    startCameraServer();
    
    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");
  } else {
    Serial.println("");
    Serial.println("Failed to connect to WiFi!");
    Serial.println("Please check your credentials and try again.");
    
    // Fallback: Versuche es nochmal nach kurzer Pause
    delay(5000);
    reconnectWiFi();
  }
}

void CameraWebServer_STA::reconnectWiFi(void)
{
  // Verhindere zu häufige Reconnect-Versuche
  if (millis() - lastReconnectAttempt < reconnectInterval) {
    return;
  }
  
  lastReconnectAttempt = millis();
  
  Serial.println("Attempting to reconnect to WiFi...");
  
  // WiFi-Verbindung beenden und neu starten
  WiFi.disconnect();
  delay(1000);
  
  WiFi.begin(ssid, password);
  
  // Kurz warten (max. 10 Sekunden)
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("Reconnected to WiFi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // WiFi-Name aktualisieren
    wifi_name = WiFi.localIP().toString();
  } else {
    Serial.println("");
    Serial.println("Reconnection failed. Will try again later.");
  }
}