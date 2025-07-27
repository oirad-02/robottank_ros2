#include <WiFi.h>

#include "env.hpp"
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;  

const char* server_ip = "192.168.10.1";

const uint16_t server_port = 4242;

WiFiClient client;


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("Connecting to SSID: %s\n", ssid);

  WiFi.begin(ssid, password);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    Serial.print("Connecting to server: ");
    Serial.println(server_ip);

    if (!client.connect(server_ip, server_port)) {
      Serial.println("Connection to server failed.");
    } else {
      Serial.println("Connected to server.");
    }

  } else {
    Serial.println("\nFailed to connect.");
    Serial.printf("WiFi status: %d\n", WiFi.status());
  }
}

void loop() {
  if (client.connected()) {
  while (client.available()) {
    String msg = client.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(msg);

    int vals[3];
    if (sscanf(msg.c_str(), "[%d,%d,%d]", &vals[0], &vals[1], &vals[2]) == 3) {
      Serial.printf("Parsed: %d, %d, %d\n", vals[0], vals[1], vals[2]);

      // einfache Antwort senden
      int response = vals[0] + vals[1] + vals[2];
      client.println(String(response));
      Serial.printf("Sent: %d\n", response);
    }
   }
  } else {
    Serial.println("Lost connection to server. Reconnecting...");
    delay(2000);
    client.connect(server_ip, server_port);
  }

  delay(500); // etwas Luft lassen

}
