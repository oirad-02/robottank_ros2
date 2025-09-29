/*
 * @Descripttion: ESP32 Camera Server - Station Mode (STA)
 * @version: Modified for existing WiFi connection
 * @Author: Modified from Elegoo original
 * @Date: 2025-07-29
 */

#include "CameraWebServer_AP.h"
#include <WiFi.h>
#include "esp_camera.h"

WiFiServer server(100);

#define RXD2 33
#define TXD2 4
CameraWebServer_STA CameraWebServerSTA;

bool WA_en = false;

void SocketServer_Test(void)
{
  static bool ED_client = true;
  WiFiClient client = server.available(); //尝试建立客户对象
  if (client)                             //如果当前客户可用
  {
    WA_en = true;
    ED_client = true;
    Serial.println("[Client connected]");
    String readBuff;
    String sendBuff;
    uint8_t Heartbeat_count = 0;
    bool Heartbeat_status = false;
    bool data_begin = true;
    while (client.connected()) //如果客户端处于连接状态
    {
      if (client.available()) //如果有可读数据
      {
        char c = client.read();             //读取一个字节
        Serial.print(c);                    //从串口打印
        if (true == data_begin && c == '{') //接收到开始字符
        {
          data_begin = false;
        }
        if (false == data_begin && c != ' ') //去掉空格
        {
          readBuff += c;
        }
        if (false == data_begin && c == '}') //接收到结束字符
        {
          data_begin = true;
          if (true == readBuff.equals("{Heartbeat}"))
          {
            Heartbeat_status = true;
          }
          else
          {
            Serial2.print(readBuff);
          }
          readBuff = "";
        }
      }
      if (Serial2.available())
      {
        char c = Serial2.read();
        sendBuff += c;
        if (c == '}') //接收到结束字符
        {
          client.print(sendBuff);
          Serial.print(sendBuff); //从串口打印
          sendBuff = "";
        }
      }

      static unsigned long Heartbeat_time = 0;
      if (millis() - Heartbeat_time > 1000) //心跳频率
      {
        client.print("{Heartbeat}");
        if (true == Heartbeat_status)
        {
          Heartbeat_status = false;
          Heartbeat_count = 0;
        }
        else if (false == Heartbeat_status)
        {
          Heartbeat_count += 1;
        }
        if (Heartbeat_count > 3)
        {
          Heartbeat_count = 0;
          Heartbeat_status = false;
          break;
        }
        Heartbeat_time = millis();
      }
      
      static unsigned long Test_time = 0;
      if (millis() - Test_time > 1000) //定时检测WiFi连接状态
      {
        Test_time = millis();
        if (WiFi.status() != WL_CONNECTED) //如果WiFi断开连接，发送停止命令
        {
          Serial2.print("{\"N\":100}");
          break;
        }
      }
    }
    Serial2.print("{\"N\":100}");
    client.stop(); //结束当前连接:
    Serial.println("[Client disconnected]");
  }
  else
  {
    if (ED_client == true)
    {
      ED_client = false;
      Serial2.print("{\"N\":100}");
    }
  }
}

/*作用于测试架*/
void FactoryTest(void)
{
  static String readBuff;
  String sendBuff;
  if (Serial2.available())
  {
    char c = Serial2.read();
    readBuff += c;
    if (c == '}') //接收到结束字符
    {
      if (true == readBuff.equals("{BT_detection}"))
      {
        Serial2.print("{BT_OK}");
        Serial.println("Factory...");
      }
      else if (true == readBuff.equals("{WA_detection}"))
      {
        Serial2.print("{");
        Serial2.print(CameraWebServerSTA.wifi_name);
        Serial2.print("}");
        Serial.println("Factory...");
      }
      readBuff = "";
    }
  }
  {
    if (WiFi.status() == WL_CONNECTED) //WiFi verbunden
    {
      if (true == WA_en)
      {
        digitalWrite(13, LOW);
        Serial2.print("{WA_OK}");
        WA_en = false;
      }
    }
    else
    {
      //获取时间戳 timestamp
      static unsigned long Test_time;
      static bool en = true;
      if (millis() - Test_time > 100)
      {
        if (false == WA_en)
        {
          Serial2.print("{WA_NO}");
          WA_en = true;
        }
        if (en == true)
        {
          en = false;
          digitalWrite(13, HIGH);
        }
        else
        {
          en = true;
          digitalWrite(13, LOW);
        }
        Test_time = millis();
      }
    }
  }
}

void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Initialisiere Camera Web Server im Station Mode
  CameraWebServerSTA.CameraWebServer_STA_Init();
  server.begin();
  delay(100);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.println("Elegoo-2020 Station Mode...");
  Serial2.print("{Factory}");
}

void loop()
{
  // Überprüfe WiFi-Verbindung und versuche Reconnect bei Bedarf
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Attempting to reconnect...");
    CameraWebServerSTA.reconnectWiFi();
  }
  
  SocketServer_Test();
  FactoryTest();
}