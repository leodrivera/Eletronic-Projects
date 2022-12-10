// ESP IR Transmitter with OLED display
// Written by Leonardo Rivera

// Libraries needed:
//  SH1106Wire.h: https://github.com/ThingPulse/esp8266-oled-ssd1306
//  PubSubClient.h: https://github.com/knolleary/pubsubclient
//  NTPClient.h: https://github.com/arduino-libraries/NTPClient
//  IRremoteESP8266 & IRsend.h: https://github.com/crankyoldgit/IRremoteESP8266

#include <Wire.h>
#include <TimeLib.h>
#include "SH1106Wire.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include "secrets.h"

//Prototypes
void setup_wifi();
void reconnectMQTT();
void ota_setup();
void mqtt_callback(char* topic, byte* payload, unsigned int length);

/************************* Ports to Connect ****************************
 D1 -> SCL
 D2 -> SDA
 D4 -> IR LED
***********************************************************************/

/*************************** IR Setup ************************************/

IRsend irsend(D4);  // Set the GPIO to be used to sending the message.

// Variables:
uint16_t NEC_Repeat[3] = {8996, 2210, 608}; // TODO: Ver para que serve
uint16_t AC_on[347] = {9090, 4504,  584, 538,  584, 536,  584, 536,  584, 1670,  586, 1670,  584, 538,  586, 538,  584, 538,  584, 538,  584, 536,  612, 530,  586, 536,  584, 536,  584, 536,  586, 536,  584, 538,  584, 538,  584, 538,  584, 538,  584, 536,  586, 536,  584, 538,  610, 528,  586, 538,  584, 538,  584, 536,  586, 536,  588, 536,  584, 536,  584, 538,  584, 536,  586, 536,  586, 538,  584, 1672,  608, 528,  586, 538,  584, 536,  586, 1668,  586, 538,  586, 536,  586, 536,  586, 538,  584, 538,  584, 538,  584, 536,  584, 538,  612, 528,  586, 536,  586, 538,  584, 1670,  584, 538,  584, 1672,  582, 1670,  584, 1672,  584, 538,  582, 538,  610, 13522,  9062, 4506,  584, 538,  610, 528,  586, 536,  584, 1670,  584, 1670,  586, 536,  586, 536,  584, 538,  584, 538,  584, 538,  584, 536,  586, 536,  584, 538,  610, 532,  584, 536,  584, 536,  586, 538,  584, 538,  584, 536,  586, 536,  584, 538,  584, 538,  586, 536,  584, 538,  584, 538,  610, 528,  586, 538,  584, 536,  584, 538,  584, 536,  586, 538,  584, 536,  586, 536,  586, 1670,  586, 536,  586, 536,  586, 534,  612, 1664,  584, 536,  584, 538,  586, 536,  586, 538,  584, 536,  586, 538,  584, 536,  586, 534,  586, 538,  584, 536,  584, 538,  610, 1662,  586, 538,  584, 1670,  584, 1670,  586, 1670,  586, 534,  586, 538,  584, 13506,  9088, 4504,  584, 540,  584, 536,  584, 538,  584, 1672,  584, 1670,  584, 536,  586, 536,  584, 538,  584, 538,  584, 538,  610, 530,  586, 536,  586, 536,  584, 538,  586, 536,  584, 538,  584, 536,  584, 538,  582, 538,  584, 538,  584, 536,  586, 538,  610, 532,  582, 536,  584, 538,  584, 538,  584, 536,  586, 536,  586, 536,  586, 538,  582, 538,  584, 536,  584, 538,  586, 1670,  610, 528,  586, 536,  584, 538,  584, 1670,  584, 538,  584, 538,  584, 536,  584, 538,  586, 538,  584, 536,  586, 538,  584, 536,  610, 530,  584, 536,  586, 536,  586, 1672,  584, 536,  586, 1670,  586, 1668,  586, 1670,  584, 536,  586, 538,  610};  // UNKNOWN 3DE70F42
uint16_t AC_21[347] = {9086, 4504,  586, 536,  586, 538,  584, 538,  584, 1670,  584, 1670,  584, 536,  586, 538,  584, 538,  584, 538,  586, 1668,  610, 1662,  586, 536,  584, 538,  584, 538,  584, 538,  584, 1672,  584, 538,  584, 538,  584, 538,  584, 538,  584, 538,  584, 536,  612, 526,  586, 536,  584, 538,  586, 534,  586, 538,  584, 536,  584, 536,  584, 538,  584, 536,  586, 536,  586, 538,  584, 1670,  610, 528,  586, 536,  584, 538,  582, 1670,  586, 538,  584, 540,  584, 536,  584, 536,  586, 536,  584, 538,  584, 536,  586, 536,  610, 530,  584, 536,  584, 538,  584, 536,  586, 538,  584, 536,  586, 538,  584, 536,  586, 1670,  584, 1670,  612, 13522,  9060, 4504,  584, 536,  612, 528,  586, 538,  586, 1672,  582, 1670,  584, 546,  578, 536,  586, 536,  586, 536,  584, 1670,  584, 1670,  584, 536,  586, 536,  610, 530,  584, 538,  584, 1672,  582, 538,  584, 536,  586, 536,  584, 538,  586, 536,  584, 536,  586, 534,  586, 536,  584, 536,  612, 528,  586, 538,  584, 538,  584, 536,  586, 534,  586, 536,  584, 536,  586, 540,  582, 1670,  586, 534,  588, 536,  586, 536,  612, 1662,  584, 538,  584, 536,  586, 536,  586, 536,  584, 536,  586, 536,  584, 538,  584, 538,  584, 538,  584, 538,  584, 538,  610, 530,  584, 538,  584, 538,  584, 536,  586, 536,  584, 1670,  584, 1670,  584, 13506,  9088, 4504,  584, 538,  584, 538,  582, 538,  584, 1672,  582, 1672,  584, 538,  584, 536,  586, 536,  586, 538,  584, 1672,  610, 1662,  584, 538,  584, 538,  584, 536,  584, 538,  584, 1672,  584, 538,  584, 538,  584, 538,  584, 536,  586, 538,  584, 536,  610, 530,  584, 538,  584, 538,  584, 536,  584, 538,  584, 536,  584, 538,  584, 538,  584, 538,  584, 538,  584, 536,  584, 1670,  610, 530,  584, 538,  584, 538,  584, 1670,  584, 536,  586, 536,  586, 536,  584, 538,  584, 536,  586, 536,  584, 536,  586, 538,  610, 530,  584, 536,  584, 538,  584, 536,  586, 536,  586, 538,  584, 538,  584, 538,  584, 1670,  584, 1672,  608};  // UNKNOWN D6631D07
uint16_t AC_22[347] = {9088, 4506,  582, 538,  584, 536,  586, 536,  586, 1670,  586, 1668,  586, 538,  584, 536,  584, 538,  584, 536,  586, 536,  610, 530,  584, 1670,  586, 538,  584, 538,  586, 536,  584, 1672,  584, 538,  584, 538,  584, 538,  586, 536,  586, 536,  584, 538,  610, 528,  584, 538,  586, 536,  586, 536,  584, 538,  584, 538,  584, 536,  586, 538,  584, 538,  584, 536,  584, 538,  584, 1672,  610, 528,  584, 538,  584, 538,  584, 1670,  586, 538,  584, 536,  584, 538,  586, 536,  586, 536,  584, 538,  584, 536,  586, 536,  612, 528,  586, 536,  584, 538,  584, 1670,  586, 536,  584, 538,  584, 536,  586, 538,  584, 1672,  584, 1670,  610, 13522,  9062, 4504,  584, 536,  612, 528,  586, 538,  584, 1670,  586, 1670,  584, 538,  584, 538,  586, 536,  584, 538,  584, 538,  584, 536,  586, 1670,  584, 538,  612, 530,  584, 536,  584, 1670,  584, 538,  584, 538,  584, 536,  586, 538,  584, 538,  584, 536,  586, 536,  586, 536,  586, 536,  610, 530,  584, 538,  584, 536,  584, 540,  582, 536,  586, 536,  584, 536,  586, 538,  584, 1670,  584, 538,  584, 538,  584, 534,  612, 1662,  586, 534,  586, 536,  586, 538,  584, 536,  586, 536,  584, 538,  584, 536,  586, 534,  588, 536,  584, 538,  584, 538,  610, 1662,  586, 536,  584, 538,  584, 536,  584, 536,  586, 1670,  584, 1670,  586, 13506,  9088, 4502,  586, 538,  584, 536,  586, 536,  584, 1670,  584, 1670,  586, 538,  584, 538,  584, 536,  586, 536,  584, 538,  610, 530,  584, 1670,  584, 538,  584, 536,  584, 538,  584, 1672,  584, 538,  584, 538,  584, 538,  584, 538,  584, 538,  584, 538,  612, 528,  584, 538,  582, 538,  584, 538,  584, 538,  582, 538,  584, 538,  584, 538,  582, 538,  586, 536,  586, 536,  586, 1670,  610, 530,  584, 536,  584, 538,  584, 1672,  584, 536,  584, 538,  584, 536,  586, 538,  584, 538,  584, 538,  586, 536,  584, 540,  608, 530,  584, 538,  584, 536,  586, 1670,  584, 538,  584, 538,  584, 536,  584, 538,  586, 1670,  584, 1668,  612};  // UNKNOWN 62ED5241

/*************************** OLED Setup ************************************/
SH1106Wire display(0x3c, SDA, SCL);

/*************************** NTP Setup ************************************/
// Set up the NTP UDP client
WiFiUDP ntpUDP;

// Define NTP properties
#define TZ   -3*60*60      // In seconds
#define NTP_INTERVAL 60*1000    // In miliseconds
#define NTP_ADDRESS  "pool.ntp.br"  // change this to whatever pool is closest (see ntp.org)

NTPClient timeClient(ntpUDP, NTP_ADDRESS, TZ, NTP_INTERVAL);

/************************* MQTT & Wi-Fi Setup *********************************/

//Variáveis e objetos globais
#define DEVICE_NAME "NodeMCU_IR"
WiFiClient espClient; // Cria o objeto espClient
PubSubClient mqtt_client(espClient); // Instancia o Cliente MQTT passando o objeto espClient

void setup() {
  Serial.begin(115200);
  delay(10);
  irsend.begin(); // Start the ir module
  display.init(); // Start the display
  setup_wifi(); // Start the Wi-Fi connection
  timeClient.begin();   // Start the NTP UDP client
  mqtt_client.setServer(MQTT_SERVER, MQTT_SERVERPORT);
  mqtt_client.setCallback(mqtt_callback);
  ota_setup();
}

void setup_wifi() {
  // Connect to Wi-Fi access point.
  delay(10);
  Serial.println();
  Serial.println("Connecting to:");
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Connecting to:");
  Serial.println(WLAN_SSID);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 22, WLAN_SSID);
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  unsigned long prev_time = millis();
  int counter = 1;
  String text;
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    text += ".";
    display.drawString(64, 45, text);
    if (millis() - prev_time > 5000) {
      Serial.println("Connection Failed! Rebooting...");
      ESP.restart();
    }
    counter++;
  }

  display.clear();
  Serial.println();
  Serial.println("Wi-Fi Connected");
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 0, "Wi-Fi Connected");
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  Serial.print("IP address: ");
  display.drawString(0, 25, "IP address:");
  Serial.println(WiFi.localIP());
  display.drawString(0, 37, WiFi.localIP().toString());
  display.display();
  delay(1000);
}

void wifi_disconnected(WiFiEvent_t event){
  Serial.println("Disconnected from Wi-Fi access point");
  Serial.println("Reconnecting...");
  WiFi.begin(WLAN_SSID, WLAN_PASS);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  char msg[60];
  sprintf(msg, "Received topic [%s] with payload [%s]", topic, payload);
  Serial.println(msg);
  if (String(topic) == "/feeds/ac-on"){
    Serial.println("ac-on");
  }
  if (String(topic) == "/feeds/ac-21"){
    Serial.println("ac-21");
  }
}

void reconnectMQTT() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(DEVICE_NAME, MQTT_USERNAME, MQTT_PASS)) {
      Serial.println("Connected");
      //char time_on[30];
      //sprintf(time_on, "%ld", millis()/1000);
      //mqtt_client.publish("/status/uptime", time_on);
      Serial.println("Sent uptime message");
      mqtt_client.subscribe("/feeds/ac_on");
      mqtt_client.subscribe("/feeds/ac_21");
      mqtt_client.subscribe("/feeds/ac_22");
      mqtt_client.subscribe("/feeds/tv_on");
      mqtt_client.subscribe("/feeds/sound_on");
      mqtt_client.subscribe("/feeds/sound_audio1");
      mqtt_client.subscribe("/feeds/sound_up");
      mqtt_client.subscribe("/feeds/sound_down");
      Serial.println("Feeds subscribed");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void ota_setup() {
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(DEVICE_NAME);

  // No authentication by default
  ArduinoOTA.setPassword(OTA_PASS);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

unsigned long prev_time = millis();

void loop() {
  ArduinoOTA.handle();
  WiFi.onEvent(wifi_disconnected, WIFI_EVENT_STAMODE_DISCONNECTED); 
  if (!mqtt_client.connected()) {
    reconnectMQTT();
  }
  mqtt_client.loop();
  if (millis() - prev_time > 5000) {
    prev_time = millis();
    char time_on[30];
    sprintf(time_on, "%ld", millis()/1000);
    Serial.println(time_on);
    mqtt_client.publish("/status/uptime", time_on);
    Serial.println("Sent uptime message");
  }
  display.clear(); // clear the display
  timeClient.update(); // update NTP time
  /*time_t t = timeClient.getEpochTime();
  t = t - 3*60*60;
  String clock_time = "";
  if (hour(t) < 10) // add a zero if hour is under 10
  clock_time += "0";
  clock_time += hour(t);
  clock_time += ":";
  if (minute(t) < 10) // add a zero if minute is under 10
  clock_time += "0";
  clock_time += minute(t);
  clock_time += ":";
  if (second(t) < 10) // add a zero if second is under 10
  clock_time += "0";  
  clock_time += second(t);*/
  //TODO: Adicionar dia da semana e data no formato dd/mm/yyy

  //Serial.println(timeClient.getFormattedTime()); // time in utc
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 22, timeClient.getFormattedTime());
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 54, String(millis()));
  display.display();
  delay(500);
}
