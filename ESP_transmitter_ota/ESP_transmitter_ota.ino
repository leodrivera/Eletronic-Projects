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
#include "fonts.h"
#include "secrets.h"

//Prototypes
void setup_wifi(void);
void mqtt_reconnect(void);
void check_connections(void);
void ota_setup(void);
void display_time(void);
void mqtt_callback(char* topic, byte* payload, unsigned int length);

/************************* Ports to Connect ****************************
 D1 -> SCL
 D2 -> SDA
 D5 -> IR LED
***********************************************************************/

unsigned long prev_time;
char msg[50];

/*************************** IR Setup ************************************/

IRsend irsend(D5);  // Set the GPIO to be used to sending the message.

// Set IR RAW codes
uint16_t NEC_Repeat[3] = {8996, 2210, 608}; // // 0xFFFFFFFFFFFFFFFF NEC code end of message
uint16_t AC_off[347] = {9090, 4504,  584, 538,  584, 536,  584, 536,  584, 1670,  586, 1670,  584, 538,  586, 538,  584, 538,  584, 538,  584, 536,  612, 530,  586, 536,  584, 536,  584, 536,  586, 536,  584, 538,  584, 538,  584, 538,  584, 538,  584, 536,  586, 536,  584, 538,  610, 528,  586, 538,  584, 538,  584, 536,  586, 536,  588, 536,  584, 536,  584, 538,  584, 536,  586, 536,  586, 538,  584, 1672,  608, 528,  586, 538,  584, 536,  586, 1668,  586, 538,  586, 536,  586, 536,  586, 538,  584, 538,  584, 538,  584, 536,  584, 538,  612, 528,  586, 536,  586, 538,  584, 1670,  584, 538,  584, 1672,  582, 1670,  584, 1672,  584, 538,  582, 538,  610, 13522,  9062, 4506,  584, 538,  610, 528,  586, 536,  584, 1670,  584, 1670,  586, 536,  586, 536,  584, 538,  584, 538,  584, 538,  584, 536,  586, 536,  584, 538,  610, 532,  584, 536,  584, 536,  586, 538,  584, 538,  584, 536,  586, 536,  584, 538,  584, 538,  586, 536,  584, 538,  584, 538,  610, 528,  586, 538,  584, 536,  584, 538,  584, 536,  586, 538,  584, 536,  586, 536,  586, 1670,  586, 536,  586, 536,  586, 534,  612, 1664,  584, 536,  584, 538,  586, 536,  586, 538,  584, 536,  586, 538,  584, 536,  586, 534,  586, 538,  584, 536,  584, 538,  610, 1662,  586, 538,  584, 1670,  584, 1670,  586, 1670,  586, 534,  586, 538,  584, 13506,  9088, 4504,  584, 540,  584, 536,  584, 538,  584, 1672,  584, 1670,  584, 536,  586, 536,  584, 538,  584, 538,  584, 538,  610, 530,  586, 536,  586, 536,  584, 538,  586, 536,  584, 538,  584, 536,  584, 538,  582, 538,  584, 538,  584, 536,  586, 538,  610, 532,  582, 536,  584, 538,  584, 538,  584, 536,  586, 536,  586, 536,  586, 538,  582, 538,  584, 536,  584, 538,  586, 1670,  610, 528,  586, 536,  584, 538,  584, 1670,  584, 538,  584, 538,  584, 536,  584, 538,  586, 538,  584, 536,  586, 538,  584, 536,  610, 530,  584, 536,  586, 536,  586, 1672,  584, 536,  586, 1670,  586, 1668,  586, 1670,  584, 536,  586, 538,  610};  // UNKNOWN 3DE70F42
uint16_t AC_18[347] = {9018, 4578,  500, 596,  522, 622,  474, 622,  500, 1776,  496, 1756,  498, 622,  498, 622,  498, 622,  498, 622,  498, 622,  524, 616,  496, 624,  472, 650,  470, 650,  470, 650,  470, 1782,  492, 628,  470, 648,  520, 602,  468, 652,  518, 602,  518, 602,  544, 594,  518, 602,  468, 652,  520, 600,  520, 600,  520, 600,  520, 602,  518, 600,  520, 600,  520, 602,  518, 602,  518, 1730,  548, 590,  520, 602,  468, 652,  520, 1732,  520, 600,  518, 600,  520, 600,  520, 600,  520, 600,  468, 652,  520, 602,  518, 600,  546, 592,  520, 598,  520, 602,  468, 1782,  522, 600,  470, 1782,  468, 1784,  468, 1782,  470, 652,  468, 1782,  496, 13620,  8938, 4610,  468, 652,  496, 642,  470, 650,  468, 1782,  470, 1782,  470, 652,  470, 626,  494, 650,  470, 650,  470, 652,  468, 650,  470, 626,  494, 652,  496, 616,  494, 628,  492, 1782,  470, 626,  494, 626,  494, 650,  520, 574,  546, 574,  544, 600,  522, 598,  522, 574,  546, 572,  576, 566,  546, 572,  548, 574,  546, 574,  546, 574,  548, 574,  496, 624,  546, 572,  496, 1756,  496, 622,  496, 624,  496, 624,  522, 1746,  496, 624,  496, 624,  496, 624,  496, 622,  524, 596,  524, 596,  498, 624,  498, 622,  524, 596,  524, 596,  524, 596,  550, 1720,  524, 594,  526, 1726,  526, 1726,  526, 1728,  526, 594,  526, 1726,  526, 13546,  8994, 4580,  524, 596,  500, 620,  500, 618,  500, 1752,  500, 1752,  502, 618,  500, 620,  500, 618,  502, 618,  502, 618,  528, 610,  502, 620,  500, 620,  502, 618,  502, 618,  502, 1750,  502, 618,  502, 620,  500, 620,  502, 620,  502, 618,  500, 618,  528, 610,  502, 618,  502, 618,  500, 620,  502, 618,  502, 618,  500, 620,  500, 620,  500, 620,  500, 620,  524, 594,  502, 1752,  550, 588,  526, 594,  526, 594,  526, 1724,  526, 594,  502, 620,  526, 594,  526, 594,  528, 592,  526, 594,  528, 592,  528, 592,  554, 584,  526, 596,  528, 592,  528, 1702,  550, 594,  526, 1726,  526, 1726,  526, 1726,  528, 570,  550, 1724,  552};  // UNKNOWN EF07B041
uint16_t AC_19[347] = {9026, 4578,  502, 616,  504, 594,  526, 566,  556, 1748,  504, 1724,  528, 618,  502, 592,  528, 564,  556, 616,  504, 1722,  556, 582,  530, 592,  528, 592,  528, 566,  554, 616,  504, 1726,  526, 592,  528, 592,  528, 592,  528, 592,  528, 594,  526, 592,  556, 582,  528, 618,  504, 590,  530, 590,  530, 618,  504, 592,  528, 592,  528, 592,  528, 592,  530, 590,  530, 590,  528, 1724,  556, 608,  506, 614,  504, 592,  530, 1694,  556, 590,  530, 590,  532, 590,  528, 616,  504, 590,  530, 594,  528, 590,  530, 592,  554, 582,  530, 590,  530, 592,  528, 592,  528, 1722,  530, 1722,  530, 1724,  528, 1724,  528, 590,  530, 1722,  558, 13558,  8998, 4550,  528, 590,  556, 582,  530, 592,  528, 1720,  530, 1722,  530, 592,  528, 592,  528, 590,  530, 590,  530, 1722,  528, 618,  504, 616,  504, 590,  558, 582,  530, 594,  526, 1724,  528, 594,  526, 594,  526, 594,  528, 590,  528, 594,  528, 592,  528, 594,  526, 590,  530, 590,  556, 584,  528, 592,  528, 592,  528, 592,  528, 592,  528, 594,  526, 592,  528, 594,  526, 1724,  528, 594,  528, 592,  528, 592,  556, 1714,  528, 592,  528, 592,  528, 592,  528, 594,  528, 592,  528, 592,  528, 592,  528, 592,  528, 592,  528, 594,  526, 594,  552, 584,  528, 1726,  528, 1726,  526, 1724,  528, 1724,  526, 592,  528, 1724,  528, 13544,  9022, 4552,  528, 592,  526, 594,  528, 592,  528, 1722,  530, 1722,  528, 592,  530, 592,  526, 592,  528, 592,  528, 1722,  558, 582,  530, 592,  528, 592,  528, 592,  528, 592,  528, 1724,  528, 590,  530, 592,  528, 590,  530, 590,  530, 590,  530, 590,  556, 582,  530, 590,  528, 592,  528, 592,  530, 592,  528, 590,  530, 590,  530, 592,  528, 590,  530, 590,  530, 592,  528, 1724,  554, 584,  528, 592,  528, 592,  528, 1724,  528, 590,  528, 592,  528, 594,  526, 592,  528, 592,  528, 592,  530, 592,  528, 592,  554, 584,  528, 592,  528, 592,  530, 592,  526, 1724,  528, 1724,  528, 1724,  528, 1724,  528, 594,  526, 1726,  554};  // UNKNOWN CA97C3A1
uint16_t AC_20[347] = {9000, 4574,  530, 588,  504, 642,  504, 592,  530, 1722,  504, 1774,  478, 616,  528, 592,  528, 564,  532, 590,  554, 590,  532, 1742,  500, 616,  504, 616,  530, 592,  504, 616,  528, 1722,  504, 618,  526, 592,  530, 592,  528, 594,  528, 592,  530, 590,  556, 584,  528, 590,  530, 592,  528, 592,  504, 618,  526, 592,  526, 596,  502, 616,  528, 594,  528, 592,  528, 592,  504, 1752,  554, 580,  504, 618,  526, 594,  502, 1748,  528, 592,  530, 562,  556, 592,  528, 592,  528, 590,  528, 592,  528, 618,  504, 592,  554, 610,  502, 592,  528, 592,  530, 1748,  502, 1722,  530, 1722,  504, 1748,  530, 1722,  528, 592,  530, 1724,  530, 13584,  8998, 4552,  528, 590,  558, 580,  530, 590,  530, 1722,  530, 1726,  526, 590,  530, 590,  528, 590,  530, 592,  530, 590,  530, 1722,  528, 590,  530, 590,  556, 584,  526, 592,  528, 1724,  528, 590,  530, 590,  528, 592,  530, 590,  530, 590,  530, 590,  530, 590,  532, 590,  530, 588,  556, 582,  528, 592,  530, 592,  528, 592,  528, 592,  554, 564,  530, 590,  530, 590,  530, 1722,  528, 590,  530, 592,  528, 590,  556, 1714,  530, 588,  532, 588,  530, 590,  530, 590,  530, 592,  528, 590,  530, 590,  530, 590,  530, 592,  528, 590,  530, 590,  558, 1712,  530, 1720,  530, 1722,  528, 1724,  530, 1722,  530, 590,  530, 1722,  530, 13542,  9024, 4550,  530, 590,  530, 590,  530, 592,  528, 1722,  530, 1722,  530, 590,  530, 590,  528, 590,  530, 590,  530, 590,  556, 1712,  530, 590,  530, 592,  528, 590,  530, 590,  530, 1722,  528, 590,  530, 592,  528, 590,  530, 592,  528, 592,  530, 590,  556, 582,  528, 592,  530, 590,  530, 590,  530, 590,  530, 590,  528, 592,  530, 590,  530, 592,  528, 592,  528, 590,  530, 1724,  554, 584,  528, 590,  530, 590,  530, 1722,  528, 592,  528, 590,  530, 590,  528, 592,  528, 592,  528, 592,  528, 592,  528, 592,  554, 584,  528, 592,  528, 592,  528, 1724,  528, 1724,  528, 1724,  528, 1724,  528, 1724,  528, 592,  528, 1724,  556};  // UNKNOWN C1DF06BD
uint16_t AC_21[347] = {9086, 4504,  586, 536,  586, 538,  584, 538,  584, 1670,  584, 1670,  584, 536,  586, 538,  584, 538,  584, 538,  586, 1668,  610, 1662,  586, 536,  584, 538,  584, 538,  584, 538,  584, 1672,  584, 538,  584, 538,  584, 538,  584, 538,  584, 538,  584, 536,  612, 526,  586, 536,  584, 538,  586, 534,  586, 538,  584, 536,  584, 536,  584, 538,  584, 536,  586, 536,  586, 538,  584, 1670,  610, 528,  586, 536,  584, 538,  582, 1670,  586, 538,  584, 540,  584, 536,  584, 536,  586, 536,  584, 538,  584, 536,  586, 536,  610, 530,  584, 536,  584, 538,  584, 536,  586, 538,  584, 536,  586, 538,  584, 536,  586, 1670,  584, 1670,  612, 13522,  9060, 4504,  584, 536,  612, 528,  586, 538,  586, 1672,  582, 1670,  584, 546,  578, 536,  586, 536,  586, 536,  584, 1670,  584, 1670,  584, 536,  586, 536,  610, 530,  584, 538,  584, 1672,  582, 538,  584, 536,  586, 536,  584, 538,  586, 536,  584, 536,  586, 534,  586, 536,  584, 536,  612, 528,  586, 538,  584, 538,  584, 536,  586, 534,  586, 536,  584, 536,  586, 540,  582, 1670,  586, 534,  588, 536,  586, 536,  612, 1662,  584, 538,  584, 536,  586, 536,  586, 536,  584, 536,  586, 536,  584, 538,  584, 538,  584, 538,  584, 538,  584, 538,  610, 530,  584, 538,  584, 538,  584, 536,  586, 536,  584, 1670,  584, 1670,  584, 13506,  9088, 4504,  584, 538,  584, 538,  582, 538,  584, 1672,  582, 1672,  584, 538,  584, 536,  586, 536,  586, 538,  584, 1672,  610, 1662,  584, 538,  584, 538,  584, 536,  584, 538,  584, 1672,  584, 538,  584, 538,  584, 538,  584, 536,  586, 538,  584, 536,  610, 530,  584, 538,  584, 538,  584, 536,  584, 538,  584, 536,  584, 538,  584, 538,  584, 538,  584, 538,  584, 536,  584, 1670,  610, 530,  584, 538,  584, 538,  584, 1670,  584, 536,  586, 536,  586, 536,  584, 538,  584, 536,  586, 536,  584, 536,  586, 538,  610, 530,  584, 536,  584, 538,  584, 536,  586, 536,  586, 538,  584, 538,  584, 538,  584, 1670,  584, 1672,  608};  // UNKNOWN D6631D07
uint16_t AC_22[347] = {9088, 4506,  582, 538,  584, 536,  586, 536,  586, 1670,  586, 1668,  586, 538,  584, 536,  584, 538,  584, 536,  586, 536,  610, 530,  584, 1670,  586, 538,  584, 538,  586, 536,  584, 1672,  584, 538,  584, 538,  584, 538,  586, 536,  586, 536,  584, 538,  610, 528,  584, 538,  586, 536,  586, 536,  584, 538,  584, 538,  584, 536,  586, 538,  584, 538,  584, 536,  584, 538,  584, 1672,  610, 528,  584, 538,  584, 538,  584, 1670,  586, 538,  584, 536,  584, 538,  586, 536,  586, 536,  584, 538,  584, 536,  586, 536,  612, 528,  586, 536,  584, 538,  584, 1670,  586, 536,  584, 538,  584, 536,  586, 538,  584, 1672,  584, 1670,  610, 13522,  9062, 4504,  584, 536,  612, 528,  586, 538,  584, 1670,  586, 1670,  584, 538,  584, 538,  586, 536,  584, 538,  584, 538,  584, 536,  586, 1670,  584, 538,  612, 530,  584, 536,  584, 1670,  584, 538,  584, 538,  584, 536,  586, 538,  584, 538,  584, 536,  586, 536,  586, 536,  586, 536,  610, 530,  584, 538,  584, 536,  584, 540,  582, 536,  586, 536,  584, 536,  586, 538,  584, 1670,  584, 538,  584, 538,  584, 534,  612, 1662,  586, 534,  586, 536,  586, 538,  584, 536,  586, 536,  584, 538,  584, 536,  586, 534,  588, 536,  584, 538,  584, 538,  610, 1662,  586, 536,  584, 538,  584, 536,  584, 536,  586, 1670,  584, 1670,  586, 13506,  9088, 4502,  586, 538,  584, 536,  586, 536,  584, 1670,  584, 1670,  586, 538,  584, 538,  584, 536,  586, 536,  584, 538,  610, 530,  584, 1670,  584, 538,  584, 536,  584, 538,  584, 1672,  584, 538,  584, 538,  584, 538,  584, 538,  584, 538,  584, 538,  612, 528,  584, 538,  582, 538,  584, 538,  584, 538,  582, 538,  584, 538,  584, 538,  582, 538,  586, 536,  586, 536,  586, 1670,  610, 530,  584, 536,  584, 538,  584, 1672,  584, 536,  584, 538,  584, 536,  586, 538,  584, 538,  584, 538,  586, 536,  584, 540,  608, 530,  584, 538,  584, 536,  586, 1670,  584, 538,  584, 538,  584, 536,  584, 538,  586, 1670,  584, 1668,  612};  // UNKNOWN 62ED5241
uint16_t AC_23[347] = {9026, 4550,  528, 590,  530, 594,  528, 592,  528, 1724,  526, 1750,  502, 592,  528, 592,  528, 590,  530, 592,  528, 1726,  554, 584,  504, 1748,  528, 594,  502, 616,  528, 592,  528, 1696,  556, 594,  526, 592,  528, 592,  528, 594,  526, 592,  528, 592,  554, 586,  526, 592,  528, 594,  528, 592,  528, 618,  500, 594,  526, 594,  526, 592,  528, 594,  526, 592,  530, 592,  528, 1724,  530, 606,  530, 592,  526, 592,  528, 1726,  526, 596,  526, 592,  528, 592,  528, 594,  526, 592,  528, 592,  504, 618,  552, 568,  552, 584,  528, 594,  526, 590,  504, 618,  528, 1724,  528, 592,  530, 590,  528, 592,  530, 1748,  502, 1726,  528, 13584,  8996, 4550,  530, 590,  556, 582,  528, 592,  528, 1726,  528, 1722,  528, 592,  528, 592,  526, 592,  528, 594,  528, 1724,  528, 592,  530, 1722,  528, 592,  554, 584,  528, 592,  528, 1722,  530, 592,  530, 590,  530, 592,  530, 590,  530, 590,  530, 590,  530, 592,  528, 590,  530, 592,  554, 582,  528, 592,  528, 592,  528, 592,  530, 592,  528, 592,  528, 592,  528, 592,  528, 1724,  528, 590,  530, 592,  528, 592,  556, 1714,  528, 594,  528, 592,  528, 592,  528, 592,  528, 592,  528, 592,  528, 594,  526, 592,  530, 590,  530, 592,  528, 592,  556, 584,  526, 1724,  556, 566,  528, 592,  528, 592,  528, 1724,  530, 1724,  528, 13544,  9022, 4552,  528, 594,  526, 592,  530, 592,  528, 1722,  502, 1750,  528, 592,  528, 592,  528, 592,  530, 590,  528, 1724,  554, 584,  528, 1724,  528, 594,  528, 590,  528, 592,  528, 1724,  526, 592,  530, 594,  526, 592,  528, 592,  528, 594,  528, 592,  554, 584,  528, 592,  528, 592,  504, 618,  528, 592,  526, 594,  528, 594,  526, 594,  526, 592,  528, 592,  528, 592,  528, 1726,  552, 584,  528, 592,  526, 594,  528, 1724,  526, 594,  524, 594,  528, 592,  528, 592,  528, 592,  528, 594,  526, 594,  526, 594,  554, 584,  526, 594,  528, 592,  526, 594,  528, 1724,  526, 594,  528, 594,  526, 592,  528, 1724,  528, 1724,  528};  // UNKNOWN CE152637
uint16_t AC_24[347] = {9026, 4500,  554, 588,  584, 554,  556, 564,  556, 1694,  556, 1696,  530, 590,  556, 564,  556, 562,  608, 514,  558, 562,  532, 1720,  532, 1724,  554, 564,  556, 582,  504, 616,  532, 1720,  558, 566,  526, 590,  532, 588,  558, 562,  504, 614,  530, 590,  530, 590,  530, 590,  558, 562,  532, 608,  582, 538,  582, 538,  556, 562,  532, 590,  530, 590,  556, 564,  530, 590,  558, 1694,  532, 590,  556, 564,  504, 616,  530, 1738,  532, 588,  530, 590,  532, 590,  530, 590,  532, 588,  530, 590,  530, 590,  506, 614,  530, 590,  584, 538,  530, 590,  556, 1714,  530, 1722,  556, 562,  530, 592,  528, 590,  504, 1748,  506, 1748,  528, 13566,  9000, 4576,  530, 590,  528, 590,  530, 566,  556, 1720,  530, 1722,  556, 564,  504, 616,  530, 590,  530, 590,  528, 592,  558, 1712,  532, 1720,  558, 564,  530, 590,  504, 616,  530, 1722,  530, 590,  530, 590,  532, 590,  530, 592,  530, 588,  556, 564,  558, 580,  530, 590,  530, 590,  530, 592,  528, 590,  530, 588,  530, 590,  530, 590,  530, 590,  530, 590,  530, 590,  504, 1750,  556, 582,  530, 590,  530, 590,  530, 1722,  528, 592,  530, 590,  528, 592,  528, 592,  528, 592,  530, 590,  502, 616,  528, 590,  556, 584,  530, 590,  504, 616,  530, 1722,  530, 1722,  528, 590,  530, 590,  530, 590,  530, 1722,  530, 1722,  556, 13534,  8972, 4576,  530, 592,  554, 584,  504, 616,  528, 1724,  528, 1724,  526, 592,  530, 590,  530, 590,  528, 592,  528, 594,  528, 1724,  528, 1724,  526, 592,  556, 584,  528, 592,  504, 1748,  528, 592,  530, 590,  528, 592,  528, 594,  526, 594,  528, 592,  530, 590,  528, 592,  528, 592,  556, 584,  528, 590,  530, 590,  528, 592,  528, 592,  528, 592,  528, 592,  502, 618,  528, 1724,  530, 590,  528, 592,  528, 592,  556, 1716,  528, 592,  526, 592,  528, 594,  526, 594,  528, 592,  528, 592,  528, 592,  528, 592,  530, 590,  528, 592,  528, 592,  554, 1716,  528, 1724,  528, 592,  528, 592,  528, 592,  528, 1724,  526, 1724,  528};  // UNKNOWN 3AD6EC7D

/*************************** OLED Setup ************************************/
SH1106Wire display(0x3c, SDA, SCL);

/*************************** NTP Setup ************************************/
// Set up the NTP UDP client
WiFiUDP ntpUDP;

// Define NTP properties
#define TZ   -3*60*60      // In seconds
#define NTP_INTERVAL 60*1000    // In miliseconds
#define NTP_ADDRESS  "pool.ntp.br"  // change this to whatever pool is closest (see ntp.org)

String clock_date;
const char * week_days[] = {"Dom", "Seg", "Ter", "Qua", "Qui", "Sex", "Sab"};

NTPClient timeClient(ntpUDP, NTP_ADDRESS, TZ, NTP_INTERVAL);

/************************* MQTT & Wi-Fi Setup *********************************/
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

void setup_wifi(void) {
  // Connect to Wi-Fi access point.
  delay(10);
  Serial.println();
  Serial.println("Connecting to:");
  display.clear();
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
  prev_time = millis();
  String text = "";
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    text += ".";
    display.drawString(64, 45, text);
    display.display();
    if (millis() - prev_time > 5000) {
      Serial.println("Connection Failed! Rebooting...");
      ESP.restart();
    }
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

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  sprintf(msg, "Received topic [%s] with payload [%s]", topic, payload);
  String str_payload = String((char*)payload);
  Serial.println(msg);
  if (String(topic).equals("ac/power")){
    if (str_payload.equals("OFF")){
      Serial.println("Firing ac_off");
      mqtt_client.publish("ac/state", "off");
      irsend.sendRaw(AC_off, 347, 38);
    }
  }
  if (String(topic).equals("ac/temperature")){
    if (str_payload.equals("18.0")){
      Serial.println("Firing ac_18");
      mqtt_client.publish("ac/state", "cool");
      irsend.sendRaw(AC_18, 347, 38);
    }
    if (str_payload.equals("19.0")){
      Serial.println("Firing ac_19");
      mqtt_client.publish("ac/state", "cool");
      irsend.sendRaw(AC_19, 347, 38);
    }
    if (str_payload.equals("20.0")){
      Serial.println("Firing ac_20");
      mqtt_client.publish("ac/state", "cool");
      irsend.sendRaw(AC_20, 347, 38);
    }
    if (str_payload.equals("21.0")){
      Serial.println("Firing ac_21");
      mqtt_client.publish("ac/state", "cool");
      irsend.sendRaw(AC_21, 347, 38);
    }
    if (str_payload.equals("22.0")){
      Serial.println("Firing ac_22");
      mqtt_client.publish("ac/state", "cool");
      irsend.sendRaw(AC_22, 347, 38);
    }
    if (str_payload.equals("23.0")){
      Serial.println("Firing ac_23");
      mqtt_client.publish("ac/state", "cool");
      irsend.sendRaw(AC_23, 347, 38);
    }
    if (str_payload.equals("24.0")){
      Serial.println("Firing ac_24");
      mqtt_client.publish("ac/state", "cool");
      irsend.sendRaw(AC_24, 347, 38);
    }
  }
  if (String(topic).equals("tv/power")){
    Serial.println("Firing tv_power");
    irsend.sendSAMSUNG(0xE0E040BF);
  }
  if (String(topic).equals("sound/power")){
    Serial.println("Firing sound_power");
    irsend.sendNEC(0x7E8154AB);
    irsend.sendRaw(NEC_Repeat, 3, 38);
  }
  if (String(topic).equals("sound/hdmi1")){
    Serial.println("Firing sound_hdmi1");
    irsend.sendNEC(0x5EA1E21C);
    irsend.sendRaw(NEC_Repeat, 3, 38);
  }
  if (String(topic).equals("sound/hdmi2")){
    Serial.println("Firing sound_hdmi2");
    irsend.sendNEC(0x5EA152AC);
    irsend.sendRaw(NEC_Repeat, 3, 38);
  }
  if (String(topic).equals("sound/av4")){
    Serial.println("Firing sound_av4");
    irsend.sendNEC(0x5EA13AC4);
    irsend.sendRaw(NEC_Repeat, 3, 38);
  }
  if (String(topic).equals("sound/audio1")){
    Serial.println("Firing sound_audio1");
    irsend.sendNEC(0x5EA1A658);
    irsend.sendRaw(NEC_Repeat, 3, 38);
  }
  if (String(topic).equals("sound/up")){
    Serial.println("Firing sound_up");
    irsend.sendNEC(0x5EA158A7);
    irsend.sendRaw(NEC_Repeat, 3, 38);
  }
  if (String(topic).equals("sound/down")){
    Serial.println("Firing sound_down");
    irsend.sendNEC(0x5EA1D827);
    irsend.sendRaw(NEC_Repeat, 3, 38);
  }
  if (String(topic).equals("sound/mute")){
    Serial.println("Firing sound_mute");
    irsend.sendNEC(0x5EA138C7);
    irsend.sendRaw(NEC_Repeat, 3, 38);
  }
}

void mqtt_reconnect(void) {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(DEVICE_NAME, MQTT_USERNAME, MQTT_PASS)) {
      Serial.println("Connected");
      mqtt_client.subscribe("ac/power");
      mqtt_client.subscribe("ac/temperature");
      mqtt_client.subscribe("tv/power");
      mqtt_client.subscribe("sound/power");
      mqtt_client.subscribe("sound/hdmi1");
      mqtt_client.subscribe("sound/hdmi2");
      mqtt_client.subscribe("sound/av4");
      mqtt_client.subscribe("sound/audio1");
      mqtt_client.subscribe("sound/up");
      mqtt_client.subscribe("sound/down");
      mqtt_client.subscribe("sound/mute");
      Serial.println("Feeds subscribed");
    } else {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "MQTT connection");
      display.drawString(0, 20, "failed");
      display.drawString(0, 40, "Retrying...");
      display.display();
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void ota_setup(void) {
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

void check_connections(void){
  if (WiFi.status() != WL_CONNECTED){
      setup_wifi();
    }
  if (!mqtt_client.connected()) 
      mqtt_reconnect();
}

void display_time(void) {
  display.clear(); // clear the display
  timeClient.update(); // update NTP time
  time_t t = timeClient.getEpochTime();
  clock_date = "";
  clock_date += week_days[weekday(t) - 1];
  clock_date += " - ";
  clock_date += day(t);
  clock_date += "/";
  clock_date += month(t);
  clock_date += "/";  
  clock_date += year(t);
  //Serial.println(timeClient.getFormattedTime());
  //Serial.println(clock_date);
  display.setFont(Roboto_30);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, timeClient.getFormattedTime());
  display.setFont(Roboto_15);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 40, clock_date);
  display.display();
}


void loop() {
  ArduinoOTA.handle();
  check_connections();
  mqtt_client.loop();
  if (millis() - prev_time > 5000) {
    prev_time = millis();
    sprintf(msg, "%ld", millis()/1000);
    mqtt_client.publish("ir_transmitter/status", msg);
    Serial.println("Sent uptime message");
  }
  display_time();
  delay(100);
}
