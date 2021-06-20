/*
This program was based on the Busy-Wait approach provided by Microchip Technology and modified by Leonardo Rivera
 */

//BPM: Numbers of batiments per minute
const int BPM=60;

//PWM: Define de value for maximum light intensity (0~100)
const int PWM=80;

//Define PWM led pins. In the case of Attiny85, we have 3 pins: 0,1,3
const int pins[3] = {0,1,3};

//Calculate de size of the pins array
const int len_pins = sizeof(pins)/sizeof(pins[0]);

//small_delay: delay used inside each step
const int SMALL_DELAY=7;

//get beats interval -> 1 BPM = 1 beat/(60*1000) ms
const int BEATS_INTERVAL=60000/BPM;

//get sleep_interval -> 1 BPM = 1 beat/(60*1000) ms
const long SLEEP_INTERVAL = BEATS_INTERVAL-80*SMALL_DELAY;

void setup() {
  for(uint8_t j=0; j<len_pins; j++) {
    pinMode(pins[j], OUTPUT);
  }
}

void loop() {
  for(uint8_t i=0; i<15; i++) {
      for(uint8_t j=0; j<len_pins; j++) {
        analogWrite(pins[j],((float)i/31)*((float)PWM/100)*255);
      }
      delay(SMALL_DELAY);
  }

    for(uint8_t i=15; i>5; i--) {
      for(uint8_t j=0; j<len_pins; j++) {
        analogWrite(pins[j],((float)i/31)*((float)PWM/100)*255);
        }
        delay(SMALL_DELAY);
  }

  for(uint8_t i=5; i<30; i++) {
      for(uint8_t j=0; j<len_pins; j++) {
        analogWrite(pins[j],((float)i/31)*((float)PWM/100)*255);
        }
        delay(SMALL_DELAY);
  }

  for(uint8_t i=31; i>0; i--) {
      for(uint8_t j=0; j<len_pins; j++) {
        analogWrite(pins[j],(((float)i-1.0)/31)*((float)PWM/100)*255);
        }
        delay(SMALL_DELAY);
  }
  delay(SLEEP_INTERVAL);
}
