/*
This program was based on the Busy-Wait approach provided by Microchip Technology and modified by Leonardo Rivera
 */

//BPM: Numbers of batiments per minute
#define  BPM 60

//PWM: Define de value for maximum light intensity (0~100)
#define  PWM 80

//Define PWM led pins. In the case of Attiny85, we have 3 pins: 0,1,4
const int pins[3] = {0,1,4};

//Define pin for using the touch
#define  pin_touch 2

//Calculate de size of the pins array
const int len_pins = sizeof(pins)/sizeof(pins[0]);

//small_delay: delay used inside each step
#define SMALL_DELAY 7

//get beats interval -> 1 BPM = 1 beat/(60*1000) ms
const int BEATS_INTERVAL=60000/BPM;

//get sleep_interval -> 1 BPM = 1 beat/(60*1000) ms
const int SLEEP_INTERVAL = BEATS_INTERVAL-80*SMALL_DELAY;

void setup() {
  for(uint8_t j=0; j<len_pins; j++) {
    pinMode(pins[j], OUTPUT);
  }
  pinMode(pin_touch, INPUT);
}

void loop() {
  int touchState = digitalRead(pin_touch);
  
  if(touchState == HIGH){
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
}
