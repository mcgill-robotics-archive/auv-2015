#include <FastLED.h>
#include "sin_table.h"

#define NUM_LED_PER_STRIP   25
#define NUM_LED_STRIP       7
#define LED_COUNT           NUM_LED_PER_STRIP * NUM_LED_STRIP
#define LED_OUT_PIN         13
#define LED_BRIGHTNESS      40
#define BAR_SIZE            3

#define TIMEOUT_PERIOD      2000
#define PHASE_STEP          50
#define FRACTION_BIT_MASK   0x7FF
#define NUM_FRACTION_BIT    11

struct CRGB leds[LED_COUNT];
uint32_t segmentPhase[NUM_LED_PER_STRIP];

long lastTime;

void setup()
{
  Serial.begin(115200);
  LEDS.addLeds<WS2812B, LED_OUT_PIN, GRB>(leds, LED_COUNT);
  LEDS.setBrightness(LED_BRIGHTNESS);
  LEDS.show();
}

uint8_t mapIntensity(uint16_t amplitude) {

  uint16_t fraction = amplitude & FRACTION_BIT_MASK;
  //Serial.println(fraction);
  if (fraction > (1 << 10)) {
    return (uint8_t) (((1 << 11) - fraction) >> 2);
  } else {
    return (uint8_t) (fraction >> 2);
  }
}

//  The LEDs are organized in a snake zigzag shape
//  setLedColor map xy coordinate into correct index
//   +------------x 
//   |
//   |   0 >  1 >  2 >  3 >  4
//   |                       |
//   |                       |
//   |   9 <  8 <  7 <  6 <  5
//   |   |
//   y   |
//      10 > 11 > 12 > 13 > 14
//                          |
//                          |
//      19 < 18 < 17 < 16 < 15


void setLedColor(int x, int y, CRGB color) {
  if (y % 2) {
    leds[NUM_LED_PER_STRIP * (y + 1) - x - 1] = color;
  } else {
    leds[NUM_LED_PER_STRIP * y + x] = color;
  }
}

void colorLoop() {

  // clear all led to black
  for (uint16_t i = 0; i < LED_COUNT; i++) {
    leds[i] = CRGB::Black;
  }

  for (uint8_t i = 0; i < NUM_LED_PER_STRIP; i++) {

    // Increment the phase for each led
    segmentPhase[i] = (segmentPhase[i] + i + PHASE_STEP) & 0x0FFF;

    // Determin which led to turn on, and its intensity
    uint16_t amplitude = NUM_LED_STRIP * SIN_TABLE_256[segmentPhase[i] >> 4];
    uint8_t pixelIndex = (amplitude >> NUM_FRACTION_BIT);
    // Put on the correct led
    setLedColor(i, pixelIndex, CRGB(255, 0, 0));
  }
}
  
void drawBar(uint8_t location, bool isOnLeft){
  if(location > (NUM_LED_STRIP - BAR_SIZE)){
    return;
  }  
  uint8_t x = 1;
  if(isOnLeft) {
    x = 23;
  }
  for (uint8_t i = 0; i < BAR_SIZE; i++) {
    setLedColor(x,location + i, CRGB(255, 0, 0));
  }
}

void drawPong(uint8_t x, uint8_t y){
  if(y > 12 ){
    y = 12;
  }
  if(x > 6){
    return;
  }
  setLedColor(2 * y, x, CRGB(255, 255, 255));
}

void serialLoop() {

  unsigned long lastReceiveTime = millis();

  static uint8_t buff;
  while (true) {
    if (buff != 255) {
      if (Serial.available()) {
        buff = Serial.read();
      }
    }
    
    if(buff == 255){
       if(Serial.available() > 3){
          for (uint16_t i = 0; i < LED_COUNT; i++) {
            leds[i] = CRGB::Black;
          }
          
          drawBar(Serial.read(),false);
          drawBar(Serial.read(),true);
          drawPong(Serial.read(),Serial.read());
          buff = 0;
          lastReceiveTime = millis();
          LEDS.show();
       }
    }
    
    
    // If we haven't received data in 1 second, return to playing back our animation
    if (millis() > lastReceiveTime + TIMEOUT_PERIOD) {
      while(Serial.available()){
        Serial.read();
      }
      return;
    }
  }
}

void loop()
{
  // If we get some data, switch to passthrough mode
  if (Serial.available()) {
    serialLoop();
  }

  // Draw the current pattern
  colorLoop();

  LEDS.show();
  //delay(1);
}

