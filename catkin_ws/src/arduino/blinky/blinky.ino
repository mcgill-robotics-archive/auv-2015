#include <FastLED.h>

#define LED_COUNT       150
#define LED_OUT         13
#define LED_BRIGHTNESS  100
#define TIMEOUT_PERIOD  1000

struct CRGB leds[LED_COUNT];

// For fading in a new sketch
long lastTime;

float fadeIndex;
#define FADE_STEPS 50

void setup()
{  
  Serial.begin(57600);
  
  LEDS.addLeds<WS2812B, LED_OUT, GRB>(leds, LED_COUNT);
  LEDS.setBrightness(LED_BRIGHTNESS);
  LEDS.show();
}


void colorLoop() {  
// TODO Prismata Animation
}

void serialLoop() {
  static int pixelIndex;
  
  unsigned long lastReceiveTime = millis();

  while(true) {

    if(Serial.available() > 2) {
      lastReceiveTime = millis();

      uint8_t buffer[3]; // Buffer to store three incoming bytes used to compile a single LED color

      for (uint8_t x=0; x<3; x++) { // Read three incoming bytes
        uint8_t c = Serial.read();
        
        if (c < 255) {
          buffer[x] = c; // Using 255 as a latch semaphore
        }
        else {
          LEDS.show();
          pixelIndex = 0;
          break;
        }

        if (x == 2) {   // If we received three serial bytes
          if(pixelIndex == LED_COUNT) break; // Prevent overflow by ignoring the pixel data beyond LED_COUNT
          leds[pixelIndex] = CRGB(buffer[0], buffer[1], buffer[2]);
          pixelIndex++;
        }
      }
    }
    
    // If we haven't received data in 1 second, return to playing back our animation
    if(millis() > lastReceiveTime + TIMEOUT_PERIOD) {
      // TODO: Somehow the serial port gets trashed here, how to reset it?
      return;
    }
  }
}

void loop()
{
  // If'n we get some data, switch to passthrough mode
  if(Serial.available() > 0) {
    serialLoop();
  }
  
  // Draw the current pattern
  colorLoop();

  LEDS.show();
}

