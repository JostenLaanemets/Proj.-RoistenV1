#include <FastLED.h>


void Leds(int Mode) {
  
  switch(Mode){
    // Clear all the LEDs
    case 0:
    FastLED.clear();
    FastLED.show();
    break;
    // Set the LEDs to red
    case 1:
      fill_solid(leds, numLeds,CRGB::Red);  
      FastLED.setBrightness(100); 
      FastLED.show();
      break;
    // Set the LEDs to white
    case 2:
      fill_solid(leds, numLeds,CRGB::White);
      FastLED.setBrightness(100);  
      FastLED.show();
      break;
  }
}


