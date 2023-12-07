#include <FastLED.h>







  
  // put your setup code here, to run once:



void Leds(int Mode) {
  // put your main code here, to run repeatedly:
switch(Mode){
  
  case 0:
  FastLED.clear();
  FastLED.show();
  break;
 
  case 1:
    fill_solid(leds, numLeds,CRGB::Red);  
    FastLED.setBrightness(100); 
    FastLED.show();
    break;

  case 2:
    fill_solid(leds, numLeds,CRGB::White);
    FastLED.setBrightness(100);  
    FastLED.show();
    break;
  }
}


