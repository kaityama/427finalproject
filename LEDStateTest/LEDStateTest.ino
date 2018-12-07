#include <FastLED.h>

#define LED_PIN     9
#define NUM_LEDS    50

#define MOODY_STATE 1
#define HYPER_STATE 2
#define ALERT_STATE 3
#define CHIPSET     WS2812
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

int LED_STATUS = 0;
boolean POWER_SAVING = false;
boolean received_Power_Input = false;
boolean received_State_Input = false;
int flashColor = 0;
double brightness = 1.0;


double r_Scale = 1.0;
double g_Scale = 1.0;
double b_Scale = 1.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
}

void loop() {
  // put your main code here, to run repeatedly:
  int input = 0;
  char c;
  Serial.println("What setting would you like?");
  Serial.println("1 = Moody");
  Serial.println("2 = Hyper");
  Serial.println("3 = Alert");
  Serial.println("Nothing = No change");
  Serial.println();
  
  delay(1000);   
  if(Serial.available() > 0) {
    input = Serial.read() - '0';
    Serial.println(input);
    if(input > 0 && input < 4) {
      LED_STATUS = input;
      received_State_Input = true; 
    }
  }
 

  delay(1000);
  Serial.println("Would you like to have power saving mode turned on? (Y/N)");
  Serial.println();
  if(Serial.available()) {
    c = Serial.read();
    if(c == 'Y' || c == 'y') {
      POWER_SAVING = true;
      received_Power_Input = true;  
    } else if(c == 'N' || c == 'n') {
      received_Power_Input = true;
      POWER_SAVING = false;
    }
  }
  
  if(LED_STATUS == MOODY_STATE) {
    r_Scale = 0.5;
    g_Scale = 0.2;
    b_Scale = 1.0;
  } else if(LED_STATUS == HYPER_STATE) {
      r_Scale = 8.0;
      g_Scale = 1.0;
      b_Scale = 0.2;  
  }
  if(POWER_SAVING) {
    brightness = 1.0;
  } else {
    brightness = 0.1;
  }
  if(LED_STATUS == HYPER_STATE || LED_STATUS == MOODY_STATE) {
    //meteorRain((int) 125*r_Scale*brightness, (int) 125*g_Scale*brightness, (int) 125*b_Scale*brightness, 3, 128, true, 15, 0, NUM_LEDS); 
    RunningLights((int) 125*r_Scale*brightness, (int) 125*g_Scale*brightness, (int) 125*b_Scale*brightness, 28);
  } else if(LED_STATUS == ALERT_STATE) {
    if(flashColor == 0) {
      setAll((int) 125*brightness,0,0);
      flashColor = 1;
    } else {
      setAll(0,0,(int) 125 * brightness);
      flashColor = 0;
    }
  }
  Serial.println(LED_STATUS);
}

/* LED methods */

void showStrip() {
  #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.show();
  #endif
  #ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  FastLED.show();
  #endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    leds[i].setRGB(red, green, blue); 
  }
  FastLED.show();
}

void RunningLights(byte red, byte green, byte blue, int WaveDelay) {
  int Position=0;
  
  for(int j=0; j<NUM_LEDS*2; j++)
  {
      Position++; // = 0; //Position + Rate;
      for(int i=0; i<NUM_LEDS; i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixel(i,level,0,0);
        //float level = sin(i+Position) * 127 + 128;
        setPixel(i,((sin(i+Position) * 127 + 128)/255)*red,
                   ((sin(i+Position) * 127 + 128)/255)*green,
                   ((sin(i+Position) * 127 + 128)/255)*blue);
      }
      
      showStrip();
      delay(WaveDelay);
  }
}

void meteorRain(byte red, byte green, byte blue, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay, int startLed, int endLed) {  
  setAll(0,0,0);
  Serial.println("Starting raining!");
  for(int i = startLed; i < endLed + endLed; i++) { 
    // fade brightness all LEDs one step
    for(int j=startLed; j<endLed; j++) {
      if( (!meteorRandomDecay) || (random(10)>5) ) {
        fadeToBlack(j, meteorTrailDecay );        
      }
    }
       
    // draw meteor
    for(int j = 0; j < meteorSize; j++) {
      if( ( i-j <NUM_LEDS) && (i-j>=0) ) {
        setPixel(i-j, red, green, blue);
      } 
    }
    showStrip();
    delay(SpeedDelay);
  }
  Serial.println("Done Raining  !");
}

void fadeToBlack(int ledNo, byte fadeValue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
    // NeoPixel
    uint32_t oldColor;
    uint8_t r, g, b;
    int value;
    
    oldColor = strip.getPixelColor(ledNo);
    r = (oldColor & 0x00ff0000UL) >> 16;
    g = (oldColor & 0x0000ff00UL) >> 8;
    b = (oldColor & 0x000000ffUL);

    r=(r<=10)? 0 : (int) r-(r*fadeValue/256);
    g=(g<=10)? 0 : (int) g-(g*fadeValue/256);
    b=(b<=10)? 0 : (int) b-(b*fadeValue/256);
    
    strip.setPixelColor(ledNo, r,g,b);
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   leds[ledNo].fadeToBlackBy( fadeValue );
 #endif  me
}
