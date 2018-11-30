//#include <ESP8266_TCP.h>

#include "arduinoFFT.h"
#include <FastLED.h>
#include <SoftwareSerial.h>

 
#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 10000 //Hz, must be less than 10000 due to ADC

#define LED_PIN     7
#define NUM_LEDS    100
 
arduinoFFT FFT = arduinoFFT();
SoftwareSerial mySerial(2, 3); // RX, TX
 
unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];
int num;
CRGB leds[NUM_LEDS];
 
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  mySerial.begin(9600);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  num = 0;
}
 
void loop() {
   if(num == NUM_LEDS - 3) {
    num = 0;
   }
    if(1 < 0) {
    /*SAMPLING*/
    for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();    //Overflows after around 70 minutes!
        // Request data from slave end
        vReal[i] = analogRead(A0);
         
        vImag[i] = 0;
        // ACK
        while(micros() < (microseconds + sampling_period_us)){
        }
    }
    

    
    /*FFT*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
 
    /*PRINT RESULTS*/
    //Serial.println(peak);     //Print out what frequency is the most dominant.
    int frequency = 0;
    for(int i=0; i<(SAMPLES/2); i++)
    {
        /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/
        
        if(vReal[i] >= 150 && vReal[i] < 5000) {
          frequency = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
          if(frequency >= 200 && frequency <= 389) {
            meteorRain(0,75,0,1, 128, true, 20, i, i + 5);
            //leds[num] = CRGB(0,75,0);
          } else if(frequency >= 390 && frequency <= 468) {
            meteorRain(75,0,32, 1, 128, true, 20, i, i +5);
            //leds[num] = CRGB(75, 0, 32);
          } else if(frequency > 493 && frequency < 1000) {
            meteorRain(0,75,75,1, 128, true, 20, i, i + 5);
            //leds[num] = CRGB(0, 75, 75);
          } else if(frequency > 1200 && frequency < 1500) {
            meteorRain(32,0,75,3, 128, true, 20, i, i + 5);
            //leds[num] = CRGB(32, 0, 75);
          } else if(frequency > 1500 && frequency < 2000) {
            meteorRain(75,75,0,1, 128, true, 20, i, i + 5);
            //leds[num] = CRGB(75, 75, 0);            
          } else if(frequency > 2000 && frequency < 2500) {
            meteorRain(75, 0,0,3, 128, true, 20, i, i + 5);
            //leds[num] = CRGB(75, 0, 0);            
          } else if(frequency > 2500 && frequency < 300) {
            meteorRain(10,75,10,3, 128, true, 20, i, i + 5);
            //leds[num] = CRGB(75,0, 0);            
          } else if(frequency > 3000 && frequency < 4500) {
            meteorRain(0, 0,75,3, 128, true, 20, i, i +5);
            //leds[num] = CRGB(0,0, 75);            
          } else if(frequency > 4500) {
            meteorRain(30,10,40,3, 128, true, 20, i, i + 5);
            //leds[num] = CRGB(75, 75, 75);
          }
          //FastLED.show();
          // fade brightness all LEDs one step
      
     
          
          if(!(frequency < 1100 && frequency > 1050)) {
          Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
          Serial.print(" ");
          Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins
          }
        }
        
    }
 
    delay(100);  //Repeat the process every second OR:
    //while(1);       //Run code once
    num++;
    }
    // RunningLights(125, 125, 125, 50);
    
    while(mySerial.available() > 0) {
      char c = mySerial.read();
      if(c == 'c') {
        meteorRain(0,0,200,3, 128, true, 20, 0, NUM_LEDS);
      }
    }
    
} 

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
    setPixel(i, red, green, blue); 
  }
  showStrip();
}

void RunningLights(byte red, byte green, byte blue, int WaveDelay) {
  int Position=0;
  for(int j=0; j<NUM_LEDS*2; j++)
  {
      
   // red = random(0, 255);
   // green = random(0, 255);
   // blue = random(0, 255);
    Position++; // = 0; //Position + Rate;
      for(int i=0; i<NUM_LEDS / 9; i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixel(i,level,0,0);
        //float level = sin(i+Position) * 127 + 128;
        setPixel(i,((sin(i+Position) * 127 + 128)/255)*red,
                   ((sin(i+Position) * 127 + 128)/255)*green*0,
                   ((sin(i+Position) * 127 + 128)/255)*blue*0);
        setPixel(i*3,((sin(i+Position) * 127 + 128)/255)*red*0,
                   ((sin(i+Position) * 127 + 128)/255)*green,
                   ((sin(i+Position) * 127 + 128)/255)*blue*0);
        setPixel(i*9,((sin(i+Position) * 127 + 128)/255)*red*0,
                   ((sin(i+Position) * 127 + 128)/255)*green*0,
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
 #endif  
}
