/*
 Team Litt - Master Uno Device 
 
 Sends control messages to Master Rx/Tx NodeMCU device
 over UART to be received by Slave Mega device to configure
 sensors.
 Receives data messages from Slave Mega device containing
 general sensor data or microphone data to control the LEDs.
 These data messages are displayed to the terminal (Serial
 monitor) when received.
  
 Components to be added: keypad, LCD, buzzer, LEDs.
 
 by Kaitlyn Yamamoto
 */
#include "arduinoFFT.h"
#include <FastLED.h>
#include <SoftwareSerial.h>
#include <Keypad.h>
#include <IRremote.h>

/* Sensor pins */
const int PHOTOCELL_PIN = A0;
const int REED_PIN      = 2; 
const int BUTTON_PIN    = 3;
const int BUZZER_PIN    = 4;
const int IR_PIN        = 5;
const int LED_PIN       = 6;

/* Interrupts */
const int BUTTON_INT  = 1;

#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 10000 //Hz, must be less than 10000 due to ADC

/* Data buffers */
char OUTGOING_PACKET[32];
char COMMAND_PACKET[32];
char RATE_PACKET[16];
char SEND_POLL_RATE[8];
char SEQ_NUM[8];
char COMMAND_CODE[8];
char SENSOR_END[8];

/* LED variables */
#define NUM_LEDS    100
#define CHIPSET     WS2812
#define COLOR_ORDER GRB
#define BRIGHTNESS  128
CRGB leds[NUM_LEDS];
int ledNum;

/* FFT variables ??? may not need */
double        vReal[SAMPLES];
double        vImag[SAMPLES];
arduinoFFT    FFT = arduinoFFT();
unsigned int  sampling_period_us;
unsigned long microseconds;

/* Keypad variables */
const byte ROWS = 4; // Four rows
const byte COLS = 3; // Three columns
// Define the Keymap
const char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
// Connect keypad ROW0, ROW1, ROW2 and ROW3 to these Arduino pins.
const byte rowPins[ROWS] = { 28, 26, 24, 22 }; // 9 8 7 6 
// Connect keypad COL0, COL1 and COL2 to these Arduino pins.
const byte colPins[COLS] = { 34, 32, 30 };  // 12, 11, 10

// Create the Keypad
const Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
 
/* UART variables */
const int       rxPin = 10;
const int       txPin = 11;
SoftwareSerial  nodeMCU(rxPin, txPin); // RX, TX
String          rxString;

/* IR sensor variables */
IRrecv irDetect(IR_PIN);
decode_results irIn;

/* Global variables for message auditing */
int MASTER_TO_SLAVE = 0;
int MASTER_TO_TERMINAL = 0;
int TERMINAL_TO_MASTER = 0;
int CURR_POLL_RATE = 5000;
int CURR_POLL_HZ = 2; // Hz * 10
volatile bool DOOR_SWITCH = true;
bool ENERGY_SAVER = false;
volatile bool LOCK_OUT = false;
volatile bool LED_STATUS = false;
bool DONE_READING = false;
bool RX_MSG = false;

/* String constants */
const char * BOTH_ON      = "Y";
const char * BOTH_OFF     = "N";
const char * POLLING      = "P";
const char * ON_DEMAND    = "D";
const char * REQUEST      = "R";
const char * COMMAND_FLAG  = "!";
const char * POLLRATE_FLAG = "&";
const char * LOCKOUT_FLAG  = "^";
const char * ACK_FLAG      = "@";

void setup() {
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  irDetect.enableIRIn(); // Start the Receiver
  attachInterrupt(BUTTON_INT, BUTTON_ISR, RISING);

  Serial.begin(9600);
  nodeMCU.begin(9600);
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  ledNum = 0;
}

/* Interrupt Service Routines */

void BUTTON_ISR()
{
  /* FOR NOW, TURN LED ON AND OFF */
  if (LED_STATUS)
  {
    digitalWrite(LED_BUILTIN, LOW);
    LED_STATUS = false;
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
    LED_STATUS = true;
  }
}

/* Sensor helper methods */
void decodeIR() // Indicate what key is pressed
{
  String packet;
  switch(irIn.value)
  {
  case 0xFF629D: Serial.println("Up Arrow"); break;
  case 0xFF22DD: Serial.println("Left Arrow"); break;
  case 0xFF02FD: Serial.println("OK"); break;
  case 0xFFC23D: Serial.println("Right Arrow"); break;
  case 0xFFA857: Serial.println("Down Arrow"); break;
  case 0xFF6897: Serial.println("1"); break;
  case 0xFF9867: Serial.println("2"); break;
  case 0xFFB04F: Serial.println("3"); break;
  case 0xFF30CF: Serial.println("4"); break;
  case 0xFF18E7: Serial.println("5"); break;
  case 0xFF7A85: Serial.println("6"); break;
  case 0xFF10EF: Serial.println("7"); 
    ENERGY_SAVER = true; 
    break;
  case 0xFF38C7: Serial.println("8"); break;
  case 0xFF5AA5: Serial.println("9"); 
    LOCK_OUT = true;
    packet += LOCKOUT_FLAG;
    packet += String(++MASTER_TO_SLAVE);
    packet += "-";
    packet += "N*";
    stringToCharArray(packet);
    Serial.println("Sending to NodeMCU:");
    Serial.println(OUTGOING_PACKET);
    nodeMCU.write(OUTGOING_PACKET);
    packet = "";
    break;
  case 0xFF42BD: Serial.println("*"); 
    ENERGY_SAVER = false; 
    break;
  case 0xFF4AB5: Serial.println("0"); break;
  case 0xFF52AD: Serial.println("#"); 
    LOCK_OUT = false; 
    packet += LOCKOUT_FLAG;
    packet += String(++MASTER_TO_SLAVE);
    packet += "-";
    packet += "F*";
    stringToCharArray(packet);
    Serial.println("Sending to NodeMCU:");
    Serial.println(OUTGOING_PACKET);
    nodeMCU.write(OUTGOING_PACKET);
    packet = "";
    break; 
  default: 
    break;
  }
}

/* Control message methods */

void commandToString(const char* command)
{
  Serial.print("Terminal to Master: ");
  Serial.println((String)++TERMINAL_TO_MASTER);
  sprintf(SEQ_NUM, "%d", ++MASTER_TO_SLAVE);
  strcpy(COMMAND_CODE, "-");
  strcat(COMMAND_CODE, command);
  sprintf(SENSOR_END, "*");
  concatCommandArrays();
  Serial.println("Sending to NodeMCU: ");
  Serial.println(COMMAND_PACKET);
  nodeMCU.write(COMMAND_PACKET);
  clearCommandBuffers();
}

void concatCommandArrays()
{
  strcpy(COMMAND_PACKET, COMMAND_FLAG);
  strcat(COMMAND_PACKET, SEQ_NUM);
  strcat(COMMAND_PACKET, COMMAND_CODE);
  strcat(COMMAND_PACKET, SENSOR_END);
}

void clearCommandBuffers()
{
  COMMAND_PACKET[0] = 0;
  SEQ_NUM[0] = 0;  
  COMMAND_CODE[0] = 0;
  SENSOR_END[0] = 0;
}

void stringToCharArray(String s)
{
  for (int i = 0; i < s.length(); i++)
  {
     OUTGOING_PACKET[i] = s.charAt(i);
  }
  for (int i = s.length(); i < sizeof(OUTGOING_PACKET); i++)
  {
    OUTGOING_PACKET[i] = 0;
  }
}

/* Poll rate methods */

void decrementPollRate()
{
  --CURR_POLL_HZ;
  CURR_POLL_RATE = 10000 / CURR_POLL_HZ;
  sendPollRate(CURR_POLL_RATE);
}

void incrementPollRate()
{
  ++CURR_POLL_HZ;
  CURR_POLL_RATE = 10000 / CURR_POLL_HZ;
  sendPollRate(CURR_POLL_RATE);
}

void sendPollRate(int rate)
{
  Serial.print("Terminal to Master: ");
  Serial.println((String)++TERMINAL_TO_MASTER);
  sprintf(SEQ_NUM, "%d", ++MASTER_TO_SLAVE);
  sprintf(SEND_POLL_RATE, "-%d", rate);
  sprintf(SENSOR_END, "*");
  concatRateArrays();
  Serial.println("Sending to NodeMCU: ");
  Serial.println(RATE_PACKET);
  nodeMCU.write(RATE_PACKET);
  clearRateBuffers();
}

void concatRateArrays()
{
  strcpy(RATE_PACKET, POLLRATE_FLAG);
  strcat(RATE_PACKET, SEQ_NUM);
  strcat(RATE_PACKET, SEND_POLL_RATE);
  strcat(RATE_PACKET, SENSOR_END);
}

void clearRateBuffers()
{
  RATE_PACKET[0] = 0;
  SEQ_NUM[0] = 0;  
  SEND_POLL_RATE[0] = 0;
  SENSOR_END[0] = 0;
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

void parseTask(String task)
{
  switch (task.charAt(0))
  {
    case '#':
      char c;
      int currTotal = 0;
      int currIndex = 0;
      int values[5];
      for (int i = 1; i < task.length(); i++)
      {
        c = task.charAt(i);
        if (isDigit(c))
        {
          currTotal *= 10;
          currTotal += (c - 48);
        }
        else
        {
          values[currIndex++] = currTotal;
          currTotal = 0;
        }
      }
      /* Tests microphone parsing */
      setAll(values[1], values[2], values[3]);
      break;
  }
}

void loop() 
{
  /* One time door switch activates LEDs */
  // NEED TO CHANGE TO OPEN ACTIVATES
  if (DOOR_SWITCH)
  {
    int reed = digitalRead(REED_PIN);
    if (reed == HIGH)
    {
      Serial.println("Turning on!");
      // Turn actual LEDs on later!!!
      digitalWrite(LED_BUILTIN, HIGH); 
      DOOR_SWITCH = false;
    }
  }
  
  if (MASTER_TO_TERMINAL >= 999) 
  {
    MASTER_TO_TERMINAL = 0;
  }
  if (TERMINAL_TO_MASTER >= 999) 
  {
    TERMINAL_TO_MASTER = 0;
  }
  if (MASTER_TO_SLAVE >= 999) 
  {
    MASTER_TO_SLAVE = 0;
  }
  
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    switch (input)
    {
      case 'a': // Both on
        commandToString(BOTH_ON);
        break;
      case 'b': // Polling only
        commandToString(POLLING);
        break;
      case 'c': // On Demand only
        commandToString(ON_DEMAND);
        break;
      case 'd': // Both off
        commandToString(BOTH_OFF);
        break;
      case 'e': // Request sensor data
        commandToString(REQUEST);
        break;
    }
  }

  /* See if keypad was pressed */
  const char key = kpd.getKey();
  if (key)
  { 
    switch (key)
    {
      case '1': // decrement poll rate by .1
        if (CURR_POLL_RATE < 10000)
        {
          decrementPollRate();
        }
        break;
      case '3': // increment poll rate by .1
        if (CURR_POLL_RATE > 100)
        {
          incrementPollRate();
        }
        break;      
    }
  }

  /* If in Lockout Mode, set off alarm */
  if (LOCK_OUT)
  {
    // FLASH LEDs RED
  }

  /* If in Energy Saver Mode, use photocell */
  if (ENERGY_SAVER)
  {
    int photocellRead = analogRead(PHOTOCELL_PIN);
    photocellRead = 1023 - photocellRead;
    //now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
    int LEDbrightness = map(photocellRead, 0, 1023, 0, 255);
    analogWrite(LED_BUILTIN, LEDbrightness);
  }

  /* If controller press is detected, print FOR NOW */
  if (irDetect.decode(&irIn)) 
  {
    decodeIR();
    irDetect.resume(); 
  }
  
  char c;

  /* UART Rx - check for data from Master Rx/Tx NodeMCU */

  /* Check for UART data from NodeMCU */
  if (nodeMCU.available() > 0) 
  {
    c = nodeMCU.read();
    /* Microphone, Sensors, or Alarm */
    if (c == '#' || c == '$' || c == '^' || c == '@')
    {
      if (RX_MSG)
      {
        Serial.print("Master to Terminal: ");
        Serial.println((String)++MASTER_TO_TERMINAL);  
        Serial.println(rxString); // check what String is received
        parseTask(rxString);
        rxString = "";
      }
      RX_MSG = true;
    }

    if (RX_MSG)
    {
      if (isDigit(c) || isAlpha(c) || isPunct(c))
      {
        rxString += c;
      }
      if (c == '*')
      {
        RX_MSG = false;
        DONE_READING = true;
      }
    }
    
    /* Outgoing UDP packet processing and sending */
    if (DONE_READING)
    {
      Serial.print("Master to Terminal: ");
      Serial.println((String)++MASTER_TO_TERMINAL);  
      Serial.println(rxString); // check what String is received     
      parseTask(rxString);
      rxString = "";
      DONE_READING = false;
    }
  }
  
//   if(ledNum == NUM_LEDS - 3) {
//    ledNum = 0;
//   }
//    if(1 < 0) {
//    /* SAMPLING */
//    for(int i=0; i<SAMPLES; i++)
//    {
//        microseconds = micros();    //Overflows after around 70 minutes!
//        // Request data from slave end
//        vReal[i] = analogRead(A0);
//         
//        vImag[i] = 0;
//        // ACK
//        while(micros() < (microseconds + sampling_period_us)){
//        }
//    }
    
//    /* FFT */
//    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
//    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
//    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
//    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
// 
//    /* PRINT RESULTS */
//    //Serial.println(peak);     //Print out what frequency is the most dominant.
//    int frequency = 0;
//    for(int i=0; i<(SAMPLES/2); i++)
//    {
//        /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/
//        
//        if(vReal[i] >= 150 && vReal[i] < 5000) {
//          frequency = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
//          if(frequency >= 200 && frequency <= 389) {
//            meteorRain(0,75,0,1, 128, true, 20, i, i + 5);
//            //leds[ledNum] = CRGB(0,75,0);
//          } else if(frequency >= 390 && frequency <= 468) {
//            meteorRain(75,0,32, 1, 128, true, 20, i, i +5);
//            //leds[ledNum] = CRGB(75, 0, 32);
//          } else if(frequency > 493 && frequency < 1000) {
//            meteorRain(0,75,75,1, 128, true, 20, i, i + 5);
//            //leds[ledNum] = CRGB(0, 75, 75);
//          } else if(frequency > 1200 && frequency < 1500) {
//            meteorRain(32,0,75,3, 128, true, 20, i, i + 5);
//            //leds[ledNum] = CRGB(32, 0, 75);
//          } else if(frequency > 1500 && frequency < 2000) {
//            meteorRain(75,75,0,1, 128, true, 20, i, i + 5);
//            //leds[ledNum] = CRGB(75, 75, 0);            
//          } else if(frequency > 2000 && frequency < 2500) {
//            meteorRain(75, 0,0,3, 128, true, 20, i, i + 5);
//            //leds[ledNum] = CRGB(75, 0, 0);            
//          } else if(frequency > 2500 && frequency < 300) {
//            meteorRain(10,75,10,3, 128, true, 20, i, i + 5);
//            //leds[ledNum] = CRGB(75,0, 0);            
//          } else if(frequency > 3000 && frequency < 4500) {
//            meteorRain(0, 0,75,3, 128, true, 20, i, i +5);
//            //leds[ledNum] = CRGB(0,0, 75);            
//          } else if(frequency > 4500) {
//            meteorRain(30,10,40,3, 128, true, 20, i, i + 5);
//            //leds[ledNum] = CRGB(75, 75, 75);
//          }
//          //FastLED.show();
//          // fade brightness all LEDs one step
//              
//          if(!(frequency < 1100 && frequency > 1050)) {
//          Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
//          Serial.print(" ");
//          Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins
//          }
//        }
//        
//    }
// 
//    delay(100);  //Repeat the process every second OR:
//    //while(1);       //Run code once
//    ledNum++;
//    }
    // RunningLights(125, 125, 125, 50);
} 
