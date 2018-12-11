/*
 Team Litt - Master Mega Device 

 Master device that flashes LEDs based on input
 from microphone data. Includes control from terminal,
 IR controller, keypad, and Android app.
 Sends control messages to Master Rx/Tx NodeMCU device
 over UART to be received by Slave NodeMCU device to configure
 sensors and actuators.
 Receives data messages from slave device containing
 general sensor data or microphone data to control the LEDs.
 These data messages are displayed to the terminal (Serial
 monitor) when received.
 
 by Kaitlyn Yamamoto and Drew Scott
 */ 
#include <FastLED.h>
#include <SoftwareSerial.h>
#include <Keypad.h>
#include <IRremote.h>
#include <LiquidCrystal_I2C.h>

/* Sensor pins */
const int PHOTOCELL_PIN = A0;
const int REED_PIN      = 2; 
const int BUTTON_PIN    = 3;
const int IR_PIN        = 5;
const int LED_PIN       = 6;

/* Interrupts */
const int BUTTON_INT  = 1;

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
volatile int R;
volatile int G;
volatile int B;
volatile double R_SCALE = 1.0;
volatile double G_SCALE = 1.0;
volatile double B_SCALE = 1.0;
double brightness = 1.0;

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
const Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
 
/* UART variables */
const int       rxPin = 10;
const int       txPin = 11;
SoftwareSerial  nodeMCU(rxPin, txPin); // RX, TX
String          rxString;

/* IR sensor variables */
IRrecv irDetect(IR_PIN);
decode_results irIn;

/* LCD Display */
LiquidCrystal_I2C lcd(0x27,20,2); // set the LCD address to 0x27 for 20 x 4

/* Global variables for message auditing */
int MASTER_TO_SLAVE = 0;
int MASTER_TO_TERMINAL = 0;
int TERMINAL_TO_MASTER = 0;
int CURR_POLL_RATE = 5000;
int CURR_POLL_HZ = 2; // Hz * 10
volatile bool DOOR_SWITCH = true;
volatile bool ENERGY_SAVER = false;
volatile bool LOCK_OUT = false;
volatile bool IDLE_MODE = false;
volatile bool COOL_MODE = false;
volatile bool WARM_MODE = false;
volatile bool LED_STATUS = false;
bool DONE_READING = false;
bool RX_MSG = false;
bool RECEIVED_COMMAND = false;

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
const char * MICROPHONE_FLAG = "#";
const char WELCOME_MSG1[]  = "Welcome! Use our app";
const char WELCOME_MSG2[]  = "for best experience";

void setup() {
  /* Initialize pins and components */
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  irDetect.enableIRIn(); // Start the Receiver
  attachInterrupt(BUTTON_INT, BUTTON_ISR, RISING);
  lcd.init();
  lcd.backlight();
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  
  /* Initialize communication */
  Serial.begin(9600);
  nodeMCU.begin(9600);
}

/* Interrupt Service Routine for Button */
void BUTTON_ISR()
{
  /* Turn LEDs off */
  if (LED_STATUS)
  {
    IDLE_MODE = false;
    WARM_MODE = false;
    COOL_MODE = false;
    setAll(0,0,0);
    LED_STATUS = false;
  }
  /* Turn LEDs on */
  else
  {
    IDLE_MODE = false;
    WARM_MODE = false;
    COOL_MODE = false;
    LED_STATUS = true;
  }
}

/* For IR Sensor controller mapping */
void decodeIR() // Indicate what key is pressed
{
  switch(irIn.value)
  {
    case 0xFF629D: Serial.println("Up Arrow"); break;
    case 0xFF22DD: Serial.println("Left Arrow"); break;
    case 0xFF02FD: Serial.println("OK"); break;
    case 0xFFC23D: Serial.println("Right Arrow"); break;
    case 0xFFA857: Serial.println("Down Arrow"); break;
    case 0xFF6897: Serial.println("1"); 
      Serial.println("Warm mode activated!");
      WARM_MODE = true;
      LED_STATUS = true;
      LOCK_OUT = false;
      COOL_MODE = false;
      IDLE_MODE = false;
      stringToCharArray(buildString(MICROPHONE_FLAG, 'N'));
      Serial.println("Sending to NodeMCU:");
      Serial.println(OUTGOING_PACKET);
      nodeMCU.write(OUTGOING_PACKET);
      break;
    case 0xFF9867: Serial.println("2"); 
      Serial.println("Cool Mode Activated!");
      COOL_MODE = true;
      LED_STATUS = true;
      WARM_MODE = false;
      IDLE_MODE = false;
      stringToCharArray(buildString(MICROPHONE_FLAG, 'N'));
      Serial.println("Sending to NodeMCU:");
      Serial.println(OUTGOING_PACKET);
      nodeMCU.write(OUTGOING_PACKET);
      break;
    case 0xFFB04F: Serial.println("3"); 
      Serial.println("Idle Mode Activated!");
      IDLE_MODE = true;
      LOCK_OUT = false;
      LED_STATUS = true;
      COOL_MODE = false;
      WARM_MODE = false;
      stringToCharArray(buildString(MICROPHONE_FLAG, 'F'));
      Serial.println("Sending to NodeMCU:");
      Serial.println(OUTGOING_PACKET);
      nodeMCU.write(OUTGOING_PACKET);
      break;
    case 0xFF30CF: Serial.println("4"); break;
    case 0xFF18E7: Serial.println("5"); break;
    case 0xFF7A85: Serial.println("6"); break;
    case 0xFF10EF: Serial.println("7"); 
      ENERGY_SAVER = true; 
      break;
    case 0xFF38C7: Serial.println("8"); 
      IDLE_MODE = false;
      WARM_MODE = false;
      COOL_MODE = false;
      setAll(125, 125, 125);
      LED_STATUS = true;
      stringToCharArray(buildString(MICROPHONE_FLAG, 'F'));
      Serial.println("Sending to NodeMCU:");
      Serial.println(OUTGOING_PACKET);
      nodeMCU.write(OUTGOING_PACKET);
      break;
    case 0xFF5AA5: Serial.println("9"); 
      LOCK_OUT = true;
      LED_STATUS = true;
      IDLE_MODE = false;
      R = 255;
      G = 0;
      B = 0;
      stringToCharArray(buildString(LOCKOUT_FLAG, 'N'));
      Serial.println("Sending to NodeMCU:");
      Serial.println(OUTGOING_PACKET);
      nodeMCU.write(OUTGOING_PACKET);
      break;
    case 0xFF42BD: Serial.println("*"); 
      ENERGY_SAVER = false; 
      break;
    case 0xFF4AB5: Serial.println("0"); 
      setAll(0,0,0);
      IDLE_MODE = false;
      COOL_MODE = false;
      LOCK_OUT = false;
      WARM_MODE = false;
      LED_STATUS = false;
      stringToCharArray(buildString(MICROPHONE_FLAG, 'F'));
      Serial.println("Sending to NodeMCU:");
      Serial.println(OUTGOING_PACKET);
      nodeMCU.write(OUTGOING_PACKET);
      break;
    case 0xFF52AD: Serial.println("#"); 
      LOCK_OUT = false; 
      stringToCharArray(buildString(LOCKOUT_FLAG, 'F'));
      Serial.println("Sending to NodeMCU:");
      Serial.println(OUTGOING_PACKET);
      nodeMCU.write(OUTGOING_PACKET);
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

/* Data processing helper methods */

/* Parse incoming data message from slave */
void parseTask(String task)
{
  if(!IDLE_MODE && !LOCK_OUT && LED_STATUS) {
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
        R = values[1];
        G = values[2];
        B = values[3];
        chooseLEDMethod(values[4]);
        break;
    }
    delay(50);
  }
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

String buildString(const char * flag, char onOff)
{
  String str = ""; 
  str += flag;
  str += String(++MASTER_TO_SLAVE);
  str += "-";
  str += onOff;
  str += "*";
  return str;
}

/* LED methods */

void showStrip() {
  #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.show();
  #endif
  #ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  if(WARM_MODE) {
    if(B > 50) {
      B = 50;
    }
  }
  if(COOL_MODE) {
    if(G > 50) {
      G = 50;
    }
  }
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
      for(int i=0; i<NUM_LEDS / 9; i++) {
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

/* LED Method Selection */
void chooseLEDMethod(int freq) {
  FadeInOut((int) (R_SCALE*R*brightness),( int) (G_SCALE*G*brightness) , (int) (B_SCALE*B*(brightness)));
}

void FadeInOut(byte red, byte green, byte blue){
  float r, g, b;
      
  for(int k = 0; k < 256; k=k+4) { 
    r = (k/256.0)*red;
    g = (k/256.0)*green;
    b = (k/256.0)*blue;
    setAll(r,g,b);
    showStrip();
  }
     
  for(int k = 255; k >= 0; k=k-8) {
    r = (k/256.0)*red;
    g = (k/256.0)*green;
    b = (k/256.0)*blue;
    setAll(r,g,b);
    showStrip();
  }
}

void LockOutLED() {
  G = 0;
  int temp = R;
  R = B;
  B = temp;
  setAll(R, G, B);
  delay(500);
}

void colorWipe(byte red, byte green, byte blue, int SpeedDelay) {
  for(uint16_t i=0; i<NUM_LEDS; i++) {
      setPixel(i, red, green, blue);
      showStrip();
      delay(SpeedDelay);
  }
}

void loop() 
{
  /* Door switch activates system */
  while(DOOR_SWITCH)
  {
    int reed = digitalRead(REED_PIN);
    if (reed == HIGH)
    {
      Serial.println("Turning on!");
      lcd.setCursor(15,0); // set the cursor to column 15, line 0
      for (int positionCounter1 = 0; positionCounter1 < 20; positionCounter1++)
      {
        lcd.scrollDisplayLeft(); //Scrolls the contents of the display one space to the left.
        lcd.print(WELCOME_MSG1[positionCounter1]); // Print a message to the LCD.
        delay(600); //wait for 250 microseconds
      }
      lcd.clear(); //Clears the LCD screen and positions the cursor in the 
      lcd.setCursor(15,1); // set the cursor to column 15, line 1
      for (int positionCounter = 0; positionCounter < 20; positionCounter++)
      {
        lcd.scrollDisplayLeft(); //Scrolls the contents of the display one space to the left.
        lcd.print(WELCOME_MSG2[positionCounter]); // Print a message to the LCD.
        delay(600); //wait for 250 microseconds
      }
      lcd.clear(); //Clears the LCD screen and positions the cursor in the upper-left corner.
      digitalWrite(LED_BUILTIN, HIGH); 
      DOOR_SWITCH = false;
      IDLE_MODE = true;    
    }  
  }

  /* If sequence numbers exceed 999, reset to 0. */
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

  /* Terminal commands for configurations. */
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    switch (input)
    {
      case 'a': // Both on
        commandToString(BOTH_ON);
        RECEIVED_COMMAND = false;
        break;
      case 'b': // Polling only
        commandToString(POLLING);
        RECEIVED_COMMAND = false;
        break;
      case 'c': // On Demand only
        commandToString(ON_DEMAND);
        RECEIVED_COMMAND = false;
        break;
      case 'd': // Both off
        commandToString(BOTH_OFF);
        RECEIVED_COMMAND = false;
        break;
      case 'e': // Request sensor data
        commandToString(REQUEST);
        RECEIVED_COMMAND = false;
        break;
      case 'g': // Idle Mode
        Serial.println("Idle Mode!");
        WARM_MODE = false;
        IDLE_MODE = true;
        COOL_MODE = false;
        stringToCharArray(buildString(MICROPHONE_FLAG, 'F'));
        nodeMCU.write(OUTGOING_PACKET);
        RECEIVED_COMMAND = false;
        break;
      case 'f': // Microphone turn on Warm Mode
        Serial.println("Warm Mode!");
        WARM_MODE = true;
        IDLE_MODE = false;
        COOL_MODE = false;
        stringToCharArray(buildString(MICROPHONE_FLAG, 'N'));
        nodeMCU.write(OUTGOING_PACKET);
        RECEIVED_COMMAND = false;
        break;
    } 
  }
  
  /* IDLE MODE */
  if(IDLE_MODE && !RECEIVED_COMMAND) {
    R_SCALE = G_SCALE = B_SCALE = 1.0;
    colorWipe(125,125,125, 6);
    delay(100);
    setAll(0,0,0);
    delay(1000);
    
    /* Checks for IR controller commands */
    if (irDetect.decode(&irIn)) 
      {
        decodeIR();
        irDetect.resume(); 
      }
      
    /* Checks for terminal commands */
    if(nodeMCU.available() > 0) {
      RECEIVED_COMMAND = true;  
    } else if(Serial.available()) {
      if(Serial.read() != '\n') {
        RECEIVED_COMMAND = true;
      }
    }
  }

  /* Keypad commands to change poll rate */
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
    LockOutLED();
  }

  /* If WARM mode on, set RGB scalars to warm */
  if(WARM_MODE) {
    R_SCALE = 0.8;
    G_SCALE = 1.0;
    B_SCALE = 0.2; 
  }

  /* If COOL mode on, set RGB scalars to cool */
  if(COOL_MODE) {
    R_SCALE = 0.3;
    G_SCALE = 0.2;
    B_SCALE = 1.0;
  }

  /* If in Energy Saver Mode, use photocell */
  if (ENERGY_SAVER)
  {
    double photoCellValue = (1023) -  (double) analogRead(A0);
    int tempBrightness = map(photoCellValue, 0, 1023, 0, 1023);
    brightness = (double) tempBrightness / 1023;
    
  }

  /* IR controller commands */
  if (irDetect.decode(&irIn)) 
  {
    RECEIVED_COMMAND = false;
    decodeIR();
    irDetect.resume(); 
  }
  
  /* UART Rx - check for data from Master Rx/Tx NodeMCU */
  
  char c;
  /* Check for UART data from NodeMCU */
  if (RECEIVED_COMMAND | nodeMCU.available() > 0) 
  {
    c = nodeMCU.read();
    /* Android command received */
    if (c == '~')
    {
      char mode = nodeMCU.read();
      Serial.println(mode);
      switch(mode)
      {
        case 'W':
          Serial.println("Warm Mode!");
          WARM_MODE = true;
          IDLE_MODE = false;
          LED_STATUS = true;
          COOL_MODE = false;
          stringToCharArray(buildString(MICROPHONE_FLAG, 'N'));
          nodeMCU.write(OUTGOING_PACKET);
          RECEIVED_COMMAND = false;
          break;
        case 'C':
          Serial.println("Cool Mode!");
          WARM_MODE = false;
          IDLE_MODE = false;
          LED_STATUS = true;
          COOL_MODE = true;
          stringToCharArray(buildString(MICROPHONE_FLAG, 'N'));
          nodeMCU.write(OUTGOING_PACKET);
          RECEIVED_COMMAND = false;
          break;
        case 'I':
          Serial.println("Idle Mode!");
          WARM_MODE = false;
          IDLE_MODE = true;
          LED_STATUS = true;
          COOL_MODE = false;
          stringToCharArray(buildString(MICROPHONE_FLAG, 'F'));
          nodeMCU.write(OUTGOING_PACKET);
          RECEIVED_COMMAND = false;
          break;
        case 'E':
          Serial.println("Energy Saving!");
          ENERGY_SAVER = true;
          RECEIVED_COMMAND = false;
          break;
        case 'F': // Energy saving off
          ENERGY_SAVER = false;
          RECEIVED_COMMAND = false;
          break;
        case 'L': // Lockout mode on
          LOCK_OUT = true;
          RECEIVED_COMMAND = false;
          break;
        case 'M': // Lockout mode off
          LOCK_OUT = false;
          RECEIVED_COMMAND = false;
          break;
        case 'P': // Power LEDs on
          LED_STATUS = true;
          setAll(125, 125, 125);
          RECEIVED_COMMAND = false;
          break;
        case 'Q': // Power LEDs off
          LED_STATUS = false;
          IDLE_MODE = false;
          WARM_MODE = false;
          COOL_MODE = false;
          RECEIVED_COMMAND = false;
          setAll(0,0,0);
          break;
        default :
          break;
      }
    }
    /* Microphone, Sensors, Alarm, or Ack data received */
    if ((c == '#' && !IDLE_MODE) || c == '$' || c == '^' || c == '@')
    {
      RECEIVED_COMMAND = false;
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
      RECEIVED_COMMAND = false;
    }
  }
  delay(200);
} 
