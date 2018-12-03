/*
 Team Litt - Slave Mega Device 
 
 Sends data messages to Slave Rx/Tx NodeMCU device
 over UART to be received by Master Uno device to be
 displayed in the Serial monitor.
 Receives control messages from Master Uno device containing
 configuration information for the sensors.
 These control messages are displayed to the Serial
 monitor when received.
  
 Sensors to be integrated: IR sensor, reed switch. 
 
 by Kaitlyn Yamamoto
 */
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "arduinoFFT.h"

/* Sensor pins */
const int PHOTOCELL_PIN = A0;
const int MICROPHONE_PIN = A1;
const int REED_PIN = 2;
const int BUTTON_PIN = 4;

/* Data buffers */
char SENSOR_PACKET[64];
char MICROPHONE_PACKET[32];
char SENSOR_READ[16];
char SEQ_NUM[8];
char SENSOR_CODE[8];
char UNITS[8];
char REPORT_MODE[8];
char POLL_RATE[8];
char SENSOR_END[8];
char RED[8];
char GREEN[8];
char BLUE[8];
char FREQ[8];

/* String constants */
const char * PHOTOCELL_STR = "PCL";
const char * BUTTON_STR = "BTN";
const char * INFRARED_STR = "IRD";
const char * TEMPERATURE_STR = "TMP";
const char * HUMIDITY_STR = "HMD";
const char * PRESSURE_STR = "PRS";
const char * POLLING_STR = "PG";
const char * DEMAND_STR = "OD";
const char * SENSOR_FLAG = "$";
const char * MICROPHONE_FLAG = "#";

/* Microphone variables */
#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 10000 //Hz, must be less than 10000 due to ADC
#define NUM_LEDS    100
arduinoFFT FFT = arduinoFFT();
double vReal[SAMPLES];
double vImag[SAMPLES];
int micNum;
unsigned int sampling_period_us;
unsigned long microseconds;

/* UART variables */
const int       rxPin = 10;
const int       txPin = 11;
SoftwareSerial  nodeMCU (rxPin, txPin);
String          rxString;
bool            DONE_READING = false;
bool            RX_MSG = false;

/* I2C variables */
Adafruit_BME280 bme; 
unsigned long delayTime;

/* Global flags for sensor configuration */
int     CURR_POLLING_RATE = 5000;
boolean POLLING = true; 
boolean ON_DEMAND = true;
boolean GET_WEATHER = false;

/* Sequence number */
int     SLAVE_TO_MASTER = 0;

/* Sensor reads */
int PHOTOCELL_READ; // Photocell
int BUTTON_READ; // Button
int MICROPHONE_READ; // Microphone 
int DOOR_READ; // Reed switch 
float TEMPERATURE_READ; // Temp
float HUMIDITY_READ; // Humidity 
float PRESSURE_READ; // Pressure 

void setup() {
  Serial.begin(9600);
  Serial.println();
  nodeMCU.begin(9600);
  bool status;
  status = bme.begin();  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }
  pinMode(PHOTOCELL_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(MICROPHONE_PIN, INPUT);
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  micNum = 0;
}

/* Polling sensor reading methods */
void getMicrophone()
{
  strcpy(MICROPHONE_PACKET, MICROPHONE_FLAG);
  sprintf(SEQ_NUM, "%d-", ++SLAVE_TO_MASTER);
  sprintf(RED, "%d-", random(0,255));
  sprintf(GREEN, "%d-", random(0,255));
  sprintf(BLUE, "%d-", random(0,255));
  sprintf(FREQ, "%d*", 20);
  strcat(MICROPHONE_PACKET, SEQ_NUM);
  strcat(MICROPHONE_PACKET, RED);
  strcat(MICROPHONE_PACKET, GREEN);
  strcat(MICROPHONE_PACKET, BLUE);
  strcat(MICROPHONE_PACKET, FREQ);
  Serial.println("Sending Microphone Data: ");
  Serial.println(MICROPHONE_PACKET);
  delay(500);
  nodeMCU.write(MICROPHONE_PACKET);
  MICROPHONE_PACKET[0] = 0;
  SEQ_NUM[0] = 0;
  RED[0] = 0;
  GREEN[0] = 0;
  BLUE[0] = 0;
  FREQ[0] = 0;
}

/* On-demand sensor reading methods */

void getWeather(const char * reportMode)
{
  weatherToString(TEMPERATURE_STR, bme.readTemperature(), reportMode);
  weatherToString(PRESSURE_STR, bme.readPressure() / 100.0F, reportMode);
  weatherToString(HUMIDITY_STR, bme.readHumidity(), reportMode);
}

void getSensorData()
{
  PHOTOCELL_READ = analogRead(PHOTOCELL_PIN);
  BUTTON_READ = digitalRead(BUTTON_PIN);
  dataToString(PHOTOCELL_STR, PHOTOCELL_READ, DEMAND_STR);
  dataToString(BUTTON_STR, BUTTON_READ, DEMAND_STR);
}

/* UART helper methods */

void dataToString(const char* sensor, int sensorRead, const char* reportMode)
{
  sprintf(SEQ_NUM, "%d", ++SLAVE_TO_MASTER);
  strcpy(SENSOR_CODE, "-");
  strcat(SENSOR_CODE, sensor);
  sprintf(SENSOR_READ, "-%d", sensorRead);
  strcpy(REPORT_MODE, "-");
  strcat(REPORT_MODE, reportMode);  
  
  if (reportMode == "PG")
  {
      sprintf(POLL_RATE, "-%d", CURR_POLLING_RATE);
  }
  sprintf(SENSOR_END, "*");
  concatSensorArrays();
  Serial.println("Sending to NodeMCU: ");
  Serial.println(SENSOR_PACKET);
  nodeMCU.write(SENSOR_PACKET);
  clearSensorBuffers();
}

void weatherToString(const char* sensor, float sensorRead, const char * reportMode)
{
  sprintf(SEQ_NUM, "%d", ++SLAVE_TO_MASTER);
  strcpy(SENSOR_CODE, "-");
  strcat(SENSOR_CODE, sensor);
  sprintf(SENSOR_READ, "-%d.%02d", (int)sensorRead, (uint8_t)(sensorRead*100)%100);
    
  switch (sensor[0])
  {
    case 'T':
      strcpy(UNITS, " C");
      break;
    case 'H':
      strcpy(UNITS, " %");
      break;
    case 'P':
      strcpy(UNITS, " hPa");
      break;
  }
  strcpy(REPORT_MODE, "-");
  strcat(REPORT_MODE, reportMode);
  if (reportMode == "PG")
  {
    sprintf(POLL_RATE, "-%d", CURR_POLLING_RATE);
  }
  sprintf(SENSOR_END, "*");
  concatSensorArrays();
  Serial.println("Sending to NodeMCU: ");
  Serial.println(SENSOR_PACKET);
  nodeMCU.write(SENSOR_PACKET);
  clearSensorBuffers();
}

void concatSensorArrays()
{
  strcpy(SENSOR_PACKET, SENSOR_FLAG);
  strcat(SENSOR_PACKET, SEQ_NUM);
  strcat(SENSOR_PACKET, SENSOR_CODE);
  strcat(SENSOR_PACKET, SENSOR_READ);
  strcat(SENSOR_PACKET, UNITS);
  strcat(SENSOR_PACKET, REPORT_MODE);
  strcat(SENSOR_PACKET, POLL_RATE);
  strcat(SENSOR_PACKET, SENSOR_END);
}

void clearSensorBuffers()
{
  SENSOR_PACKET[0] = 0;
  SEQ_NUM[0] = 0;  
  SENSOR_CODE[0] = 0;
  SENSOR_READ[0] = 0;
  UNITS[0] = 0;
  REPORT_MODE[0] = 0;
  POLL_RATE[0] = 0;
  SENSOR_END[0] = 0;
}

void parseTask(String task)
{
  switch (task.charAt(0))
  {
    case '!':
      char c;
      char command;
      for (int i = 3; i < task.length(); i++)
      {
        c = task.charAt(i);
        if (isAlpha(c))
        {
          command = c;  
        }
      }
      switch(command)
      {
        case 'Y':
          POLLING = true;
          ON_DEMAND = true;
          break;
        case 'N':
          POLLING = false;
          ON_DEMAND = false;
          break;
        case 'P':
          POLLING = true;
          ON_DEMAND = false;
          break;
        case 'D':
          POLLING = false;
          ON_DEMAND = true;
          break;
        case 'R':
          if (ON_DEMAND)
          {
            getWeather(DEMAND_STR);
          }
          break;
      }
//      Serial.print("Command Msg from Uno: ");    
//      Serial.println(task);
      break;
    case '&':
      int values[2];
      int currIndex = 0;
      int currTotal = 0;
      int currNum = 0;
      for (int i = 1; i < task.length(); i++)
      {
        currNum = task.charAt(i);
        if (isDigit(currNum))
        {
          currTotal *= 10;
          currTotal += (currNum - 48);
        }
        else
        {
          values[currIndex++] = currTotal;
          currTotal = 0;
        }
      }
      CURR_POLLING_RATE = values[1];
//      Serial.print("Poll Rate Msg from Uno: ");    
//      Serial.println(task);
      break;
  }
}

void loop() {
  if (SLAVE_TO_MASTER >= 999) 
  {
    SLAVE_TO_MASTER = 0;
  }

//  delay(3000);
//  getMicrophone();
  
  /* get sensor data if polling is true */
//  if (POLLING)
//  {
//    getWeather(POLLING_STR);
//    delay(CURR_POLLING_RATE);
//  }
  
  
  char c;
  
  /* UART Rx - check for data from Slave Rx/Tx NodeMCU */

   /* Check for UART data from NodeMCU */
  if (nodeMCU.available() > 0) 
  {
    c = nodeMCU.read();
    if (c == '!' || c == '&' || c == '@') // Microphone message
    {
      if (RX_MSG)
      {
        Serial.print("Msg from Mega: ");
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
      Serial.print("Msg from Mega: ");
      Serial.println(rxString); // check what String is received
      /* Add to queue to be processed */
      parseTask(rxString);
      rxString = "";
      DONE_READING = false;
    }
  }
}
