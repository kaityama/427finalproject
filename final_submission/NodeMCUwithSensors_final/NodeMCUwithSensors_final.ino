/*
 Team Litt - Slave NodeMCU Device 

 Slave device that collects data from microphone sensor
 to be sent to Master device to drive LEDs. Also contains
 BME280 weather sensor and a Piezo buzzer for the alarm
 lockout mode.
 Sends UDP packets to Master Rx/Tx NodeMCU containing 
 general sensor and microphone data. 
 Receives UDP packets from Master Rx/Tx NodeMCU containing
 control messages from Master device to configure sensors.
 
 by Kaitlyn Yamamoto and Drew Scott
 */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "arduinoFFT.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/* FFT variables */
#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 6000 //Hz, must be less than 10000 due to ADC
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
int R = 0;
int G = 0;
int B = 0;

/* Actuator pin */
const int BUZZER_PIN = D0;

/* Data Buffers */
char INCOMING_PACKET[128];  // buffer for incoming control packets, from Master NodeMCU
char OUTGOING_PACKET[32];  // outgoing data packets, data from mega sending to Master NodeMCU

/* String constants */
const char * TEMPERATURE_STR = "TMP";
const char * HUMIDITY_STR = "HMD";
const char * PRESSURE_STR = "PRS";
const char * POLLING_STR = "PG";
const char * DEMAND_STR = "OD";
const char * SENSOR_FLAG = "$";
const char * MICROPHONE_FLAG = "#";
const char * ACK_FLAG = "@";

/* I2C variables */
Adafruit_BME280 bme; 

/* Global variables */
String    sending;
uint16_t  CURR_POLLING_RATE = 5000;
int       SLAVE_TO_MASTER = 0; 
boolean   POLLING   = false; 
boolean   ON_DEMAND = false;
boolean   LOCK_OUT  = false;
boolean   MICROPHONE_STATUS = false;

/* WiFi variables */
WiFiUDP       Udp;
IPAddress     MASTER_IP(172, 20, 10, 4); // need to update this with master IP
unsigned int  LOCAL_UDP_PORT = 4210;  // local port to listen on
const char*   NETWORK = "iPhone"; //  your network SSID (name)
const char*   PASSWORD = "MeowFi91";    // your network password (use for WPA, or use as key for WEP)

void setup() 
{
  /* Initialize sensors */
  pinMode(BUZZER_PIN, OUTPUT);
  bool status;
  status = bme.begin();  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }
  /* Communications */
  Serial.begin(9600);
  Serial.printf("Connecting to %s ", NETWORK);
  WiFi.begin(NETWORK, PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
  Udp.begin(LOCAL_UDP_PORT);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), LOCAL_UDP_PORT);
  Serial.println();

  /* For FFT */
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
}

/* Sensor reading methods */

void getWeather(const char * reportMode)
{
  weatherToString(TEMPERATURE_STR, bme.readTemperature(), reportMode);
  weatherToString(PRESSURE_STR, bme.readPressure() / 100.0F, reportMode);
  weatherToString(HUMIDITY_STR, bme.readHumidity(), reportMode);
}

void weatherToString(const char* sensor, float sensorRead, const char * reportMode)
{
  sending += SENSOR_FLAG;
  sending += String(++SLAVE_TO_MASTER);
  sending += "-";
  sending += sensor;
  sending += "-";
  sending += String(sensorRead);
      
  switch (sensor[0])
  {
    case 'T':
      sending += " C-";
      break;
    case 'H':
      sending += " %-";
      break;
    case 'P':
      sending += " hPa-";
      break;
  }
  sending += reportMode;
  if (reportMode == "PG")
  {
    sending += "-";
    sending += String(CURR_POLLING_RATE);
  }
  sending += "*";
  stringToCharArray(sending);
  Serial.println("Sending to NodeMCU: ");
  Serial.println(OUTGOING_PACKET);
  sendPacket(OUTGOING_PACKET, MASTER_IP, LOCAL_UDP_PORT);
  sending = "";
}

/* Microphone helper methods */

void getMicrophone(int frequency)
{
  noInterrupts();
  String temp = "";
  temp += MICROPHONE_FLAG;
  temp += String(++SLAVE_TO_MASTER);
  temp += "-";
  temp += R + '0';
  temp += "-";
  temp += G + '0';
  temp += "-";
  temp += B + '0';
  temp += "-";
  temp += frequency + '0';
  temp += "*";
  stringToCharArray(temp);
  Serial.println("Sending Microphone Data: ");
  Serial.println(OUTGOING_PACKET);
  sendPacket(OUTGOING_PACKET, MASTER_IP, LOCAL_UDP_PORT);
  temp = "";
  interrupts();
}

void frequencyToRGB(int frequency) {
  boolean setRGBValue = false;
  while(!setRGBValue) {
    if(frequency < 45) { // Red
      R = 250;
      G = 0;
      B = 0;
      setRGBValue = true;
    } else if(frequency < 93) { // Red-Orange
      R = 250;
      G = 69;
      B = 0;
      setRGBValue = true;
    } else if(frequency < 142) { // Orange
      R = 250;
      G = 125;
      B = 0;
      setRGBValue = true;
    } else if(frequency < 188) { // Yellow
      R = 250;
      G = 250;
      B = 0;
      setRGBValue = true;
    } else if(frequency < 235) { // Spring Green
      R = 125;
      G =  250;
      B = 0;
      setRGBValue = true;
    } else if(frequency < 282) { // Green
      R = 0;
      G = 250;
      B = 0;
      setRGBValue = true;
    } else if(frequency < 329) { // Turqoise
      R = 0;
      G = 250;
      B = 125;
      setRGBValue = true;
    } else if(frequency < 375) { // Cyan
      R = 0;
      G = 250;
      B = 250;
      setRGBValue = true;
    } else if(frequency < 422) { // Ocean
      R = 0;
      G = 125;
      B = 250;
      setRGBValue = true;
    } else if(frequency < 469) { // Blue
      R = 0;
      G = 0;
      B = 250;
      setRGBValue = true;
    } else if(frequency < 516) { // Raspberry
      R = 250;
      G = 0;
      B = 125;
      setRGBValue = true;
    } else if(frequency < 563) { // Indigo
      R = 75;
      G = 0;
      B = 130;
      setRGBValue = true;
    } else if(frequency < 610) { //Violet
      R = 125;
      G = 0;
      B = 250; 
      setRGBValue = true;
    } else {
      frequency = frequency / 10;
    }
  }
}

/* Packet helper methods for packets received/sent */

void interpretPacket(char data[])
{
  char command = data[0];
  switch (command)
  {
    case '#': // Mic
      processMicrophone(data);
      break;
    case '!': // Configuration
      processCommand(data);
      break;
    case '&': // Polling Rate
      processPollingRate(data);
      break;
    case '^': // Alarm 
      processAlarm(data);
      break; 
    case '@': // Ack
      Serial.println(data);
      break;
  }
}

void processMicrophone(char data[]) {
  char option;
  for (int i = 1; i < strlen(data); i++)
  {
    option = data[i];
    if (isAlpha(option))
    {
      switch (option)
      {
        case 'N': MICROPHONE_STATUS = true; break;
        case 'F': MICROPHONE_STATUS = false; break;
      }
    }
  }
}

void processCommand(char data[])
{
  char curr;
  for (int i = 1; i < strlen(data); i++)
  {
    curr = data[i];
    if (isAlpha(curr))
    {
      switch (curr)
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
          ON_DEMAND = true;
          POLLING = false;
          break;
        case 'R':
          if (ON_DEMAND)
          {
            getWeather(DEMAND_STR);
          }
          break;
      }
    }
  }
}

void processPollingRate(char data[])
{
  uint16_t nums[2];
  uint16_t currSum = 0;
  int currIndex = 0;
  char curr;
  for (int i = 1; i < strlen(data); i++)
  {
    curr = data[i];
    if (isDigit(curr))
    {
      currSum *= 10;
      currSum += (curr - 48);
    }
    else
    {
      nums[currIndex++] = currSum;
      currSum = 0;
    }
  }
  
  CURR_POLLING_RATE = nums[1];
  Serial.print("New polling rate = ");
  Serial.println(CURR_POLLING_RATE);
}

void processAlarm(char data[])
{
  char option;
  for (int i = 1; i < strlen(data); i++)
  {
    option = data[i];
    if (isAlpha(option))
    {
      switch (option)
      {
        case 'N': LOCK_OUT = true; break;
        case 'F': LOCK_OUT = false; break;
      }
    }
  }
}

/* For sending UDP packets to Master */
void sendPacket(char data[], IPAddress& ip, unsigned int& UdpPort)
{
  Udp.beginPacket(ip, UdpPort);
  Udp.write(data);
  Udp.endPacket();
}

/* Data processing helper methods */

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

void loop()
{
  /* FFT */
  for(int i=0; i<SAMPLES; i++)
  {
    microseconds = micros();    //Overflows after around 70 minutes!
    vReal[i] = analogRead(A0);
    vImag[i] = 0;
    while(micros() < (microseconds + sampling_period_us)){
    }
  }

  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  int idx = (peak * SAMPLES) / (1.0 * SAMPLING_FREQUENCY);
  int frequency = peak;
  if(vReal[idx] >= 150) {
    frequencyToRGB(frequency);
    if(MICROPHONE_STATUS) {
      getMicrophone(frequency);
      delay(1000);
    }
  }
    
  /* Polling mode on */
  if (POLLING)
  {
    getWeather(POLLING_STR);
    delay(CURR_POLLING_RATE);
  }

  /* Alarm on */
  if (LOCK_OUT)
  {
    tone(BUZZER_PIN, 1000);
    delay(500);
    noTone(BUZZER_PIN);
    delay(500);
    tone(BUZZER_PIN, 1000);
    delay(500);
    noTone(BUZZER_PIN);
    delay(500);
    tone(BUZZER_PIN, 1000);
    delay(500);
    noTone(BUZZER_PIN);
    delay(500);
  }
  
  /* WiFi Rx - check for data from Master Rx/Tx NodeMCU */

  /* Incoming UDP packet reading and processing */
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    MASTER_IP = Udp.remoteIP();
    unsigned int masterPort = Udp.remotePort();
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, MASTER_IP.toString().c_str(), masterPort);
    int len = Udp.read(INCOMING_PACKET, 128);
    if (len > 0)
    {
      INCOMING_PACKET[len] = 0;
    }
    Serial.print("UDP packet contents: ");
    Serial.println(INCOMING_PACKET);
    interpretPacket(INCOMING_PACKET);
    /* Send an ack for non-ack messages received */
    if (INCOMING_PACKET[0] != '@')
    {
      String ackPacket;
      ackPacket += ACK_FLAG;
      ackPacket += String(++SLAVE_TO_MASTER);
      ackPacket += "*";
      stringToCharArray(ackPacket);
      sendPacket(OUTGOING_PACKET, MASTER_IP, masterPort);
      ackPacket = "";
    }
    INCOMING_PACKET[0] = 0;
  }
}
