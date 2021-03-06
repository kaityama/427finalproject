/*
 Team Litt - Slave Rx/Tx NodeMCU Device 
 
 Sends UDP packets to Master Rx/Tx NodeMCU containing 
 general sensor and microphone data. 
 Receives UDP packets from Master Rx/Tx NodeMCU containing
 control messages from Master Uno device to be received 
 by Slave Mega device to configure sensors.
 Sends and receives data messages from Slave Mega device
 over UART to be received by Master Uno device for
 displaying.
 by Kaitlyn Yamamoto
 */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/* Sensor pins */
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
unsigned long delayTime;
String sending;

/* Sequence number */
int     SLAVE_TO_MASTER = 0;

/* Global flags for sensor configuration */
uint16_t CURR_POLLING_RATE = 5000;
boolean POLLING   = false; 
boolean ON_DEMAND = false;
boolean LOCK_OUT  = false;

/* WiFi variables */
WiFiUDP       Udp;
IPAddress     MASTER_IP(10, 0, 0, 125); // subject to change.. see if this works without setting master
unsigned int  LOCAL_UDP_PORT = 4210;  // local port to listen on
const char*   NETWORK = "AlliDoIsInternet"; //  your network SSID (name)
const char*   PASSWORD = "monkey111";    // your network password (use for WPA, or use as key for WEP)

void setup() 
{
  pinMode(BUZZER_PIN, OUTPUT);
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

  bool status;
  status = bme.begin();  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }
}

/* On-demand sensor reading methods */

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

/* Packet helper methods for packets received/sent */

void interpretPacket(char data[])
{
  char command = data[0];
  switch (command)
  {
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

void sendPacket(char data[], IPAddress& ip, unsigned int& UdpPort)
{
  // send back a reply, to the IP address and port we got the packet from
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
  if (POLLING)
  {
    getWeather(POLLING_STR);
    delay(CURR_POLLING_RATE);
  }

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
  
  /* WiFi Rx && UART Tx - check for data from Master Rx/Tx NodeMCU */

  /* Incoming UDP packet reading and processing */
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
//    MASTER_IP = Udp.remoteIP();
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
    if (INCOMING_PACKET[0] != '@')
    {
      String ackPacket;
      ackPacket += ACK_FLAG;
      ackPacket += String(++SLAVE_TO_MASTER);
      ackPacket += "-";
      ackPacket += INCOMING_PACKET;
      ackPacket += "*";
      stringToCharArray(ackPacket);
      sendPacket(OUTGOING_PACKET, MASTER_IP, masterPort);
      ackPacket = "";
    }

    INCOMING_PACKET[0] = 0;
  }
}
