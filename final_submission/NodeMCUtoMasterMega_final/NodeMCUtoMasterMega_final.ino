/*
 Team Litt - Master Rx/Tx NodeMCU Device 

 Acts as a messenger for devices that interface with Master
 Mega device, including Slave NodeMCU and Android device.
 Sends UDP packets to Slave Rx/Tx NodeMCU containing
 control messages from Master Mega device to be received 
 by slave device to configure sensors.
 Receives UDP packets from Slave Rx/Tx NodeMCU containing 
 general sensor data or microphone data to be received by
 Master Mega device to control the LEDs.
 Sends and receives control messages from Master Mega device
 over UART to be received by Slave Mega device to configure
 sensors.
 Receives TCP packets from Android device to be transferred 
 over UART to Master Mega device. 
 
 by Kaitlyn Yamamoto
 */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>

/* String constants */
const char * COOL_STR = "~C";
const char * WARM_STR = "~W";
const char * IDLE_STR = "~I";
const char * ENERGYON_STR = "~E";
const char * ENERGYOFF_STR = "~F";
const char * LOCKON_STR = "~L";
const char * LOCKOFF_STR = "~M";
const char * POWERON_STR = "~P";
const char * POWEROFF_STR = "~Q";

/* Data buffers */
char INCOMING_PACKET[128];  // buffer for incoming packets
char OUTGOING_PACKET[32];  // outgoing data packets
char clientPacket[4];

/* WiFi variables */
unsigned int  LOCAL_PORT = 4210;  // local port to listen on
WiFiServer    server(LOCAL_PORT);
WiFiClient    client;
WiFiUDP       Udp;
IPAddress     SLAVE_IP(172, 20, 10, 6); // need to update to Slave device IP
const char*   NETWORK = "iPhone"; //  your network SSID (name)
const char*   PASSWORD = "MeowFi91";    // your network password (use for WPA, or use as key for WEP)

/* UART variables */
const int       rxPin = 5;  // D1
const int       txPin = 4;  // D2
SoftwareSerial  master(rxPin, txPin); // RX, TX
String          rxString;
char            c;

/* Global variables */
bool DONE_READING = false;
bool RX_MSG = false;
bool MICROPHONE_STATUS = false;
char android;

void setup()
{
  Serial.begin(9600);  
  /* Initialize WiFi connection */
  Serial.printf("Connecting to %s ", NETWORK);
  Serial.println();
  WiFi.begin(NETWORK, PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
  /* Initialize TCP server */
  server.begin();
  /* Initialize UDP port */
  Udp.begin(LOCAL_PORT);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), LOCAL_PORT);
  Serial.println();
  /* Initialize UART to Master */
  master.begin(9600);
}

/* UDP helper method */
void sendPacket(char data[], IPAddress& ip, unsigned int& UdpPort)
{
  Udp.beginPacket(ip, UdpPort);
  Udp.write(data);
  Udp.endPacket();
}

/* Data processing helper method */
void stringToCharArray(String s)
{
  /* If a microphone message, set appropriate flag */
  bool MICROPHONE = (s.charAt(0) == '#');
  for (int i = 0; i < s.length(); i++)
  {
    if (MICROPHONE)
    {
      if (s.charAt(i) == 'N')
      {
        Serial.println("Microphone on.");
        MICROPHONE_STATUS = true;
      }
      else if (s.charAt(i) == 'F')
      {
        Serial.println("Microphone off.");
        MICROPHONE_STATUS = false;
      }
    }
    OUTGOING_PACKET[i] = s.charAt(i);
  }
  for (int i = s.length(); i < sizeof(OUTGOING_PACKET); i++)
  {
    OUTGOING_PACKET[i] = 0;
  }
}

void loop()
{
  /* TCP Server - data from Android device */
  client = server.available();
  if (client)
  {
    while (client.connected())
    {
      while (client.available() > 0)
      {
        android = (char)client.read();
        switch (android)
        {
          case 'W': 
            Serial.println("Warm mode activated.");
            MICROPHONE_STATUS = true;
            strcpy(clientPacket, WARM_STR);
            master.write(clientPacket);
            break;
          case 'C':
            Serial.println("Cool mode activated.");
            MICROPHONE_STATUS = true;
            strcpy(clientPacket, COOL_STR);
            master.write(clientPacket);
            break;
          case 'I':
            Serial.println("Idle mode activated.");
            MICROPHONE_STATUS = false;
            strcpy(clientPacket, IDLE_STR);
            master.write(clientPacket);
            break;
          case 'E':
            Serial.println("Energy saver on.");
            strcpy(clientPacket, ENERGYON_STR);
            master.write(clientPacket);
            break;
          case 'F':
            Serial.println("Energy saver off.");
            strcpy(clientPacket, ENERGYOFF_STR);
            master.write(clientPacket);
            break;
          case 'L':
            Serial.println("Lockout on.");
            strcpy(clientPacket, LOCKON_STR);
            master.write(clientPacket);
            MICROPHONE_STATUS = false;
            break;
          case 'M':
            Serial.println("Lockout off.");
            strcpy(clientPacket, LOCKOFF_STR);
            master.write(clientPacket);
            MICROPHONE_STATUS = true;
            break;
          case 'P':
            Serial.println("Power on.");
            strcpy(clientPacket, POWERON_STR);
            master.write(clientPacket);
            break;
          case 'Q':
            Serial.println("Power off.");
            MICROPHONE_STATUS = false;
            strcpy(clientPacket, POWEROFF_STR);
            master.write(clientPacket);
            break;
          default:
            break;
        }
      }
    }
  }
  client.stop();
  
   /* Incoming UART packet reading and processing from Master Mega */
  if (master.available() > 0)
  {
    c = master.read();
    if (c == '#' | c == '!' | c == '&' | c == '^' | c == '@') // start message
    {
      if (RX_MSG)
      {
        Serial.print("Msg from Mega: ");
        Serial.println(rxString); // check what String is received
        
         /* put into packet and send over UDP */
        stringToCharArray(rxString); // convert string to char[]
        sendPacket(OUTGOING_PACKET, SLAVE_IP, LOCAL_PORT); // send to MasterNodeMCU
        rxString = "";
      }
      RX_MSG = true;
    }
    if (RX_MSG)
    {
      if (isDigit(c) | isAlpha(c) | isPunct(c))
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

      /* put into packet and send over UDP */
      stringToCharArray(rxString); // convert string to char[]
      sendPacket(OUTGOING_PACKET, SLAVE_IP, LOCAL_PORT); // send to MasterNodeMCU
      rxString = "";
      DONE_READING = false;
    }
  }
  
  /* Incoming UDP packet reading and processing */
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    SLAVE_IP = Udp.remoteIP();
    LOCAL_PORT = Udp.remotePort();
    // receive incoming UDP packets
    int len = Udp.read(INCOMING_PACKET, 128);
    if (len > 0)
    {
      INCOMING_PACKET[len] = 0;
    }
    Serial.println(INCOMING_PACKET);
    if (INCOMING_PACKET[0] == '#')
    {
      if (MICROPHONE_STATUS)
      {
        delay(1500);
        master.write(INCOMING_PACKET);
      }
      else
      {
        Serial.println("Microphone off, can't send.");
      }
    }
    else
    {
      delay(1500);
      master.write(INCOMING_PACKET);
    }
    INCOMING_PACKET[0] = 0;
  }
}
