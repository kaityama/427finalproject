/*
 Team Litt - Master Rx/Tx NodeMCU Device 

 Sends UDP packets to Slave Rx/Tx NodeMCU containing
 control messages from Master Uno device to be received 
 by Slave Mega device to configure sensors.
 Receives UDP packets from Slave Rx/Tx NodeMCU containing 
 general sensor data or microphone data to be received by
 Master Uno device to control the LEDs.
 Sends and receives control messages from Master Uno device
 over UART to be received by Slave Mega device to configure
 sensors.
 
 by Kaitlyn Yamamoto
 */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>

/* Data buffers */
char INCOMING_PACKET[128];  // buffer for incoming packets
char OUTGOING_PACKET[32];  // outgoing data packets

/* WiFi variables */
WiFiUDP       Udp;
IPAddress     SLAVE_IP(10, 0, 0, 174);
unsigned int  LOCAL_UDP_PORT = 4210;  // local port to listen on
const char*   NETWORK = "AlliDoIsInternet"; //  your network SSID (name)
const char*   PASSWORD = "monkey111";    // your network password (use for WPA, or use as key for WEP)

/* UART variables */
const int       rxPin = 5;  // D1
const int       txPin = 4;  // D2
SoftwareSerial  master(rxPin, txPin); // RX, TX
String          rxString;

/* Set default values for globals */
bool DONE_READING = false;
bool RX_MSG = false;

void setup()
{
  Serial.begin(9600);  
  Serial.printf("Connecting to %s ", NETWORK);
  Serial.println();
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
  master.begin(9600);
}

/* WiFi helper methods */

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
  /* WiFi Rx && UART Tx - check for data from Slave Rx/Tx NodeMCU */
  char c;
   /* Incoming UART packet reading and processing */
  if (master.available() > 0)
  {
    c = master.read();
    if (c == '!' || c == '&' || c == '^' || c == '@') // start message
    {
      if (RX_MSG)
      {
        Serial.print("Msg from Mega: ");
        Serial.println(rxString); // check what String is received
        
         /* put into packet and send over UDP */
        stringToCharArray(rxString); // convert string to char[]
        if (SLAVE_IP)
        {
          sendPacket(OUTGOING_PACKET, SLAVE_IP, LOCAL_UDP_PORT); // send to MasterNodeMCU
        }
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

      /* put into packet and send over UDP */
      stringToCharArray(rxString); // convert string to char[]
      sendPacket(OUTGOING_PACKET, SLAVE_IP, LOCAL_UDP_PORT); // send to MasterNodeMCU
      rxString = "";
      DONE_READING = false;
    }
  }
  
  /* Incoming UDP packet reading and processing */
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    SLAVE_IP = Udp.remoteIP();
    unsigned int slavePort = Udp.remotePort();
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, SLAVE_IP.toString().c_str(), slavePort);
    Serial.println();
    int len = Udp.read(INCOMING_PACKET, 128);
    if (len > 0)
    {
      INCOMING_PACKET[len] = 0;
    }
    Serial.print("UDP packet contents: ");
    Serial.println(INCOMING_PACKET);
    master.write(INCOMING_PACKET);
    INCOMING_PACKET[0] = 0;
  }
}
