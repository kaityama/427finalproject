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

/* Data Buffers */
char INCOMING_PACKET[128];  // buffer for incoming control packets, from Master NodeMCU
char OUTGOING_PACKET[32];  // outgoing data packets, data from mega sending to Master NodeMCU

/* WiFi variables */
WiFiUDP       Udp;
IPAddress     MASTER_IP(10, 0, 0, 174); // subject to change.. see if this works without setting master
unsigned int  LOCAL_UDP_PORT = 4210;  // local port to listen on
const char*   NETWORK = "AlliDoIsInternet"; //  your network SSID (name)
const char*   PASSWORD = "monkey111";    // your network password (use for WPA, or use as key for WEP)

/* UART variables */
const int         rxPin = 5;  // NodeMCU D1 -> Mega D11
const int         txPin = 4;  // NodeMCU D2 -> Mega D10
SoftwareSerial    mega(rxPin, txPin);
String            rxString;

/* Set default values for globals */
bool RX_MSG = false;
bool DONE_READING = false;

void setup() 
{
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println();

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
  mega.begin(9600);
}

/* Packet helper methods for packets received/sent */

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
  char * hello = "!7-D*";
  Serial.println("Sending to Mega: ");
  Serial.println(hello);
  mega.write(hello);
  char * hello2 = "!8-R*";
  Serial.println("Sending to Mega: ");
  Serial.println(hello2);
  mega.write(hello2);
  
  char c;
  
  /* UART Rx && WiFi Tx - check for data from Mega */

  /* Check for UART data from Mega */
  if (mega.available() > 0) 
  {
    c = mega.read();
    
    if (c == '#' | c == '$' | c == '@') // start message
    {
      if (RX_MSG)
      {
        Serial.print("Msg from Mega: ");
        Serial.println(rxString); // check what String is received
        /* put into packet and send over UDP */
        stringToCharArray(rxString); // convert string to char[]
        sendPacket(OUTGOING_PACKET, MASTER_IP, LOCAL_UDP_PORT); // send to MasterNodeMCU
        Serial.print("Msg to Master: ");
        Serial.println(OUTGOING_PACKET); // check what String is received
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
      sendPacket(OUTGOING_PACKET, MASTER_IP, LOCAL_UDP_PORT); // send to MasterNodeMCU
      Serial.print("Msg to Master: ");
      Serial.println(OUTGOING_PACKET); // check what String is received
      rxString = "";
      DONE_READING = false;
    }
  }
  
  /* WiFi Rx && UART Tx - check for data from Master Rx/Tx NodeMCU */

  /* Incoming UDP packet reading and processing */
//  int packetSize = Udp.parsePacket();
//  if (packetSize)
//  {
////    MASTER_IP = Udp.remoteIP();
//    unsigned int masterPort = Udp.remotePort();
//    // receive incoming UDP packets
//    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, MASTER_IP.toString().c_str(), masterPort);
//    int len = Udp.read(INCOMING_PACKET, 128);
//    if (len > 0)
//    {
//      INCOMING_PACKET[len] = 0;
//    }
//    
//    Serial.print("UDP packet contents: ");
//    Serial.println(INCOMING_PACKET);
//    
//    /* Outgoing UART packet processing and sending */
//    mega.write(INCOMING_PACKET); // see if this works.....
//    INCOMING_PACKET[0] = 0;
//    Serial.println();
//
//  }
}
