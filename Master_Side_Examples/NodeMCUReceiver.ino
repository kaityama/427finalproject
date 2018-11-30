#include <ESP8266_TCP.h>

/*
 Team Litt - NodeMCU Master Device Driver Code

 Tests polling and on-demand data collection configuration 
 and receiving/sending packets over UDP to Slave NodeMCU. 

 LSM303 sensor to be replaced with BME280 sensor. IR sensor
 to be added with photocell and button sensors for the 3
 required sensors.

 Components to be added: keypad, LCD, buzzer, microphone, LEDs.
 
 by Kaitlyn Yamamoto
 */
//#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>

WiFiUDP Udp;
uint8_t DATA_INC[6];

char INCOMING_PACKET[256];  // buffer for incoming packets
char COMMAND_PACKET[4];
char ACK_PACKET[] = "ACK";  // a reply string to send back

IPAddress SLAVE_IP;
SoftwareSerial mySerial(2, 3); // RX, TX
unsigned int LOCAL_UDP_PORT = 4210;  // local port to listen on
//int STATUS = WL_IDLE_STATUS;

const char* NETWORK = "AlliDoIsInternet"; //  your network SSID (name)
const char* PASSWORD = "monkey111";    // your network password (use for WPA, or use as key for WEP)

/* Initialize message counters */
int MASTER_TO_SLAVE = 0;
int SLAVE_TO_MASTER = 0;
int MASTER_TO_TERMINAL = 0;
int TERMINAL_TO_MASTER = 0;

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println();
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
  Serial.println("Configuration options: 1) All On, 2) Polling only, 3) On-Demand only, 4) All Off, 5) Get Weather, 6) Get Sensor Data");
}

void bothModes()
{
  COMMAND_PACKET[0] = '1';
  getPollingRate();
}

void getPollingRate()
{
  String rate;
  Serial.println("Enter a polling rate (.1 - 10.0 Hz)");
  while (true)
  {  
    if (Serial.available() > 1)
    {
      rate = Serial.readString();
      while (Serial.available() > 0)
      {
        Serial.read();
      }
      break;
    }
  }
  char ch;
  if (rate.length() <= 4)
  {
    int arrCount = 3;
    for (int i = rate.length(); i >= 0; i--)
    { 
      ch = rate.charAt(i);
      Serial.println(ch);
      if (isDigit(ch))
      {
        Serial.println("Is Digit");
        COMMAND_PACKET[arrCount] = ch;
        Serial.println(COMMAND_PACKET[arrCount]);
        arrCount--;
      }
    }
  }
}
void pollingMode()
{
  COMMAND_PACKET[0] = '2';
  getPollingRate();
}

void onDemandMode()
{
  COMMAND_PACKET[0] = '3';
  COMMAND_PACKET[1] = '0';
  COMMAND_PACKET[2] = '0';
  COMMAND_PACKET[3] = '0';
}

void bothOff()
{
  COMMAND_PACKET[0] = '4';
  COMMAND_PACKET[1] = '0';
  COMMAND_PACKET[2] = '0';
  COMMAND_PACKET[3] = '0';
}

void weatherOn()
{
  COMMAND_PACKET[0] = '5';
  COMMAND_PACKET[1] = '0';
  COMMAND_PACKET[2] = '0';
  COMMAND_PACKET[3] = '0';
}

void getSensor()
{
  COMMAND_PACKET[0] = '6';
  COMMAND_PACKET[1] = '0';
  COMMAND_PACKET[2] = '0';
  COMMAND_PACKET[3] = '0';
}
void getMic() {
  
}

void sendPacket(char data[], IPAddress ip, unsigned int UdpPort)
{
  // send back a reply, to the IP address and port we got the packet from
  Udp.beginPacket(ip, UdpPort);
  Udp.write(data);
  Udp.endPacket();
  delay(500);
  Serial.printf("Master-to-Slave #: %d\n", ++MASTER_TO_SLAVE);
  Serial.println();
}

void loop()
{
  /* If configuration options are entered, send control message */
  if (Serial.available() > 0)
  {
    String userInput = Serial.readString();
    char option = userInput.charAt(0);
    Serial.println();
    Serial.println(option);
    Serial.println();
    switch (option)
    {
      case '1':
        bothModes();
        break;
      case '2':
        pollingMode();
        break;
      case '3':
        onDemandMode();
        break;
      case '4':
        bothOff();
        break;
      case '5':
        weatherOn();
        break;
      case '6':
        getSensor();
        break;
      case '7':
        getMic();  
      default:
        Serial.println("Invalid option. Please try again:");
        Serial.println("Configuration options: 1) All On, 2) Polling only, 3) On-Demand only, 4) All Off, 5) Get Weather, 6) Get Sensor Data");
    }
    Serial.printf("Terminal-to-Master #: %d\n", ++TERMINAL_TO_MASTER);
    Serial.println();
    // Need to add master to terminal ack...
    Serial.printf("Master-to-Terminal #: %d\n", ++MASTER_TO_TERMINAL); // temporary
    Serial.println();
    while (Serial.available() > 0)
    {
      Serial.read();
    }

    /* Send command packet to Slave */
    Serial.println(sizeof(COMMAND_PACKET));
    sendPacket(COMMAND_PACKET, SLAVE_IP, LOCAL_UDP_PORT);
  }

  /* If received data message from Slave, read packet */
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    SLAVE_IP = Udp.remoteIP();
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    Serial.println();
    int len = Udp.read(DATA_INC, 6);
    if (len > 0)
    {
      if(DATA_INC[0] == 22) {
         for(int i = 1; i < 5; i++) {
          mySerial.write(DATA_INC[1]);
         }
         //meteorRain(DATA_INC[1], DATA_INC[2], DATA_INC[3], 3, 128, true, DATA_INC[4] / 1000, 0, NUM_LEDS); 
      }
    }
    Serial.printf("UDP packet contents: %s\nSlave-to-Master #: %d\n", INCOMING_PACKET, ++SLAVE_TO_MASTER);
    Serial.println();
    /* Only send ack master-to-slave for data messages */
    if (INCOMING_PACKET[0] != 'A')
    {
      sendPacket(ACK_PACKET, Udp.remoteIP(), Udp.remotePort());
    }
  }
}
