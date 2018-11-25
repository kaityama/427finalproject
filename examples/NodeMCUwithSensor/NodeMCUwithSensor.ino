/*
 Team Litt - NodeMCU Slave Device Driver Code

 Tests polling and on-demand data collection and sending
 packets over UDP to Master NodeMCU. 

 LSM303 sensor to be replaced with BME280 sensor. IR sensor
 to be added with photocell and button sensors for the 3
 required sensors.
 
 by Kaitlyn Yamamoto
 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_LSM303_U.h>

WiFiUDP Udp;
char INCOMING_PACKET[256];  // buffer for incoming control packets
char DATA_PACKET[256];  // a reply string to send back
char ACK_PACKET[] = "ACK";  // a reply string to send back

unsigned int LOCAL_UDP_PORT = 4210;  // local port to listen on
IPAddress MASTER_IP(10, 0, 0, 174); // subject to change
int SENSOR_READ1; // Photocell
int SENSOR_READ2; // Button
int SENSOR_READ3; // TBD

const int PHOTOCELL_PIN = A0;
const int BUTTON_PIN = D4;
//const int LSM303_SCL = D1;
//const int LSM303_SDA = D2;
/* Assign a unique ID to these sensors at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(4);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(5); // assign a slave ID


const char* NETWORK = "AlliDoIsInternet"; //  your network SSID (name)
const char* PASSWORD = "monkey111";    // your network password (use for WPA, or use as key for WEP)

/* Set default values for globals */
//int     status = WL_IDLE_STATUS;
int     SLAVE_TO_MASTER = 0;

int     POLLING_RATE = 1500;
boolean POLLING = true; 
boolean ON_DEMAND = true;
boolean GET_WEATHER = false;

void setup()
{
  Serial.begin(115200);
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
  pinMode(PHOTOCELL_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);

   /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }  
}

void getWeather()
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  sensors_event_t mevent;
  mag.getEvent(&mevent);

  /* Display the results (acceleration is measured in m/s^2) */
  float aX = event.acceleration.x;
  float aY = event.acceleration.y;
  float aZ = event.acceleration.z;
  float mX = mevent.magnetic.x;
  float mY = mevent.magnetic.y;
  float mZ = mevent.magnetic.z;
  /* Check to see if SLAVE_TO_MASTER starts at 1 !!! */
  sprintf (DATA_PACKET, "Seq #: %d; Accelerometer, raw data: X = %f, Y = %f, Z = %f; Report Mode: On-Demand\n", 
              SLAVE_TO_MASTER++, aX, aY, aZ);
  sendPacket(DATA_PACKET, MASTER_IP, LOCAL_UDP_PORT);
  sprintf (DATA_PACKET, "Seq #: %d; Magnetometer, raw data: X = %f, Y = %f, Z = %f; Report Mode: On-Demand\n", 
              SLAVE_TO_MASTER++, mX, mY, mZ);
  sendPacket(DATA_PACKET, MASTER_IP, LOCAL_UDP_PORT);
}

void getSensorData()
{
  SENSOR_READ1 = analogRead(PHOTOCELL_PIN);
  SENSOR_READ2 = digitalRead(BUTTON_PIN);
  sprintf (DATA_PACKET, "Seq #: %d; Photocell, raw data: %d; Report Mode: On-Demand\n", 
                          SLAVE_TO_MASTER++, SENSOR_READ1);
  // send a sensor data to the IP address and port master is listening to
  sendPacket(DATA_PACKET, MASTER_IP, LOCAL_UDP_PORT);

  sprintf (DATA_PACKET, "Seq #: %d; Button, raw data: %d; Report Mode: On-Demand\n", 
                          SLAVE_TO_MASTER++, SENSOR_READ2);
  sendPacket(DATA_PACKET, MASTER_IP, LOCAL_UDP_PORT);
}

void parseControlMsg(char data[])
{
  char ch = data[0];
  Serial.println();
  Serial.println(ch);
  Serial.println();
  switch (ch)
  {
    case '1':
      POLLING = true; 
      ON_DEMAND = true;
      getPollingRate(data);
      Serial.println("Both on");
      Serial.println(POLLING_RATE);
      break;
    case '2':
      POLLING = true; 
      ON_DEMAND = false;
      getPollingRate(data);
      Serial.println("Polling only");
      Serial.println(POLLING_RATE);
      break;
    case '3':
      POLLING = false; 
      ON_DEMAND = true;
      Serial.println("On Demand only");
      break;
    case '4':
      POLLING = false; 
      ON_DEMAND = false;
      Serial.println("Both off");
      break;
    case '5':
      GET_WEATHER = true;
      break;
    case '6':
      if (ON_DEMAND)
      {
        getSensorData();
      }
      break;
  }
}

void getPollingRate(char data[])
{
  int pollRate = 0;
  for (int i = 1; i < 4; i++)
  {
    pollRate *= 10;
    Serial.print("Adding ");
    Serial.println(data[i]);
    pollRate += (data[i] - '0');
  }

  /* Calculate the poll rate in ms */
  //pollRate = 10000 / pollRate; 
  //pollRate = 1 / pollRate;
  POLLING_RATE = 10000 / pollRate;
  Serial.println("New Polling Rate: ");
  Serial.print("10000");
  Serial.print(" / ");
  Serial.print(pollRate);
  Serial.print(" = ");
  Serial.println(POLLING_RATE);
}

void sendPacket(char data[], IPAddress ip, unsigned int UdpPort)
{
  // send back a reply, to the IP address and port we got the packet from
  Udp.beginPacket(ip, UdpPort);
  Udp.write(data);
  Udp.endPacket();
  delay(500);
  Serial.printf("Slave-to-Master #: %d\n", SLAVE_TO_MASTER);
}

void loop()
{
  /* If Weather Data is requested */
  if (GET_WEATHER)
  {
    getWeather();
    GET_WEATHER = !GET_WEATHER;
  }
  
  /* get sensor data if polling is true */
  if (POLLING)
  {
 
    SENSOR_READ1 = analogRead(PHOTOCELL_PIN);
    SENSOR_READ2 = digitalRead(BUTTON_PIN);
    sprintf (DATA_PACKET, "Seq #: %d; Photocell, raw data: %d; Report Mode: Polling, Rate: %d\n", 
                          SLAVE_TO_MASTER++, SENSOR_READ1, POLLING_RATE);
    // send a sensor data to the IP address and port master is listening to
    sendPacket(DATA_PACKET, MASTER_IP, LOCAL_UDP_PORT);
    sprintf (DATA_PACKET, "Seq #: %d; Button, raw data: %d; Report Mode: Polling, Rate: %d\n", 
                          SLAVE_TO_MASTER++, SENSOR_READ2, POLLING_RATE);
    sendPacket(DATA_PACKET, MASTER_IP, LOCAL_UDP_PORT);
  }

  IPAddress tempIP;
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    tempIP = Udp.remoteIP();
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(INCOMING_PACKET, 256);
    if (len > 0)
    {
      INCOMING_PACKET[len] = 0;
    }
    
    Serial.printf("UDP packet contents: %s\n", INCOMING_PACKET);

    // send back a reply, to the IP address and port we got the packet from
    if (INCOMING_PACKET[0] != 'A')
    {
      parseControlMsg(INCOMING_PACKET);
      SLAVE_TO_MASTER++;
      sendPacket(ACK_PACKET, Udp.remoteIP(), Udp.remotePort());
    }
    
  }

  delay(POLLING_RATE);
}
