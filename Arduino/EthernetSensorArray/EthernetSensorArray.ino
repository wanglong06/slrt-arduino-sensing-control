/*
  Ethernet Motor Controller
  By Colette Abah, Long Wang, 2018/1/25, ARMA LAb

  This code integrates ethernet commnication and sensor array using multiplexer:
  More details of documentation can be found
  https://github.com/wanglong06/slrt-arduino-sensing-control/tree/master/Arduino/EthernetSensorArray

  This code does the following:
  (1) read the sensor array via i2c protocol
  (2) stream to an ip addres via UDP protocol
  (3) in future, an interrupt scheme to ensure "hard" real-time may be considered

*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
// Includes needed for Robogaia.com
// the sensor communicates using SPI, so include the library:
// #include <SPI.h>

// Includes needed for Ethernet
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

// Include needed for i2c
#include <Wire.h>//Include the Wire library to talk I2C

// Include needed for the proximity sensor
//*****************************************************
extern "C" {
#include "Adafruit_VL6180X.h" //Library from Adafruit. Adapt this library for whatever sensor you are reading
}

//*****************************************************
// Declare and initialize variables needed for Ethernet communication
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);
IPAddress sendIP(192, 168, 1, 65); // IP address to report motor status
unsigned int localPort = 10001;      // local port to listen on
unsigned int sendPort = 10002;      // remote port to send to
//const unsigned int SEND_MSG_SIZE = 6; // message size

// declare buffers for receiving and sending data
char packetBuffer[100];  //buffer to hold incoming packet,
char  SendBuffer[100];  // a string to send back

// Declare the UDP object
EthernetUDP Udp;
//*****************************************************

//*****************************************************
// Declare and initialize variables needed for Proximity Sensor arrays
uint8_t default_tca_addr = 0x70;
const int N_MPX =  2; //Number of multiplexers
const int N_SNSR = 4; //Number of sensors per multiplexer
Adafruit_VL6180X ToF_Sensors[N_MPX][N_SNSR];
uint8_t tca_addr[N_MPX];

// time and test value
unsigned long TimeIdx = 0; //

//*****************************************************
void setup()
//*****************************************************
{
  // setup the i2c
  Wire.begin();

  // Create sensor objects based on the number of multiplexers and sensors
  for (int j = 0; j < N_MPX; j++) {
    tca_addr[j] = default_tca_addr + j;
    //Serial.println(tca_addr[j],HEX);
    for (int k = 0; k < N_SNSR; k++) {
      ToF_Sensors[j][k] = Adafruit_VL6180X(); //j = number of multiplexers, k = number of sensors
    }
  }

  // Establish communication with ports of the multiplexers*/
  for (int j = 0; j < N_MPX; j++)
  { // loop on multiplexer
    for (int k = 0; k < N_SNSR; k++)
    { // loop on sensors
      tcaselect(k,  tca_addr[j]);
      if (! ToF_Sensors[j][k].begin()) {
        Serial.print("!!! ERROR !!! No sensor found on port "); Serial.println(k);
        Serial.print(" Multiplexer #"); Serial.print(j + 1); Serial.print("(0x"); Serial.print(tca_addr[j], HEX); Serial.println(")");
        Serial.println("------------------------------------");
      }
      else {
        Serial.print("Sensor found on port "); Serial.println(k);
        Serial.print(" Multiplexer #"); Serial.print(j + 1); Serial.print("(0x"); Serial.print(tca_addr[j], HEX); Serial.println(")");
        Serial.println("------------------------------------");
      }
    }
  }

  // setup for Ethernet UDP
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  Serial.begin(9600);

  delay(100);

}

//*****************************************************
void loop()
//*****************************************************
{
  // Read sensors
  uint8_t DataToSend[N_MPX][N_SNSR];
  uint8_t range;
  uint8_t status;

  for (int j = 0; j < N_MPX; j++) {
    for (int k = 0; k < N_SNSR; k++) {
      //float lux = ToF_Sensors[j][k].readLux(VL6180X_ALS_GAIN_5);
      //Serial.print("Lux: "); Serial.println(lux);
      tca_disable (tca_addr[j]); //Turn off all ports
      tcaselect(k,  tca_addr[j]); // Transmit to port k on multiplexer j

      range = ToF_Sensors[j][k].readRange();
      DataToSend[j][k] = range;
      status = ToF_Sensors[j][k].readRangeStatus();
      //Serial.println(status)
      //Serial.println(loop_count);
//      sensorStatMsgDisp_ToF(status,j,k,range);
    }
  }


  // Report the robot status via Ethernet
  // send a reply to the IP address and port that sent us the packet we received
  pack_UDP_message(SendBuffer, DataToSend);
  Udp.beginPacket(sendIP, sendPort);
  Udp.write(SendBuffer, sizeof(DataToSend));
  Udp.endPacket();



  // Display the status for debugging purposes
  //    Serial.print("q1: ");
  //    Serial.print(motorJointRad[0]);
  //    Serial.print("\r\n");
  TimeIdx = TimeIdx + 1;

}//end loop

// Function to select a port on a multiplexer
void tcaselect(uint8_t i, uint8_t address)
//*************************************************
{
  if (i > 7) return;
  Wire.beginTransmission(address);
  Wire.write(1 << i);
  Wire.endTransmission();
}//end func
//*************************************************

// Function to disable all the ports on a multiplexer
void tca_disable(uint8_t address)
//*************************************************
{
  Wire.beginTransmission(address);
  Wire.write(0);  // no channel selected
  Wire.endTransmission();
}//end func
//*************************************************

// pack UDP message to the buffer for sending sensor data
//*************************************************
void pack_UDP_message(char * SendBuffer, const uint8_t DataToSend[N_MPX][N_SNSR])
//*************************************************
{
  // zero out the buffer
  memset(SendBuffer, 0, N_MPX * N_SNSR * sizeof(uint8_t));
  for (int j = 0; j < N_MPX; j++)
  {
    for (int k = 0; k < N_SNSR; k++)
    {
      int i = (j * N_SNSR  + k);
      memcpy(SendBuffer +  i * sizeof(uint8_t), 
      &(DataToSend[j][k]), sizeof(uint8_t));
    }
  }
}//end func

// sensor status message display of Time of Flight
//*************************************************
void sensorStatMsgDisp_ToF(uint8_t status,int j, int k, int range)
{
  if (status == VL6180X_ERROR_NONE) {
    Serial.print("[Multiplexer "); Serial.print(j + 1);  Serial.print(",Port "); Serial.print(k); Serial.print("] : ");
    Serial.print(range); Serial.println(" mm");
//    continue;
  }

  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }
}// end func
