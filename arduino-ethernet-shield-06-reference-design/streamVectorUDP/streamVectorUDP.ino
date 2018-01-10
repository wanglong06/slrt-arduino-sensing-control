/*
 UDPSendReceiveVectors:
 This sketch sends UDP datagram of number arrays

 A Processing sketch is included at the end of file that can be used to send
 and received messages for testing with a computer.

 created 8 Jan 2018
 by Long Wang

 This code is in the public domain.
 */

#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);
IPAddress sendIP(192, 168, 1, 64);
unsigned int localPort = 10001;      // local port to listen on
unsigned int sendPort = 10002;      // local port to listen on
const unsigned int SEND_MSG_SIZE = 6;
// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
// char  SendBuffer[] = "acknowledgedacknowledgedacknowledgedacknowledged";       // a string to send back
char  SendBuffer[100];  // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
unsigned long Time = 0;
float value = 0;
void setup() {
  // start the Ethernet and UDP:
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  Serial.begin(9600);
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);
  }
  
  // send a reply to the IP address and port that sent us the packet we received
  float DataToSend[] = {
  value, value+10.0, value+20.0, value+30.0, value+40.0, value+50.0
  };
  pack_UDP_message(SendBuffer,DataToSend);
  Udp.beginPacket(sendIP, sendPort);
  Udp.write(SendBuffer,sizeof(DataToSend));
  Udp.endPacket();
  
  Serial.write(27);       // ESC command
  Serial.print("Current Value 1: ");    // clear screen command
  Serial.write(27);
  Serial.println(value);
  delay(10);
  Time = Time + 1;
  value = value + 0.05;
}

void pack_UDP_message(char * SendBuffer, const float DataToSend[SEND_MSG_SIZE])
{
  // zero out the buffer
  memset(SendBuffer, 0, SEND_MSG_SIZE*sizeof(float));
  for(int i=0; i<SEND_MSG_SIZE; i++)
  {
    memcpy(SendBuffer+i*sizeof(float), &(DataToSend[i]), sizeof(float));
  }
}   
