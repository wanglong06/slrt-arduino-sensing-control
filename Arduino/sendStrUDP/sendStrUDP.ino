/*
 UDPSendReceiveString:
 This sketch receives UDP message strings, prints them to the serial port
 and sends an "acknowledge" string back to the sender

 A Processing sketch is included at the end of file that can be used to send
 and received messages for testing with a computer.

 created 21 Aug 2010
 by Michael Margolis

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

unsigned int localPort = 10001;      // local port to listen on
const unsigned int SEND_MSG_SIZE = 6;
// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
// char  SendBuffer[] = "acknowledgedacknowledgedacknowledgedacknowledged";       // a string to send back
char  SendBuffer[100];  // a string to send back
float DataToSend[] = {
  1.1, 1.2, 1.3, 1.4, 1.5, 1.6
};

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

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

    // send a reply to the IP address and port that sent us the packet we received
    Serial.println("Size of data to send:");
    Serial.println(sizeof(DataToSend));    
    pack_UDP_message(SendBuffer,DataToSend);
    Serial.println("Size of msg buf:");
    Serial.println(sizeof(SendBuffer));
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(SendBuffer,sizeof(DataToSend));
    Udp.endPacket();
  }
  delay(10);
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
