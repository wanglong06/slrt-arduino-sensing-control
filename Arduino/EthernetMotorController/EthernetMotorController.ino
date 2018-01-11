/*
 Ethernet Motor Controller
 By Long Wang, 2018/1/11, ARMA LAb
 
 This code integrates ethernet commnication and motor control from primarily the following two sources:
 (1) Robogaia.com
 (2) Ethernet Shield V1

 This code does the following:
 (1) receive command via Ethernet
 (2) read encoder
 (3) output PWM control signal to an amplifier
 (4) PID motor control
 
 */
 
// Includes needed for Robogaia.com
// the sensor communicates using SPI, so include the library:
#include <SPI.h>

// Includes needed for Ethernet
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

// Includes needed for PID
#include <PID_v1.h>

//*****************************************************  
// Declare variables needed for Encoder reading
int chipSelectPin1=10;
int chipSelectPin2=9;
int chipSelectPin3=8;
const unsigned int COUNT_PER_TURN[3] = {256,256,256}; // Encoder spec - counts per turn
const unsigned int QUAD_MODE[3] = {4,4,4}; // Encoder spec - Quadrature mode
//*****************************************************  

//*****************************************************  
// Declare and initialize variables needed for Ethernet communication
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);  
IPAddress sendIP(192, 168, 1, 64); // IP address to report motor status 
unsigned int localPort = 10001;      // local port to listen on
unsigned int sendPort = 10002;      // remote port to send to
const unsigned int SEND_MSG_SIZE = 6; // message size

// declare buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char  SendBuffer[100];  // a string to send back

// Declare the UDP object
EthernetUDP Udp;
//*****************************************************  

// time and test value
unsigned long Time = 0; // 
float value = 0;

//*****************************************************  
// Declare variables needed for PID controller
bool enablePID = false;
double motorJointRad[] = {0.0, 0.0, 0.0};
double desiredJointRad[] = {0.0, 0.0, 0.0};
double controlOuput[] = {0.0, 0.0, 0.0};
double Kp[] = {2,2,2};
double Ki[] = {0,0,0};
double Kd[] = {0,0,0};
// Specify the links and initial tuning parameters (PID Gains)
// PID joint1PID(&Input, &Output, &Setpoint, Kp_gain, Ki_gain, Kd_gain, DIRECT);
PID joint1PID(motorJointRad, controlOuput, desiredJointRad, Kp[0], Ki[0], Kd[0], DIRECT);
PID joint2PID(motorJointRad+1, controlOuput+1, desiredJointRad+1, Kp[1], Ki[1], Kd[1], DIRECT);
PID joint3PID(motorJointRad+2, controlOuput+2, desiredJointRad+2, Kp[2], Ki[2], Kd[2], DIRECT);
//*****************************************************  

//*****************************************************
void setup() 
//*****************************************************
{
  // setup the PID controllers
  joint1PID.SetMode(AUTOMATIC);
  joint2PID.SetMode(AUTOMATIC);
  joint3PID.SetMode(AUTOMATIC);
  
  // setup for Ethernet UDP
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  Serial.begin(9600);

  // setup for encoder reading
  pinMode(chipSelectPin1, OUTPUT);
  pinMode(chipSelectPin2, OUTPUT);
  pinMode(chipSelectPin3, OUTPUT);
  
  digitalWrite(chipSelectPin1, HIGH);
  digitalWrite(chipSelectPin2, HIGH);
  digitalWrite(chipSelectPin3, HIGH);

  LS7366_Init();

  delay(100);

}

//*****************************************************
void loop() 
//*****************************************************
{
  // Encoder reading and motor angle calculation
    long encoderValue[3];
    for (int i = 0; i < 3; i++) {
      encoderValue[i] = getEncoderValue(i); 
      motorJointRad[i] = (float)encoderValue[i]/(COUNT_PER_TURN[i]*QUAD_MODE[i]);
    }
  
  // Receive command from master 
    enablePID = true;
    double udp_command[] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; i++) {
      desiredJointRad[i] = udp_command[i];
    }
    
  // PID controller
    if (enablePID){
      // joint1PID.compute(); // the first axis is not using
      joint2PID.Compute();
      joint3PID.Compute();
      }

  // Report the robot status via Ethernet
    // send a reply to the IP address and port that sent us the packet we received
    double DataToSend[] = {
      motorJointRad[0], motorJointRad[1], motorJointRad[2],
      controlOuput[0], controlOuput[1], controlOuput[2]
    };
    pack_UDP_message(SendBuffer,DataToSend);
    Udp.beginPacket(sendIP, sendPort);
    Udp.write(SendBuffer,sizeof(DataToSend));
    Udp.endPacket();
  
  // Apply the computed control action to robot.
  
    delay(10); 
 
}//end loop



  
//*****************************************************  
long getEncoderValue(int encoder)
//*****************************************************
{
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;
    
    selectEncoder(encoder);
    
    SPI.transfer(0x60); // Request count
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte
    
    deselectEncoder(encoder);

    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
    
    return result;
}//end func

//*************************************************
void selectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,LOW);
        break;
     case 2:
       digitalWrite(chipSelectPin2,LOW);
       break;
     case 3:
       digitalWrite(chipSelectPin3,LOW);
       break;    
  }//end switch
  
}//end func

//*************************************************
void deselectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,HIGH);
        break;
     case 2:
       digitalWrite(chipSelectPin2,HIGH);
       break;
     case 3:
       digitalWrite(chipSelectPin3,HIGH);
       break;    
  }//end switch
  
}//end func



// LS7366 Initialization and configuration
//*************************************************
void LS7366_Init(void)
//*************************************************
{
   
    
    // SPI initialization
    SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);
   
   digitalWrite(chipSelectPin1,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin1,HIGH); 
   
   
   digitalWrite(chipSelectPin2,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin2,HIGH); 
   
   
   digitalWrite(chipSelectPin3,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin3,HIGH); 
   
}//end func

// pack UDP message to the buffer
//*************************************************
void pack_UDP_message(char * SendBuffer, const double DataToSend[SEND_MSG_SIZE])
//*************************************************
{
  // zero out the buffer
  memset(SendBuffer, 0, SEND_MSG_SIZE*sizeof(double));
  for(int i=0; i<SEND_MSG_SIZE; i++)
  {
    memcpy(SendBuffer+i*sizeof(double), DataToSend+i, sizeof(double));
  }
}//end func

