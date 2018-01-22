/**
  This code reads multiple VL6180X time of flight sensors using a multiplexer
  Includes a combination of two sample codes from Adafruit:
  - https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test
  - https://learn.adafruit.com/adafruit-vl6180x-time-of-flight-micro-lidar-distance-sensor-breakout/wiring-and-test
  Colette Abah
  01/17/18
*/

#include "Wire.h"

extern "C" {
#include "Adafruit_VL6180X.h" //Library from Adafruit. Adapt this library for whatever sensor you are reading
}

Adafruit_VL6180X ToF_Sensors[8][8];
uint8_t tca_addr[8];
uint8_t default_tca_addr = 0x70;

/*==========Initialize functions=============*/

/*Function to select a port on a multiplexer*/
void tcaselect(uint8_t i, uint8_t address) {
  if (i > 7) return;
  Wire.beginTransmission(address);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void tca_disable(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0);  // no channel selected
  Wire.endTransmission();
}



void setup()
{
  Serial.begin(115200);
  while (!Serial);
  delay(1);

  Serial.println("VL6180x Time-of-Flight Sensing!");
  Serial.println("------------------------------------");
  Serial.println("------------------------------------");

  //scan_i2c();

  int j; int k;
  int N_mpx =  2; //Number of multiplexers
  int N_sensors = 4; //Number of sensors per multiplexer

  /*Create sensor objects based on the number of multiplexers and sensors*/

  for (j = 0; j < N_mpx; j++) {
    tca_addr[j] = default_tca_addr + j;
    //Serial.println(tca_addr[j],HEX);
    for (k = 0; k < N_sensors; k++) {
      ToF_Sensors[j][k] = Adafruit_VL6180X(); //j = number of multiplexers, k = number of sensors
    }
  }

  /*Establish communication with ports of the multiplexers*/
  for (j = 0; j < N_mpx; j++) {
    for (k = 0; k < N_sensors; k++) {
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
}

void loop()
{
  int j; int k;
  int N_mpx =  2; //Number of multiplexers
  int N_sensors = 4; //Number of sensors per multiplexer
  uint8_t range;
  uint8_t status;

  for (j = 0; j < N_mpx; j++) {
    for (k = 0; k < N_sensors; k++) {
      //float lux = ToF_Sensors[j][k].readLux(VL6180X_ALS_GAIN_5);
      //Serial.print("Lux: "); Serial.println(lux);
      tca_disable (tca_addr[j]); //Turn off all ports
      tcaselect(k,  tca_addr[j]); // Transmit to port k on multiplexer j

      range = ToF_Sensors[j][k].readRange();
      status = ToF_Sensors[j][k].readRangeStatus();
      //Serial.println(status)
      //Serial.println(loop_count);

      if (status == VL6180X_ERROR_NONE) {
        Serial.print("[Multiplexer "); Serial.print(j + 1);  Serial.print(",Port "); Serial.print(k); Serial.print("] : ");
        Serial.print(range); Serial.println(" mm");
        continue;
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

    }

  }
}

/*Function to scan the I2C bus*/
//void scan_i2c()
//{
//  byte error, address;
//  int nDevices;
//
//  Serial.println("Scanning...");
//
//  nDevices = 0;
//  for (address = 1; address < 127; address++ )
//  {
//    // The i2c_scanner uses the return value of
//    // the Write.endTransmisstion to see if
//    // a device did acknowledge to the address.
//
//    Wire.beginTransmission(address);
//    error = Wire.endTransmission();
//
//    if (error == 0)
//    {
//      Serial.print("I2C device found at address 0x");
//      if (address < 16)
//        Serial.print("0");
//      Serial.print(address, HEX);
//      Serial.println("  !");
//
//      nDevices++;
//    }
//    else if (error == 4)
//    {
//      Serial.print("Unknown error at address 0x");
//      if (address < 16)
//        Serial.print("0");
//      Serial.println(address, HEX);
//    }
//  }
//  if (nDevices == 0)
//    Serial.println("No I2C devices found\n");
//  else
//  {
//    Serial.print("Number of devices on the I2C bus: ");
//    Serial.print(nDevices);
//    Serial.println("  !");
//  }
//}
