
/**
  This code is used to read an a nxm array of VL180X time-of-flight sensors (where n = number of multiplexers, m = number of sensors per multiplexers).
  The maximum array size is 8x8.

  Colette Abah
  01/17/18
*/

#include "Wire.h"

extern "C" {
#include "Adafruit_VL6180X.h" //Library from Adafruit. Adapt this library for whatever sensor you are reading
}
/*----------------------------------------------------------------------*/
/*  Initialize a VL6180 X object 8 x 8 array  */
/*----------------------------------------------------------------------*/
Adafruit_VL6180X ToF_Sensors[8][8];
uint8_t tca_addr[8];
uint8_t default_tca_addr = 0x70;

/*----------------------------------------------------------------------*/
/*SET the number of multipexers and sensors per time of flight */
/*----------------------------------------------------------------------*/

int j; int k; //j = multiplexer counter, k=sensor counter
int N_mpx =  1; //Number of multiplexers
int N_sensors = 8; //Number of sensors per multiplexer

/************************************************************************/
/************************************************************************/
/*                              SETUP
  /************************************************************************/
/************************************************************************/

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  delay(1);

  Serial.println("VL6180x Time-of-Flight Sensing!");
  Serial.println("------------------------------------");
  Serial.println("------------------------------------");
  Serial.print("Number of Multiplexers: "); Serial.println(N_mpx);
  Serial.print("Number of Sensors per Multiplexers: "); Serial.println(N_sensors);

  /*----------------------------------------------------------------------*/
  /*Create sensor objects based on the number of multiplexers and sensors*/
  /*----------------------------------------------------------------------*/

  for (j = 0; j < N_mpx; j++) {
    tca_addr[j] = default_tca_addr + j;
    Serial.print("Multiplexer # "); Serial.print(j + 1); Serial.print(" Address: 0X"); Serial.println(tca_addr[j], HEX);
    for (k = 0; k < N_sensors; k++) {
      ToF_Sensors[j][k] = Adafruit_VL6180X(); //j = number of multiplexers, k = number of sensors
    }
  }

  /*----------------------------------------------------------------*/
  /*       Check the ports of each multiplexer for devices        */
  /*---------------------------------------------------------------*/

  for (j = 0; j < N_mpx; j++) {
    for (k = 0; k < N_sensors; k++) {
      tcaselect(k,  tca_addr[j]);
      if (! ToF_Sensors[j][k].begin()) { /* ERROR If no sensors is found on the open port*/
        Serial.print("!!! ERROR !!! No sensor found on port "); Serial.println(k);
        Serial.print("of multiplexer #"); Serial.print(j + 1); Serial.print("(0x"); Serial.print(tca_addr[j], HEX); Serial.println(")");
        Serial.println("------------------------------------");
      }
      else { /* If no sensors is found on the open port*/
        Serial.print("Sensor found on port "); Serial.println(k);
        Serial.print(" of multiplexer #"); Serial.print(j + 1); Serial.print("(0x"); Serial.print(tca_addr[j], HEX); Serial.println(")");
        Serial.println("------------------------------------");
      }
    }
  }
  delay(100);

}

/************************************************************************/
/************************************************************************/
/*                           MAIN CODE
  /************************************************************************/
/************************************************************************/


void loop()
{
  uint8_t range[N_mpx][N_sensors];
  uint8_t status[N_mpx][N_sensors];
  uint8_t cur_sensor_id = 8;
  uint8_t mode=0; //MODE = 0: Output the reading from sensor "cur_sensor_id" This mode is useful for debugging.  MODE 1: Output all the sensor readings

  for (j = 0; j < N_mpx; j++) {
    for (k = 0; k < N_sensors; k++) {
      //float lux = ToF_Sensors[j][k].readLux(VL6180X_ALS_GAIN_5);
      //Serial.print("Lux: "); Serial.println(lux);

      tca_disable (tca_addr[j]); //Turn off all ports
      tcaselect(k,  tca_addr[j]); // Transmit to port k on multiplexer j

      range[j][k] = ToF_Sensors[j][k].readRange();
      status[j][k] = ToF_Sensors[j][k].readRangeStatus();

      if (mode == 0) {
        if (k + 1 == cur_sensor_id) {
          // If NO error, print out the distance from the obstacle!
          if (status[j][k] == VL6180X_ERROR_NONE) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.print(range[j][k]); Serial.println(" mm");

            continue;
          }

          // Some error occurred, print it out!

          if  ((status[j][k] >= VL6180X_ERROR_SYSERR_1) && (status[j][k] <= VL6180X_ERROR_SYSERR_5)) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.println("System error");
          }
          else if (status[j][k] == VL6180X_ERROR_ECEFAIL) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.println("ECE failure");
          }
          else if (status[j][k] == VL6180X_ERROR_NOCONVERGE) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.println("No convergence");
          }
          else if (status[j][k] == VL6180X_ERROR_RANGEIGNORE) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.println("Ignoring range");
          }
          else if (status[j][k] == VL6180X_ERROR_SNR) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.println("Signal/Noise error");
          }
          else if (status[j][k] == VL6180X_ERROR_RAWUFLOW) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.println("Raw reading underflow");
          }
          else if (status[j][k] == VL6180X_ERROR_RAWOFLOW) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.println("Raw reading overflow");
          }
          else if (status[j][k] == VL6180X_ERROR_RANGEUFLOW) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.println("Range reading underflow");
          }
          else if (status[j][k] == VL6180X_ERROR_RANGEOFLOW) {
            Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
            Serial.println("Range reading overflow");
          }

        }
      }
    }

  }
}

/************************************************************************/
/************************************************************************/
/*                           FUNCTIONS
  /************************************************************************/
/************************************************************************/


/*----------------------------------------------------------------------*/
/*Function to select a port on a multiplexer*/
/*----------------------------------------------------------------------*/

void tcaselect(uint8_t i, uint8_t address) {
  if (i > 7) return;
  Wire.beginTransmission(address);
  Wire.write(1 << i);
  Wire.endTransmission();
}
/*----------------------------------------------------------------------*/
/*          Function to disable all the ports on a multiplexer*/
/*----------------------------------------------------------------------*/

void tca_disable(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0);  // no channel selected
  Wire.endTransmission();
}
/*----------------------------------------------------------------------*/
/*          Function to read a VL6180X*/
/*----------------------------------------------------------------------*/
//void Print_VL6180X_Reading(uint8_t mpx_index, uint8_t sensor_index,  uint8_t sensor_status, uint8_t sensor_reading) {
////  if (status[j][k] == VL6180X_ERROR_NONE) {
////    Serial.print("[Mux "); Serial.print(j + 1);  Serial.print(", Sensor "); Serial.print(k + 1); Serial.print("] : ");
////    Serial.print(range[j][k]); Serial.println(" mm");
////
////    continue;
////  }
//
//  if (sensor_status == VL6180X_ERROR_NONE) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.print(sensor_reading); Serial.println(" mm");
//
//    //continue;
//  }
//
//  // Some error occurred, print it out!
//
//  if  ((sensor_status >= VL6180X_ERROR_SYSERR_1) && (sensor_status <= VL6180X_ERROR_SYSERR_5)) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.println("System error");
//  }
//  else if (sensor_status == VL6180X_ERROR_ECEFAIL) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.println("ECE failure");
//  }
//  else if (sensor_status == VL6180X_ERROR_NOCONVERGE) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.println("No convergence");
//  }
//  else if (sensor_status == VL6180X_ERROR_RANGEIGNORE) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.println("Ignoring range");
//  }
//  else if (sensor_status == VL6180X_ERROR_SNR) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.println("Signal/Noise error");
//  }
//  else if (sensor_status == VL6180X_ERROR_RAWUFLOW) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.println("Raw reading underflow");
//  }
//  else if (sensor_status == VL6180X_ERROR_RAWOFLOW) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.println("Raw reading overflow");
//  }
//  else if (sensor_status == VL6180X_ERROR_RANGEUFLOW) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.println("Range reading underflow");
//  }
//  else if (sensor_status == VL6180X_ERROR_RANGEOFLOW) {
//    Serial.print("[Mux "); Serial.print(mpx_index + 1);  Serial.print(", Sensor "); Serial.print(sensor_index + 1); Serial.print("] : ");
//    Serial.println("Range reading overflow");
//  }
//}


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
