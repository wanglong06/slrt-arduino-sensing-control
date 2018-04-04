#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

//I2C address of the TCA9548 multiplexer (Adafruit)
#define TCAADDR 0x70

//I2C device addresses; will be auto-populated after scanning in setup()
//0 if no device is connected
uint8_t I2C_addr[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//12-bit = 4095 counts, Vcc = 5 V
size_t counts_per_volt = 4095/5;

//function to select a port on the multiplexer
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  while (!Serial);
    delay(1000);

    Wire.begin();
    Serial.begin(115200);

    //scan for I2C devices

    //for each of the 8 ports on the multiplexer
    for(uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t+1);

      //scan all possible I2C address values
      for(uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
        
        uint8_t data;
        if(! twi_writeTo(addr, &data, 0, 1, 1) && addr != 0) {
           Serial.print("Found I2C 0x");
           Serial.println(addr,HEX);
           I2C_addr[t] = addr;
        }
      }
      Serial.println();
    }
}

void loop() {
    tcaselect(1);                           //select port 2 on the multiplexer
    Wire.beginTransmission(I2C_addr[1]);    //begin I2C comms with the DAC on port 2
    Wire.write(64);                         //command to update the DAC
    //set DAC output to 1 V
    Wire.write(1*counts_per_volt >> 4);         //8 most significant bits
    Wire.write((1*counts_per_volt & 15) << 4);  //4 least significant bits
    Wire.endTransmission();                     //end I2C comms with the DAC on port 2

    tcaselect(2);                           //select port 3 on the multiplexer
    Wire.beginTransmission(I2C_addr[2]);    //begin I2C comms with the DAC on port 3
    Wire.write(64);                         //command to update the DAC
    //set DAC output to 2 V
    Wire.write(2*counts_per_volt >> 4);         //8 most significant bits
    Wire.write((2*counts_per_volt & 15) << 4);  //4 least significant bits
    Wire.endTransmission();                     //end I2C comms with the DAC on port 3

    tcaselect(3);                           //select port 4 on the multiplexer
    Wire.beginTransmission(I2C_addr[3]);    //begin I2C comms with the DAC on port 4
    Wire.write(64);                         //command to update the DAC
    //set DAC output to 3 V
    Wire.write(3*counts_per_volt >> 4);         //8 most significant bits
    Wire.write((3*counts_per_volt & 15) << 4);  //4 least significant bits
    Wire.endTransmission();                     //end I2C comms with the DAC on port 4
}
