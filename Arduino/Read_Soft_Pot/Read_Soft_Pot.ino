// Code to read the output voltage of the Soft Potentioneter and the corresponding 
// Circumferential position of contact. 
// For contacts closest to to the pins, the serial monitor should read 5V (and 360 deg) and 
// contacts furthest from the pins, the serial monitor should read 0V (and 0 deg)

// Colette Abah 
// 1/4/18

int sensorPin = A0; //Input pin for potentiometer
int ledPin = 13; //LED pin
//int sensorVal = 0; //Variable to store the sensor value

void setup() {
  //start serial connection
  Serial.begin(9600);

  // Initialize the pins
  // pinMode(sensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  //Read the value on the collector Pin
  float sensorVal = analogRead(sensorPin) * 5.0 / 1024; // [volts]
  float contactLocation = analogRead(sensorPin) * 360.0 / 1024; //[Deg]

  //print out the value of the pushbutton
  //Serial.print("Voltage: "); Serial.println(sensorVal);
  Serial.print("Angle: "); Serial.println(contactLocation);
}
