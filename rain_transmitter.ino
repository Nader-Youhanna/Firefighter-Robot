#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte lightSensorPin = A0; /// pin for light sensor
const byte rainSensorPin = A1; // pin for rain sensor

const byte CEpin = 7;
const byte CSNpin = 8;

const byte address[6] = "00001";

struct SensorsValue {
  float rainSensorValue;   // holds rain value
  float lightSensorValue;   // holds light value
} sensorsValue;

RF24 radio(CEpin, CSNpin); // CE, CSN

==================================================

void setup() {
  // start the serial console
  Serial.begin(9600);
  pinMode(rainSensorPin, INPUT);
  pinMode(lightSensorPin, INPUT);

  setupTransmittNRF();

}

void loop() {
  nrfTransmittData();
}

===================================================

void setupTransmittNRF() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.enableDynamicPayloads();
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void nrfTransmittData() {
  delay(5);
  sensorsValue.rainSensorValue = analogRead(rainSensorPin); //Read data from analog pin and store it to rainSensorValue
  sensorsValue.lightSensorValue = analogRead(lightSensorValue); //Read data from analog pin and store it to rainSensorValue

  bool ok = radio.write(&sensorsValue, sizeof(sensorsValue));
  if (ok) {
    Serial.println("rain value sent!");
  } else {
    Serial.println("rain value failed to send!");
  }
  delay(5);
}
