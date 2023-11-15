#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define PloadSize  32  // 32 unsigned chars TX payload

// nrf pins---------------------
const byte CEpin = 7;
const byte CSNpin = 8;
--------------------------------

// motor driver pins------------
const byte motorIN1pin = 0; //
const byte motorIN2pin = 1; //
const byte motorEnApin = 3; // enable pin for motor
--------------------------------

// light pins-------------------
const byte lightOutputPin = 13; // pin connected to lights
--------------------------------

// NRF variables----------------
byte CurrentPloadWidth;
const byte address[6] = "00001";
unsigned char rx_buf[PloadSize] = {0};
--------------------------------

// sensor values received from nrf
struct sensorsValue {
  float rainSensorValue;   // holds rain value
  float lightSensorValue;   // holds light value
} sensorsValue;
--------------------------------

RF24 radio(CEpin, CSNpin); // CE, CSN


=========================================================

void setup() {
  // start the serial console
  Serial.begin(9600);
  
  setupReceiveNRF();
  setupMotorDriver();
  setupLights();

}

void loop() {
  nrfReceiveData();
}


=========================================================

void setupReceiveNRF() {
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.enableDynamicPayloads();
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void nrfReceiveData() {
  if (radio.available()) {
    CurrentPloadWidth = radio.getDynamicPayloadSize();

    // If a corrupt dynamic payload is received, it will be flushed
    if (!CurrentPloadWidth) {}
    else {
      radio.read(rx_buf, CurrentPloadWidth);
      newdata = 1;
    }
    if (newdata == 1) {
      newdata = 0;
      memcpy(&sensorsValue, rx_buf, sizeof(sensorsValue));

      Serial.println("Data from Sensor 1: ");
      Serial.println(sensorsValue.rainSensorValue);
      Serial.println(sensorsValue.lightSensorValue);
    }
  }
}


----------------------
// do all the required setup for lights
void setupLights() {
  pinMode(lightOutputPin, OUTPUT);
}

----------------------
// do all the required setup for the motor
void setupMotorDriver() {
  pinMode(motorIN1pin, OUTPUT);
  pinMode(motorIN2pin, OUTPUT);
  pinMode(motorEnApin, OUTPUT);
}
// function to step the motor
void stopMotor() {
  digitalWrite(motorIN1pin, LOW);
  digitalWrite(motorIN2pin, LOW);
  analogWrite(motorEnApin, 0);
}
// function to start the motor in forward direction with the given speed
// speed must be a value between 0 and 255
void startMotor(int speed) {
  digitalWrite(motorIN1pin, HIGH);
  digitalWrite(motorIN2pin, LOW);

  if (speed < 0) {
    analogWrite(motorEnApin, 0);
  } else if (speed > 255) {
    analogWrite(motorEnApin, 255);
  } else {
    analogWrite(motorEnApin, speed);
  }
}
