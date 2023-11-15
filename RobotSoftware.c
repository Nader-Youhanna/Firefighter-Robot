#include <VoiceRecognitionV3.h>

//---------------------------RAIN----------------------------------------
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

byte lightSensorPin = A0; /// pin for light sensor
byte rainSensorPin = A1;  // pin for rain sensor

const byte CEpin = 22;
const byte CSNpin = 23;

const byte address[6] = "00001";

struct SensorsValue {
  float rainSensorValue;
  float lightSensorValue;
} sensorsValue;

RF24 radio(CEpin, CSNpin); // CE, CSN

//---------------------------GRASS CUTTING----------------------------------------

//const int grassMotorPin1 = 8; // if 180 and pin2 = 0  forward the right motor
const int grassMotorPin2 = 30; // if 180 and pin1 = 0 backward the right motor
bool workingGrassCutting = false;

//---------------------------VOICE MODULE----------------------------------------
#include <SoftwareSerial.h>

uint8_t records[7]; // save record
uint8_t buf[64];
VR myVR(10, 11);
#define f 'w'
#define r 'd'
#define s 'c'
#define l 'a'
#define b 's'
#define p 'p'
#define g 'g'
#define e 'e'
#define f1 21
#define r1 22
#define s1 23
#define l1 24
#define b1 25
#define p1 26
#define g1 27


//---------------------------MOVEMENT----------------------------------------
#include <NewPing.h>
#include <Servo.h>
#define TRIG_PIN A4
#define ECHO_PIN A5
#define MAX_DISTANCE 200

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo myservo;
Servo servoFlame;

int distance = 100;
int speedSet = 0;

//Motor A
const int motorPin1 = 9; // if 180 and pin2 = 0  forward the right motor
const int motorPin2 = 8;// if 180 and pin1 = 0 backward the right motor

//Motor B
const int motorPin3 = 6; // if 180 and pin4 = 0  forward the left motor
const int motorPin4 = 5; // if 180 and pin3 = 0  forward the left motor
int state;

int distanceR = 0;
int distanceL = 0;

float currentMillis = 0;
float previousMillis = 0;
//------------------------FIRE DETECTION------------------
//Pins of flame sensor
const int rightFlame = A10;
const int leftFlame = A12;
const int forwardFlame = A11;

int rightFire;
int leftFire;
int forwardFire;

int fireDistance;

int indicator = 15; // to indicate which flame sensor is reading the highest temp
int standardDistance = 50;  // distance to set off the fire

//Pin for relayFlame
const int relayFlamePin = 14;

//Pins for servo


//---------------------PAINTING----------------------------------

// array of the directions the robot will follow
// 0--->forward, 1--> Right , 2-->Left , 3-->circle with radius
const int arrDirs[19] = {0, 2, 2, 2, 2, 2, 0, 2, 2, 3, 2, 1, 1, 1, 1, 1, 1, 0, 1};

// array of length of the lines the robot will paint in meter
const float arrLength[19] = {5, 2, 0.5, 1, 0.5, 1, 1, 2.5, 2, 0.5, 1, 2.5, 2, 0.5, 1, 0.5, 1, 1, 2.5};

// array that indicates if it will paint or not
//const bool arrBoolean[19] = {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1};

// array of the time required in each in milliseconds
float arrTime[19];

int currentTime = 0;
int startedLast = 0;
const float robotSpeed = 0.0005; // in m/msec
int k = 0;
bool paint = false;
bool justStartedPainting = true;

const int relayPin = 7;
bool switchedDirection = false;

int startedTurningTime; // in milli
const int turningTime = 1000; // in milli

//-------------------------------------------------------------------

void setup()
{
  pinMode(relayPin, OUTPUT);
  pinMode(relayFlamePin, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  //  pinMode(grassMotorPin1, OUTPUT);
  pinMode(grassMotorPin2, OUTPUT);
  pinMode(rainSensorPin, INPUT);
  pinMode(lightSensorPin, INPUT);

  setupTransmittNRF();

  for (int i = 0; i < 19; i++)
  {
    arrTime[i] = (arrLength[i] / robotSpeed);
    if (arrDirs[i] == 1 && arrDirs[i] == 2)
      arrTime[i] += turningTime;
  }
  // Serial.begin(9600); // initiate the serial port fo the Bluetooth connection
  myservo.attach(12);
  servoFlame.attach(17);
  servoFlame.write(115);
  myservo.write(115);
  distance = readPing();
  Serial.println("HEY");
  myVR.begin(9600);
  Serial.begin(9600);
  if (myVR.clear() == 0)
  {
    Serial.println("Recognizer cleared.");
  }
  else
  {
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    //while (1)
    //    ;
  }
  if (myVR.load((uint8_t)f1) >= 0)// on record should be declared
  {
    Serial.println("forward loaded");
  }

  if (myVR.load((uint8_t)r1) >= 0)
  {
    Serial.println("right loaded");
  }
  if (myVR.load((uint8_t)b1) >= 0)
  {
    Serial.println("reverse loaded");
  }
  if (myVR.load((uint8_t)l1) >= 0)
  {
    Serial.println("left loaded");
  }
  if (myVR.load((uint8_t)p1) >= 0)
  {
    Serial.println("painting loaded");
  }
  if (myVR.load((uint8_t)g1) >= 0)
  {
    Serial.println("grass cutting loaded");
  }

  attachInterrupt(digitalPinToInterrupt(19), flameDetector, RISING);
  attachInterrupt(digitalPinToInterrupt(20), flameDetector, RISING);
  attachInterrupt(digitalPinToInterrupt(21), flameDetector, RISING);

}

void loop()
{
  //    analogWrite(grassMotorPin1, 180);
  //   analogWrite(grassMotorPin2,180);
  nrfTransmittData(); // for rain
  distanceR = 0;
  distanceL = 0;
  int ret = myVR.recognize(buf, 50);
  if (ret >= 0) {
    for (int i = 0; i < 64; i++) {
      Serial.print(buf[i]);
      Serial.print(' ');
    }
    Serial.println(' ');
  }
  if (Serial.available() > 0) // if the bluetooth did not work remove this if condition
  { // read the bluetooth state and store its value
    Serial.println("READING");
    state = Serial.read();
    Serial.println(static_cast<char>(state));
  }
  if (state == f || (ret > 0 && buf[1] == f1))
  { // Forward
    Serial.println("state is f");
    moveForward();
    // obstacleAvoidance();
  }
  if (state == r || (ret > 0 && buf[1] == r1))
  { // Right
    turnRight();
    //  obstacleAvoidance();
  }
  if (state == s || (ret > 0 && buf[1] == s1))
  { // Stop
    moveStop();
  }
  if (state == l || (ret > 0 && buf[1] == l1))
  { // Left
    turnLeft();
    // obstacleAvoidance();
  }
  if (state == b || (ret > 0 && buf[1] == b1))
  { // Reverse
    moveBackward();
  }
  if (paint || (state == p || (ret > 0 && buf[1] == p1)))
  { //Painting
    paint = true;
    painting();
  }
  if (state == g || (ret > 0 && buf[1] == g1)) // toggle on click
  { // start Grass cutting
    grassCutting();
  }
  if (state == e) // fire
  {
    flameFighter();
  }
  //  else
  //  {
  //    digitalWrite(relayFlamePin, LOW);
  //  }
}

int lookRight()
{
  myservo.write(50);
  delayfunction(500);
  int distance = readPing();
  delayfunction(100);
  myservo.write(115);
  return distance;
}

int lookLeft()
{
  myservo.write(170);
  delayfunction(500);
  int distance = readPing();
  delayfunction(100);
  myservo.write(115);
  return distance;
}

int readPing()
{
  delayfunction(70);
  int cm = sonar.ping_cm();
  if (cm == 0)
  {
    cm = 250;
  }
  return cm;
}

void moveStop()
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}

void moveForward()
{
  Serial.println("in forward");
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 180);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 180);
}

void moveBackward()
{

  analogWrite(motorPin1, 180);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 180);
  analogWrite(motorPin4, 0);
}

void turnRight()
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 180);
}

void turnLeft()
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 180);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}

void obstacleAvoidance()
{
  if (distance <= 20)
  {
    moveStop();
    delayfunction(100);
    moveBackward();
    delayfunction(300);
    moveStop();
    delayfunction(200);
    distanceR = lookRight();
    distanceL = lookLeft();
    delayfunction(200);

    if (distanceR >= distanceL)
    {
      turnRight();
      moveStop();
    }
    else
    {
      turnLeft();
      moveStop();
    }
  }
  else
  {
    moveForward();
  }
  distance = readPing();
}

void delayfunction(float time)
{
  previousMillis = millis();
  currentMillis = millis();
  while (currentMillis - previousMillis < time) //delay time
  {
    currentMillis = millis();
    state = Serial.read();
  }
}

void moveCircle()
{
  moveForward();
  turnLeft();
}

void painting()
{

  currentTime = millis();
  if (justStartedPainting)
  {
    justStartedPainting = false;
    digitalWrite(relayPin, HIGH);
    delayfunction(3000); // Wait untill the paitning pump is activated
    startedLast = millis();
  }
  //  if (arrBoolean[k])
  //    digitalWrite(relayPin, HIGH);
  //  else
  //    digitalWrite(relayPin, LOW);

  if (!(arrDirs[k] == 3))
    moveForward();
  else
    moveCircle();

  if (!switchedDirection)
  {
    startedTurningTime = millis();
    switchedDirection = true;
    switch (arrDirs[k])
    {
      case 1: // move right
        while (turningTime > millis() - startedTurningTime)
        {
          turnRight();
        }
        break;
      case 2: // move left
        while (turningTime > millis() - startedTurningTime)
        {
          turnLeft();
        }
        break;
    }
  }
  if ((currentTime - startedLast) >= arrTime[k])
  {
    k++;
    startedLast = millis();
    switchedDirection = false;
  }
  if (k == 19)
  {
    justStartedPainting = true;
    paint = false;
    k = 0;
    switchedDirection = false;
    digitalWrite(relayPin, LOW);
  }
}

void grassCutting()
{
  if (!workingGrassCutting)
  {
    workingGrassCutting = true;
    //    analogWrite(grassMotorPin1, 180);
    analogWrite(grassMotorPin2, 180);
  }
  else
  {
    workingGrassCutting = false;
    //    analogWrite(grassMotorPin1, 0);
    analogWrite(grassMotorPin2, 0);
  }
};

void setupTransmittNRF()
{
  radio.begin();
  radio.openWritingPipe(address);
  radio.enableDynamicPayloads();
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void nrfTransmittData()
{
  delay(5);
  sensorsValue.rainSensorValue = analogRead(rainSensorPin); //Read data from analog pin and store it to rainSensorValue
  sensorsValue.lightSensorValue = analogRead(lightSensorPin); //Read data from analog pin and store it to rainSensorValue

  bool ok = radio.write(&sensorsValue, sizeof(sensorsValue));
  if (ok) {
    Serial.println("values sent!");
  } else {
    //Serial.println("values failed to send!");
  }
  delay(5);
}



void flameDetector()
{
  rightFire = analogRead(rightFlame);
  leftFire = analogRead(leftFlame);
  forwardFire = analogRead(forwardFlame);
  
  Serial.println(rightFire);
  Serial.println(leftFire);
  Serial.println(forwardFire);
  
  if (!(rightFire > 100 || leftFire > 100 || forwardFire > 100))
    return;

  // if painting command then stop to fight fire
  paint = false;
  state = e;
  buf[1] = e ;


  

  int turningLoop = 15; // to be corrected after trial
  int maxDistance = 100;
  int minDistance = 20;

  int maximum = 0;

  if (rightFire > leftFire)
  {
    if (rightFire > forwardFire)
    {
      maximum = rightFire;
      indicator = 1;

    }
    else
    {
      maximum = forwardFire;
      indicator = 2;
    }
  }
  else
  {
    if (leftFire > forwardFire)
    {
      maximum = leftFire;
      indicator = 3;
    }
    else
    {
      maximum = forwardFire;
      indicator = 2;
    }
  }
  // maximum = max(leftFire, rightFire);
  // maximum = max(maximum,forwardFire);

  fireDistance = map (maximum, 0, 1023, maxDistance, minDistance);

}

void flameFighter()
{
  int maxDistance = 100;
  int minDistance = 20;
    int maximum = 0;

  if (rightFire > leftFire)
  {
    if (rightFire > forwardFire)
    {
      maximum = rightFire;
      indicator = 1;

    }
    else
    {
      maximum = forwardFire;
      indicator = 2;
    }
  }
  else
  {
    if (leftFire > forwardFire)
    {
      maximum = leftFire;
      indicator = 3;
    }
    else
    {
      maximum = forwardFire;
      indicator = 2;
    }
  }
  fireDistance = map (maximum, 0, 1023, maxDistance, minDistance);
  Serial.println("fire");
  Serial.println(fireDistance);
  
  if (fireDistance > standardDistance + 10)
  {
    Serial.println("moving to fire");
    switch (indicator)
    {
      case 1: // move right
        turnRight();
        break;
      case 2: // move forward
        Serial.println("farward");
        moveForward();
        break;
      case 3: // move left

        turnLeft();
        break;
    }

  }
  else if (fireDistance < standardDistance - 10)
  {
    
    Serial.println("moving away from fire");
    moveBackward();
  }
  else  // equal to standardDistance
  {
    Serial.println("opening relay");
    switch (indicator)
    {
      case 1: // move right
        servoFlame.write(30);
        break;
      case 2: // move forward

        servoFlame.write(90);
        break;
      case 3: // move left

        servoFlame.write(120);
        break;
    }
    unsigned long startFighter = millis();
    digitalWrite(relayFlamePin, HIGH); // open relay
    while (millis() < startFighter + 3000)
    {}
    digitalWrite(relayFlamePin, LOW);
    state = g;
  }


}
