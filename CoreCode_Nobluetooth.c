
  // Arduino Obstacle Avoiding Robot              
 // Code adapted from http://www.educ8s.tv                                         
 // First Include the NewPing and Servo Libraries 


#include <NewPing.h>
#include <Servo.h> 

#define TRIG_PIN A4 
#define ECHO_PIN A5 
#define MAX_DISTANCE 200 
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
Servo myservo;   

boolean goesForward=false;
int distance = 100;
int speedSet = 0;

const int motorPin1  = 11;  
const int motorPin2  = 10;  
//Motor B
const int motorPin3  = 6; 
const int motorPin4  = 5;  
int estado = 'g'; // iniciate state


void setup() {

 Serial.begin(9600); // initiate the serial port fo the Bluetooth connection
 Serial.println("testt");
  myservo.attach(9);  
  myservo.write(115); 
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop() {
  
 int distanceR = 0;
 int distanceL =  0;
 delay(40);
 
 if(Serial.available()>0)
 { // read the bluetooth state and store its value
   estado = Serial.read();
 } 
 if(estado=='w')
 { // Forward
     Serial.println("forward");

     moveForward();
     if(distance<=20)
     {
      moveStop();
      delay(100);
      moveBackward();
      delay(300);
      moveStop();
      delay(200);
      distanceR = lookRight();
      delay(200);
      distanceL = lookLeft();
      delay(200);
    
      if(distanceR>=distanceL)
      {
        turnRight();
        moveStop();
      }else
      {
        turnLeft();
        moveStop();
      }
     }else
     {
      moveForward();
     }
     distance = readPing();
}
if(estado=='d')
{ // right
        turnRight();
//        if(distance<=20)
//       {
//        moveStop();
//        delay(100);
//        moveBackward();
//        delay(300);
//        moveStop();
//        delay(200);
//        distanceR = lookRight();
//        delay(200);
//        distanceL = lookLeft();
//        delay(200);
//      
//        if(distanceR>=distanceL)
//        {
//          turnRight();
//          moveStop();
//        }else
//        {
//          turnLeft();
//          moveStop();
//        }
//       }else
//       {
//        moveForward();
//       }
//       distance = readPing();
      } 
  if(estado=='c')
  { // Stop
      moveStop();   
  }
  if(estado=='a')
  { // left
     turnLeft();
//       if(distance<=20)
//       {
//        moveStop();
//        delay(100);
//        moveBackward();
//        delay(300);
//        moveStop();
//        delay(200);
//        distanceR = lookRight();
//        delay(200);
//        distanceL = lookLeft();
//        delay(200);
//      
//        if(distanceR>=distanceL)
//        {
//          turnRight();
//          moveStop();
//        }else
//        {
//          turnLeft();
//          moveStop();
//        }
//       }else
//       {
//        moveForward();
//       }
//       distance = readPing();
  }
  if(estado=='s')
  { // Reverse
    moveBackward();
  }
  if (estado =='f'){ // Boton ON se mueve sensando distancia 

}
if (estado=='g')
{ // Boton OFF, detiene los motores no hace nada 
}
}





int lookRight()
{
    myservo.write(50); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
}

int lookLeft()
{
    myservo.write(170); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(115); 
    return distance;
    delay(100);
}

int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  return cm;
}

void moveStop() {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, 0);
  } 
  
void moveForward() {

     Serial.println("in forward ");

    analogWrite(motorPin1, 180);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 180);
    analogWrite(motorPin4, 0);  
  
}

void moveBackward() {
    
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 180);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, 180);   
  
}  

void turnRight() {
//  analogWrite(motorPin1, 180);
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 180);
  analogWrite(motorPin4, 0);
//  delay(500);
//  moveForward();      
} 
 
void turnLeft() {
  analogWrite(motorPin1, 180);
  analogWrite(motorPin2, 0);   
  analogWrite(motorPin3, 0);
  //analogWrite(motorPin3, 180);
  analogWrite(motorPin4, 0); 
//  delay(500);
//  moveForward();    
}  
