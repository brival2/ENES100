/* This script is autonomous drive code for an over-sand vehicle constructed for the purposes of
 * the ENES100 class at the University of Maryland. The over-sand Vehicle made use of an Arduino
 * Romeo Board, RF communications, and Servo libraries. ENES100 is a team-based Engineering Design
 * Class, so this code was constructed as a collaboration with group members
 */

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "enes100.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#define trigPinU A5
#define echoPinU A4
#define trigPin A2
#define echoPin A1
Servo Servo1;


SoftwareSerial mySerial(8, 9);
Marker marker(10);
RF_Comm rf(&mySerial, &marker);
float starting;
float bHeight;
float bLength;


//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

/*
 * This function causes the vehicle to stop by setting the two sides to zero
 */
void stop(void)                    //Stop
{
  digitalWrite(E1,LOW);           //  Set the Speed of the E1 to 0
  digitalWrite(E2,LOW);           // Set the speed of E2 to 0
}   

/* This function causes the vehicle to move forward at a certain speed
 *  The parameters a and b are the desired speed of the left and right motors
 *  Setting M1 to M2 to HIGH sets both motors to move forward
 */
void advance(int a,int b)          
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);   // set direction 
  analogWrite (E2,b);      // PWM Speed Control
  digitalWrite(M2,HIGH);   // set direction
}  


/*
 * This function causes the vehicle to move backward at a given speed
 * The parameters a and b are the desired speed of the left and right motors
 * Setting M1 and M2 to LOW sets both motors to move backwards
 */
void back_off (int a,int b)          //Move backward
{
  analogWrite (E1,a);       //PWM Speed Control
  digitalWrite(M1,LOW);     // set direction
  analogWrite (E2,b);       // PWM Speed Control
  digitalWrite(M2,LOW);     // set direction
}


/*
 * This function causes the vehicle to turn right at a given speed
 * The parameters a and b are the desired speed of the left and right motors.
 * Setting M1 to LOW and M2 to high causes the vehicle to rotate clockwise
 */
void turn_R (int a,int b)             //Turn Right
{
  analogWrite (E1,a);     //PWM Speed Control
  digitalWrite(M1,LOW);   // set direction  
  analogWrite (E2,b);     //PWM speed Control
  digitalWrite(M2,HIGH);  // set direction
}

/*
 * This function causes the vehicle to turn right at a given speed
 * The parameters a and b are the desired speed of the left and right motors.
 * Setting M2 to LOW and M1 to HIGH causes the vehicle to rotate counter
 */
void turn_L (int a,int b)             //Turn Left
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);   // set direction
  analogWrite (E2,b);      // PWM Speed Control
  digitalWrite(M2,LOW);    // set direction
}

/*
 * This function configures all the pins for the apparatuses on the oversand vehicle
 */
void setup()
{
  /* Initialize serial output */
  mySerial.begin(9600);
  Serial.begin(9600);
  pinMode(A3, OUTPUT); // trigger up for rangefinder
  pinMode(A0, INPUT); // echo down
  pinMode(trigPin, OUTPUT); //trigger len
  pinMode(echoPin, INPUT); //echo len
  pinMode(trigPinU,OUTPUT);
  pinMode(echoPinU,INPUT);
  int i;
  for(i=4;i<=7;i++)
    pinMode(i, OUTPUT);   
}
/*
 * This function turns the OSV to a desired theta value, based on feedback from the RF communications
 * It calls functions defined earlier
 */
void turnToTheta(double desiredTheta){

  /* Execution does not continue until a location is returned */
  while(!rf.updateLocation());
  
  //convert theta values to 0-2pi scale, because the values are -pi to pi
  desiredTheta = desiredTheta+PI;
  double theta = marker.theta+PI;
  //compute the initial error
  double err = desiredTheta-theta;
  delay(500);


  /* If the desired angle is greater
   *  than the current angle, turn left
   */
  if(desiredTheta>theta){
    while(err>0.05){
      turn_L(255,255);
      if(err<0.2){
        delay(500);
      }
      else{
        delay(1000);
      }
      stop();
      /* wait for location to update again */
      while(!rf.updateLocation());
      theta = marker.theta + PI;
      err = desiredTheta-theta;
      delay(200);
    }
  }

  /* If the desired angle is less than the current angle, turn right */
  else{
    while(err<-0.05){
      turn_R(255,255);
      if(err>-0.2){
        delay(500);
      }
      else{
        delay(1000);
      }
      stop();
      /* wait for location to update again */
      while(!rf.updateLocation());
      theta = marker.theta + PI;
      err = desiredTheta-theta;
      delay(200);
    }
  }
  //Final Error
  while(!rf.updateLocation());
  theta = marker.theta+PI;
  err = desiredTheta-theta;
 } 
  

/*
 * This function handles the case where the OSV
 * is above the starting Y position, so it rotates down
 */
void aboveStartingY() {
  rf.println("rotate to theta = 0");
  turnToTheta(0);
  delay(100);
}

/* This function serves as a continuation
 *  of aboveStartingY()
 */
void aboveStartingY2() {
  rf.println("turn to theta = -PI/2");
  turnToTheta(-PI/2);
  rf.println("drive to y = .44");
  moveToY(.44);
  rf.println("rotate to theta = 0");
  turnToTheta(0);
  delay(100);
}

/*
 * The function handles the case where the OSV is above
 * above the desired starting Y position, so the OSV rotates downwards
 */
void belowStartingY() {
  rf.println("rotate to theta = 0");
  turnToTheta(0);
  delay(100);
  }

/*
 * This function serves as a continuation of belowStartingY()
 */
void belowStartingY2() {
  rf.println("turn to theta = PI/2");
  turnToTheta(PI/2);
  rf.println("drive to y = 1.56");
  moveToY(1.56);
  rf.println("rotate to theta = 0");
  turnToTheta(0);
  delay(100);
  }


/*
 * The function iks for when the OSV is left of the starting x
 */
void leftStartingX() {
  rf.println("leftstartingx");
  moveToX(0.5);
  delay(100);
  }

/*
 * 
 */
void rightStartingX() {
  rf.println("rightstartingx");
  moveToX(0.5);
  delay(100);
  }

// the following should check if the wall is there or not, if the wall is on top, return true
// if the wall is on bottom, return false
boolean checkWall() {
  rf.println("checking wall");
  delay(50);
  digitalWrite(A3, LOW); 
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);
  delayMicroseconds(10);
  digitalWrite(A3,LOW);
  float duration = pulseIn(A0, HIGH);
  float distance = (duration / 2) * 0.0344;
  if(marker.y>1){
  if(distance <= 50){
    rf.println("There is a wall");
    rf.println("distance:");
    rf.println(distance);
    return true;
  }
  else{
    rf.println("There is not a wall");
    rf.println("distance:");
    rf.println(distance);
    return false;
    };
  }
  else{
    if(distance <= 50){
    rf.println("There is a wall");
    rf.println("distance:");
    rf.println(distance);
    return false;
  }
  else{
    rf.println("There is not a wall");
    rf.println("distance:");
    rf.println(distance);
    return true;
    };
  }
  }

// the following should move forward, rotate right, then move forward to the checkpoint, then rotate left
void wallOnTop() {
  double down = (-PI/2);
  rf.println("Turn to theta = -PI/2");
  turnToTheta(down);
  rf.println("Drive to y = 0.4");
  moveToY(0.4);
  rf.println("Turn to theta = 0");
  turnToTheta(0);
  rf.println("drive to x = 1.5");
  moveToX(1.5);
  rf.println("turn to theta = PI/2");
  turnToTheta(PI/2);
  rf.println("move forward to y = 1");
  moveToY(1);
  rf.println("turn to theta = 0");
  turnToTheta(0);
  rf.println("drive to x=2.25");
  moveToX(2.25);
  rf.println("turn to theta = -PI/4");
  turnToTheta(-PI/4);
  rf.println("move to y=.84");
  movePast(.84);
  rf.println("turn to theta = 0");
  turnToTheta(0);
  delay(100);
  }

void wallOnTop2() {
  rf.println("Go forward to x = 1.4");
  moveToX(1.2);
  rf.println("Turn to theta = PI/2");
  turnToTheta(PI/2);
  rf.println("Drive to y = 1");
  moveToY(1);
  rf.println("turn to theta = 0");
  turnToTheta(0);
  rf.println("drive to x = 2.25");
  moveToX(2.25);
  rf.println("turn to theta = -PI/4");
  turnToTheta(-PI/4);
  rf.println("move to y = .84");
  movePast(.84);
  rf.println("turn to theta = 0");
  turnToTheta(0);
  delay(100);
  }
// the following should rotate right to an appropiate y value, rotate left, move forward, and then rotate left,
// move forward to the check point, then rotate left
void wallOnBottom() {
  rf.println("Go forward to x = 1.4");
  moveToX(1.2);
  rf.println("Turn to theta = -PI/2");
  turnToTheta(-PI/2);
  rf.println("Drive to y = 1.2");
  moveToY(1.2);
  rf.println("turn to theta = 0");
  turnToTheta(0);
  rf.println("drive to x = 2.25");
  moveToX(2.25);
  rf.println("turn to theta = -PI/4");
  turnToTheta(-PI/4);
  rf.println("move to y = .84");
  movePast(.84);
  rf.println("turn to theta = 0");
  turnToTheta(0);
  delay(100);
  }

void wallOnBottom2() {
  rf.println("Turn to theta = PI/2");
  turnToTheta(PI/2);
  rf.println("Drive to y = 1.56");
  moveToY(1.56);
  rf.println("Turn to theta = 0");
  turnToTheta(0);
  rf.println("Go forward to x = 1.4");
  moveToX(1.2);
  rf.println("Turn to theta = -PI/2");
  turnToTheta(-PI/2);
  rf.println("Drive to y = 1.2");
  moveToY(1.2);
  rf.println("turn to theta = 0");
  turnToTheta(0);
  rf.println("drive to x = 2.25");
  moveToX(2.25);
  rf.println("turn to theta = -PI/4");
  turnToTheta(-PI/4);
  rf.println("move to y = .84");
  movePast(.84);
  rf.println("turn to theta = 0");
  turnToTheta(0);
  delay(100);
  }

void moveToX(double targetX){
  while(!rf.updateLocation());
  if(marker.x > targetX){
    back_off(255,255);
    delay(200);
    while(!rf.updateLocation());
    }
  else if (marker.x < targetX){
    while(marker.x < targetX){
      advance(255,255);
      delay(200);
      while(!rf.updateLocation());
      rf.updateLocation();
      if(marker.theta > -.15 || marker.theta < .15)
      turnToTheta(0);
      }
  }
    stop();
  }

  void movePast(double targetY){
  while(!rf.updateLocation());
  if(marker.y > targetY){
      while(marker.y > targetY){
        if(marker.theta > 0){
          back_off(255,255);
        }
        else{
        advance(255,255);
        }
        while(!rf.updateLocation());
        rf.println("y is ");
        rf.print(marker.y);
        
      }
      delay(200);
      }
  else if (marker.y < targetY){
    while(marker.y < targetY){
      if(marker.theta < 0){
        back_off(255,255);
      }
      else{
        advance(255,255);
      }
      delay(200);
      while(!rf.updateLocation());
      rf.println("y is ");
      rf.print(marker.y);
      
      }
  }
    stop();
  }
  
void moveToY(double targetY){
  while(!rf.updateLocation());
  if(marker.y > targetY){
      while(marker.y > targetY){
        if(marker.theta > 0){
          back_off(255,255);
        }
        else{
        advance(255,255);
        }
        while(!rf.updateLocation());
        rf.println("y is ");
        rf.print(marker.y);
        if(marker.theta > -1.4 || marker.theta < -1.7)
        turnToTheta(-PI/2);
      }
      delay(200);
      }
  else if (marker.y < targetY){
    while(marker.y < targetY){
      if(marker.theta < 0){
        back_off(255,255);
      }
      else{
        advance(255,255);
      }
      delay(200);
      while(!rf.updateLocation());
      rf.println("y is ");
      rf.print(marker.y);
      if(marker.theta > 1.4 || marker.theta < 1.7)
      turnToTheta(PI/2);
      }
  }
    stop();
  }

// the following should move the osv to the mission location from a pretermined starting location
void moveToMissionLocation() {
  rf.println("Drive till X=2.9");
  moveToX(3.1);
  
  rf.println("Go forward to terrain");
  digitalWrite(A3, LOW); 
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);
  delayMicroseconds(10);
  digitalWrite(A3,LOW);
  float duration = pulseIn(A0, HIGH);
  float distance = (duration / 2) * 0.0344;
  rf.println(distance);
  delay(500);
  while(distance>6){
    advance(255,255);
    delay(500);
    stop();
    digitalWrite(A3, LOW); 
    delayMicroseconds(2);
    digitalWrite(A3, HIGH);
    delayMicroseconds(10);
    digitalWrite(A3,LOW);
    duration = pulseIn(A0, HIGH);
    distance = (duration / 2) * 0.0344;
  }
  rf.println("Arrived at Terrain");
  rf.println(distance);
  }

 void backUp(){
  rf.println("Back up to 15 cm away");
  digitalWrite(A3, LOW); 
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);
  delayMicroseconds(10);
  digitalWrite(A3,LOW);
  float duration = pulseIn(A0, HIGH);
  float distance = (duration / 2) * 0.0344;
  rf.println(distance);
  delay(500);
  while(distance<10){
    back_off(255,255);
    delay(500);
    stop();
    digitalWrite(A3, LOW); 
    delayMicroseconds(2);
    digitalWrite(A3, HIGH);
    delayMicroseconds(10);
    digitalWrite(A3,LOW);
    duration = pulseIn(A0, HIGH);
    distance = (duration / 2) * 0.0344;
  }
  rf.println("Arrived");
  rf.println(distance);
 }

 void measureLength(){
  Servo1.attach(11);
  int count = 0;
  float duration, distance;
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0344;

  Servo1.write(-30);
  while(distance > 30.0 || distance < 2.0){
    delay(50);
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2) * 0.0344;
    rf.print("Distance = ");
    rf.println(distance);
    Serial.println(distance);
  }
  int errCount=0;
  while((distance <= 30.0 && distance >= 2.0 ) || errCount < 3){
    count++;
    if(distance > 30.0 || distance < 2.0 ){
      errCount++;
    }
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2) * 0.0344;
    rf.print("Distance = ");
    rf.println(distance);
    delay(50);
  }
  rf.println("The length is  ");  
  float len = (count-errCount)*4.50;
  rf.println((count-errCount)*4.50);
  bLength = len;
  rf.transmitData(BASE, len);

  rf.print("Should detach");
  Servo1.detach();
  delay(20000);
}

void measureHeight(){
  delay(1000);
  float duration, distance;
  digitalWrite(trigPinU, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPinU, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinU, LOW);
  duration = pulseIn(echoPinU, HIGH);
  distance = (duration / 2) * 0.0344;
  rf.println("distance");
  rf.println(distance);
  rf.println("height: ");
  bHeight = ((30.9-distance)*10);
  rf.println(bHeight);
}


void moveServo(){
  Servo1.attach(11);
  Servo1.write(150);
  delay(650);
  Servo1.detach();
  delay(2000);
  }
  
void checkBlock(){  
  float duration,distance;
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0344;
  if(distance<35){
    while(distance<35){
      rf.println(distance);
      back_off(255,255);
      digitalWrite(trigPin, LOW); 
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin,LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = (duration / 2) * 0.0344;
    }
    advance(255,255);
    delay(250);
    stop();
  }
  else{
    while(distance>35){
      rf.println(distance);
      advance(255,255);
      digitalWrite(trigPin, LOW); 
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin,LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = (duration / 2) * 0.0344;
    }
    back_off(255,255);
    delay(250);
    stop();
  }
}

void checkColor(){
  if (tcs.begin()) {
    Serial.println("Found sensor");
  }
  uint16_t r=0, g=0, b=0, c=0, colorTemp=0, lux=0;
  
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  
  
  if(r>850) {
    rf.transmitData(BONUS, GREEN);
    rf.println(GREEN);
  }
  else {
    rf.transmitData(BONUS, BLACK);
    rf.println(BLACK);
  }
  
  }

void surfaceArea(){
  float bSurfaceArea = bLength*bHeight;
  rf.transmitData(BONUS, bSurfaceArea);
  rf.println(bSurfaceArea);
  }

void loop(){
// wait until location is updated
                  
while(!rf.updateLocation());
if(marker.y>=1){
      // decide whether to move down or up to the checkpoint
  if(marker.y > 1.56) aboveStartingY();
  else belowStartingY();
  // decide whether to move left or right to the checkpoint
  if(marker.x < .75) leftStartingX();
  else rightStartingX();
  // check the wall
  if(checkWall()) wallOnTop();
  else wallOnBottom(); 
}
else{
  if(marker.y > .44) aboveStartingY2();
  else belowStartingY2();
  // decide whether to move left or right to the checkpoint
  if(marker.x < .75) leftStartingX();
  else rightStartingX();
  // check the wall
  if(checkWall()) wallOnTop2();
  else wallOnBottom2(); 
}
                    
  moveToMissionLocation();
  rf.transmitData(NAV, TERRAIN);
  checkColor();
  measureHeight();
  backUp();
  turnToTheta(PI/2);
  moveServo();
  checkBlock();
  measureLength();
  rf.println("Mission Finished");
  surfaceArea();
  rf.transmitData(END_MISSION, NO_DATA);
}
