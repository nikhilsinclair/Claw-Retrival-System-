
// --------------------------------------------------------------------------------------
// APSC101 - 2020WT2
// Code for video Arduino 06  -  Servo + Serial Monitor and External Switch
//
//
// video located at:
// 
//https://learning.video.ubc.ca/media/Arduino+06_+Servo%2C+Serial+Monitor+an
// d+External+Switch/0_6ebcb6w1
//
// This program controls a servo motor to go to two different
// positions according to the state of an input switch
// and shows switch & servo status on the screen of an attached
// computer, via the Arduino Serial Monitor.
// --------------------------------------------------------------------------------------
#include <Servo.h>  //include the servo library for this program Servo myservo;              

/*

trigger: 11
echo: 12

joystick: 1
servo: 6

VCC: 13
ground: 10

*/

#include <NewPing.h>      // include the NewPing library for this program
#define VCC_PIN 13
#define TRIGGER_PIN 11    // sonar trigger pin will be attached to Arduino pin 12
#define ECHO_PIN 12       // sonar echo pint will be attached to Arduino pin 11
#define GROUND_PIN 10
#define MAX_DISTANCE 200  // fmaximum distance set to 200 cm

#define JOYSTICK_PIN 1
#define SERVO_PIN 6

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // initialize NewPing

Servo myservo;  //create servo object to control a servo

#define CLOSED 0
#define OPEN 1

#define CLOSE_DELAY 1000
#define OPEN_DELAY 1000

#define OPEN_POSITION 180
#define CLOSE_POSITION 0

bool claw = 1;

int joystickMax = 0;
int joystickMin = 1023;
int count = 0;
int low = 0;
bool lowB = false;


void setup() {
  
  Serial. begin(9600);      //set data transmission rate to communicate  with computer
  pinMode(8,INPUT_PULLUP);  //pin 8 forced to HIGH when there is no  external input.   //.      what is this pin used for?
  myservo.attach(SERVO_PIN);        // servo pin
//  myservo.write(60);       //tells servo to go to 60 degree position


  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(GROUND_PIN, OUTPUT);    // tell pin 10 it is going to be an output
  pinMode(VCC_PIN, OUTPUT);       // tell pin 13 it is going to be an output

  pinMode(JOYSTICK_PIN, INPUT);
  pinMode(A1, INPUT);
  
  digitalWrite(GROUND_PIN,LOW);   // tell pin 10 to output LOW (OV, or ground)
  digitalWrite(VCC_PIN, HIGH); 

  delay(1000);

}
void loop() {


  int DISTANCE_IN_CM = getDistance();
  unsigned long startTime;

  if (DISTANCE_IN_CM <= 10){
    Serial.println(DISTANCE_IN_CM);  

    startTime = millis();
     while(millis()-startTime<1700){

    }
    
    myservo.write(180);
    Serial.println("Closed");   
   
    startTime = millis();
    while(millis()-startTime<5000){

    }

  }

  myservo.write(0);
  
}
  //myservo.write(180);
  //toggle(DISTANCE_IN_CM, 5);
  //moveWithHeight(10, DISTANCE_IN_CM);//--chagne to send distnace first
  
  
  //moveWithJoystick(); 
  //find the angle of the servo which work - have to mount it  
  
  
  //timerMode();

  //clawMode_toggleAtDistance(10,10);
  //toggle(DISTANCE_IN_CM, 10, 5000, 2000, 3000);

  

/*
Immediately opens the claw and keeps it open for openTime, then closes it for closeTime

*/
void timerMode(int openTime, int closeTime){


  myservo.write(OPEN_POSITION);
  delay(openTime);//open delay
    

  myservo.write(CLOSE_POSITION);
  delay(closeTime);//close delay
  

}


/*
Opens the claw after a delay if the claw is below the clawOPenHeight.
Closes the claw after a delay if the claw is below the clawCloseHeight

*/
void clawMode_toggleAtDistance(int clawOpenHeight, int clawCloseHeight){


  int distance = getDistance();
    
  if (claw == CLOSED && distance < clawOpenHeight){
    delay(OPEN_DELAY);//open delay
    myservo.write(0);
    claw = OPEN;
  }

  else if (distance<clawCloseHeight){
    delay(CLOSE_DELAY);//close delay
    myservo.write(180);
    claw = CLOSED;
  }

}

/*
Gets distance from the sonar and prints in to screen, and returns it in CM.

*/
int getDistance(){

  int distance = sonar.ping_cm();   // read the sonar sensor, using a variable
  Serial.print("Ping: ");                 //print “Ping:" on the computer display
  Serial.print(distance);           //print the value of the variable next
  Serial.println("cm"); 

  return distance;

}

/*
Moves the servo as a variable of the height. From the startHeight to 0.

*/
void moveWithHeight(int startHeight, int distance){

  int val;           // reads the value of the potentiometer (value between 0 and 1023)
  val = map(distance, 0, startHeight, 0, 180);     // scale it for use with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  //delay(15);   

}

/*
Moves the servo with the joystick

*/
void moveWithJoystick(){

  int joystick = analogRead(JOYSTICK_PIN);


 //joystickMax += joystick;

 if (joystickMax <= joystick){
   joystickMax = joystick;


 }
 else  if (joystickMin >= joystick){
   joystickMin = joystick;


 }

  
  Serial.print("JoystickMax: ");
  Serial.println(joystickMax);

  Serial.print("JoystickMin: ");
  Serial.println(joystickMin);
 count++;

 Serial.println(joystickMax/count);

  int val;
  val = map(joystick, 330, 610, 0, 180);

  /*if(joystick>360 && joystick <460){
    val = 90    
;  }*/

  myservo.write(val);
  Serial.print("Joystick: ");
  Serial.print(joystick);
  Serial.print(", Servo: ");
  Serial.println(val);
  //delay(10);
  
}

/*
Opens the claw at toggleDistance. Closes the claw above toggle position

*/
void toggle(int distance, int toggleDistance, int closetime, int opentime, int threshold){
  
  if(distance < toggleDistance && !lowB) {
    low = millis();
    lowB = true;
  }
  if(lowB && distance > toggleDistance){
    lowB = false;
  }
  if(lowB && (threshold < millis() - low)){
    myservo.write(CLOSE_POSITION);      //servo position is 60 degrees
    delay(closetime);
    myservo.write(OPEN_POSITION);
    delay(opentime);
    Serial.print("OPEN");   //print the word "open"
    Serial.println("");     //print nothing, go to next line
  }

}