// Simple_navigation.ino
// Super simple navigation for your sumobot
// Not a fighting behavior, just collision avoidance
// Written by Brian Bailey
// brian@bunedoggle.com
// This code released under a Beerware license
// free for all to use, modify and share

/*******************************************************
* Include statements - include extra code we need
*******************************************************/
// This line includes the code we need to control servos
//#include <Servo.h>
// This line includes the code we need to use the ultrasonic range finder
#include <NewPing.h>

#include <toneAC.h>

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>


/*******************************************************
* Define statements - Define constants we will use later
*******************************************************/
#define CENTER_TRIGGER_PIN        A0    // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define CENTER_ECHO_PIN           A1    // Arduino pin tied to echo pin on the ultrasonic sensor.
#define RIGHT_TRIGGER_PIN         6     // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define RIGHT_ECHO_PIN            7     // Arduino pin tied to echo pin on the ultrasonic sensor.
#define LEFT_TRIGGER_PIN          5     // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define LEFT_ECHO_PIN             4     // Arduino pin tied to echo pin on the ultrasonic sensor.

#define LEFT_WHEEL_PIN    5     // Arduino pin tied to the left servo wheel motor
#define RIGHT_WHEEL_PIN   6     // Arduino pin tied to the right servo wheel motor
#define MAX_DISTANCE      400   // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define FRONT_EDGE_SENSOR A4    // Pin connected to the front edge IR sensor
#define REAR_EDGE_SENSOR  A5    // Pin connected to the rear edge IR sensor
#define LIGHT_COLOR_VALUE 512   // This is a value returned from the edge sensors when they see a lighter color, may need tweaking
#define SABER_SERIAL_PIN  2     // Serial connection to sabertooth controller
#define LEFT_WHEEL        1     // For Sabertooth controller 
#define RIGHT_WHEEL        2     // For Sabertooth controller 
#define MAX_POSITIVE_SPEED   96
#define MAX_NEGATIVE_SPEED   -96

/********************************************************
* Class object instances - These will help us control the
* sensors and motors.
********************************************************/
// NewPing setup of pins and maximum distance.
NewPing sonar(CENTER_TRIGGER_PIN, CENTER_ECHO_PIN, MAX_DISTANCE);

NewPing left_sonar(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);

NewPing right_sonar(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);


// Servo motor control objects
// Up to twelve servo objects can be created and controlled
//Servo rightWheel;
//Servo leftWheel;

// Setup the serial communication
SoftwareSerial SWSerial(NOT_A_PIN, SABER_SERIAL_PIN); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.


/********************************************************
* Setup code runs one time at the very beginning after we
* power up (or reset)
********************************************************/
void setup()
{
  // Open serial monitor so we can print out debug information
  // When connected to a USB port
  Serial.begin(9600);

  SWSerial.begin(9600);

  // Tell the servo objects which pins the servos are connected to
//  leftWheel.attach(LEFT_WHEEL_PIN, 1000, 2000);
//  rightWheel.attach(RIGHT_WHEEL_PIN, 1000, 2000);

}

/********************************************************
* Loop code runs over and over, forever while powered on
********************************************************/
void loop()
{

//  talk();
 
  // Variables we will need in the loop code
  int distInCentimeters;  // We'll store the ultrasonic range distance here
  int leftDistInCent;     // We'll store the ultrasonic range distance here
  int rightDistInCent;    // We'll store the ultrasonic range distance here

  /*******************************
  * Ultrasonic Range Finder Code
  ********************************/
  // This code takes a distance reading from the range finder
  distInCentimeters = ping_cm_BugFix(sonar, CENTER_ECHO_PIN);
  leftDistInCent    = ping_cm_BugFix(left_sonar, LEFT_ECHO_PIN);
  rightDistInCent   = ping_cm_BugFix(right_sonar, RIGHT_ECHO_PIN);

  // Print distance for debug (0 = outside set distance range)
  Serial.print("Center: ");
  Serial.print(distInCentimeters);
  Serial.println("cm");

  Serial.print("Left: ");
  Serial.print(leftDistInCent);
  Serial.println("cm");

  Serial.print("Right: ");
  Serial.print(rightDistInCent);
  Serial.println("cm");


  // Open area pounce mode
  if(leftDistInCent > 200 && rightDistInCent > 200 && distInCentimeters > 200 &&
     leftDistInCent!= 999 && rightDistInCent!= 999 && distInCentimeters!= 999)
  {
    ST.motor(LEFT_WHEEL, 0);
    ST.motor(RIGHT_WHEEL, 0);

    do{
      distInCentimeters = ping_cm_BugFix(sonar, CENTER_ECHO_PIN);
      leftDistInCent    = ping_cm_BugFix(left_sonar, LEFT_ECHO_PIN);
      rightDistInCent   = ping_cm_BugFix(right_sonar, RIGHT_ECHO_PIN);

      Serial.println("Waiting to pounce...");

      // Print distance for debug (0 = outside set distance range)
      Serial.print("Center: ");
      Serial.print(distInCentimeters);
      Serial.println("cm");

      Serial.print("Left: ");
      Serial.print(leftDistInCent);
      Serial.println("cm");

      Serial.print("Right: ");
      Serial.print(rightDistInCent);
      Serial.println("cm");

      execute_random();
      
    }while(leftDistInCent > 100 && rightDistInCent > 100 && distInCentimeters > 100);
  }

  // If there's something left nudge right
  if(leftDistInCent < 10)
  {
    Serial.println("Right nudge");
    ST.motor(LEFT_WHEEL, MAX_POSITIVE_SPEED);
    ST.motor(RIGHT_WHEEL, MAX_POSITIVE_SPEED);
    delay(300);
    ST.motor(LEFT_WHEEL,  MAX_NEGATIVE_SPEED);
    ST.motor(RIGHT_WHEEL, MAX_POSITIVE_SPEED);
  }
  // If there's somethign right nudge left
  if(rightDistInCent < 10)
  {
    Serial.println("Left nudge");
    ST.motor(LEFT_WHEEL, MAX_NEGATIVE_SPEED);
    ST.motor(RIGHT_WHEEL, MAX_NEGATIVE_SPEED);
    delay(300);
    ST.motor(LEFT_WHEEL,  MAX_NEGATIVE_SPEED);
    ST.motor(RIGHT_WHEEL, MAX_POSITIVE_SPEED);    
  }
  // If don't see an obsticle keep driving
  if(distInCentimeters > 30 || distInCentimeters == 0){
    Serial.println("Driving");
    ST.motor(LEFT_WHEEL,  MAX_NEGATIVE_SPEED);
    ST.motor(RIGHT_WHEEL, MAX_POSITIVE_SPEED);
    //leftWheel.write(0);
    //rightWheel.write(180);
  }
  // Otherwise turn and find a clearer path
  else {
                while(distInCentimeters < 60){
      Serial.println("Looking around");
      ST.motor(LEFT_WHEEL, MAX_POSITIVE_SPEED);
      ST.motor(RIGHT_WHEEL, MAX_POSITIVE_SPEED);
      //leftWheel.write(180);
      //rightWheel.write(180);
                  distInCentimeters = ping_cm_BugFix(sonar, CENTER_ECHO_PIN);
                  Serial.print("Seeing things ");
                  Serial.print(distInCentimeters);
                  Serial.println(" cm away");
                }
                Serial.println("Looks clear!");
  }

  execute_random();

}

void execute_random()
{
  if(random(0,100) == 1)
  {
      Serial.println("Disco Time!!");
      ST.motor(RIGHT_WHEEL, MAX_POSITIVE_SPEED);
      ST.motor(LEFT_WHEEL, MAX_POSITIVE_SPEED);    
      delay(random(500,1500));
  }

  if(random(0,200) == 2){
    ST.motor(RIGHT_WHEEL, -25);
    ST.motor(LEFT_WHEEL, -25);
    talk();
    ST.motor(RIGHT_WHEEL, 25);
    ST.motor(LEFT_WHEEL, 25);
    talk();
  }
}

// This is a wrapper function that tries to avoid a bug with the
// HC-SR04 modules where they can get stuck return zero forever
int ping_cm_BugFix(NewPing sensor, int echo_pin){
  
        int distCm = sensor.ping_cm();
        
        if(distCm == 0){
          delay(100);
          pinMode(echo_pin, OUTPUT);
          digitalWrite(echo_pin, LOW);
          delay(100);
          pinMode(echo_pin, INPUT);
        }

        if(distCm == 0){
          distCm = 999;
        }
      
        return distCm;
}

void talk(){
  int t,i;
  t = random(65, 250);
  for(i=0;i<random(3,12);i++){
     toneAC(t+random(0,5)*t);
     delay(random(100,200));
     delay(random(100,200)*1.3);
     toneAC(0);
  }
}


