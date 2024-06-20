// C++ program for the open source power wheel chair youtube project
// Running on esp32 devkit v1

#include<Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <PID_v1.h>


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
int yIn = 34;    // input pin for the joystick x-axis
int xIn = 35;    // input pin for the joystick y-axis
int xValue = 0;  // variable to store the value coming from the x-axis input
int yValue = 0;  // variable to store the value coming from the x-axis input
int xCMD = 0;    // variable to store the command value in % for the x-axis
int yCMD = 0;    // variable to store the command value in % for the y-axis
int rMotSpeed = 12;   // PWM signal for throttle for motor controller a (right)
int rMotFwd  = 14;    // pin for right motor fwd relay control
int rMotRev = 27;     // pin for right motor rev relay control
int lMotSpeed = 26;   /// PWM signal for throttle for motor controller b (left)
int lMotFwd = 25;     // pin for left motor fwd relay control
int lMotRev = 33;     // pin for leftmotor rev relay control
int rMotCalc = 0;     // variable to store the calculated motor cmd percentage
int lMotCalc = 0;     // variable to store the calculated motor cmd percentage
int rMotPowOn = 15;   // pin to turn on relay for motor controller a on (right)
int lMotPowOn = 2;    // pin to turn on relay for motor controller b on (left)
int speedPot = 4;     // potentiometer input for speed control
int speedVal = 0;     // raw 12 bit value to store the speed potentiometer value in to be used to determine pwm output
int pwmVal = 0;       // converted 12 bit value of speedVal to pwm value between 0 and pwmMax
int pwmMax = 4000;    // maximum pwm value between 0 and pwm resolution defined in 'resolution'
int joyStatus = 32;   // Status light for joystick module 
int joyPowSwitch = 0; // Input pullup for power switch on joystick module 
// variables
int cmdLEFT = 0;
int cmdRIGHT = 0;
float angle = 0;
float currentAngle = 0;
float angleLock = 0;
int error = 0;
int absError = 0;
bool powerOn = false;
// pwm properties
const int freq = 500;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 12;
int dutyCycleA = 0;
int dutyCycleB = 0;
// PID properties
// Define Variables we'll be using:
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=5.0, Ki=0.0, Kd=0.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
void run(); // function to run the program
void drive(); // function to control the drive system
void stop(); // stop function
void zeroPtRight(); // function for zero point right turn
void zeroPtLeft(); // function for zero point left turn
double xProcess(int xValue); // function to process the x values from the joystick
double yProcess(int yValue); // function to process the y values from the joystick
float angleProcess(float angle); // function to process the angle values

void setup() {
    Serial.begin(115200);
    // declare the ledPin as an OUTPUT:
    pinMode(xIn, OUTPUT);
    pinMode(yIn, OUTPUT);
    pinMode(rMotPowOn, OUTPUT);
    pinMode(rMotFwd, OUTPUT);
    pinMode(rMotRev, OUTPUT);
    pinMode(rMotSpeed, OUTPUT);
    pinMode(lMotPowOn, OUTPUT);
    pinMode(lMotFwd, OUTPUT);
    pinMode(lMotRev, OUTPUT);
    pinMode(lMotSpeed, OUTPUT);
    pinMode(joyStatus, OUTPUT);
    pinMode(joyPowSwitch, INPUT_PULLUP); 
    ledcSetup(pwmChannelA, freq, resolution);
    ledcSetup(pwmChannelB, freq, resolution);
    ledcAttachPin(rMotSpeed, pwmChannelA);    // attach the pwm channel to the output pin
    ledcAttachPin(lMotSpeed, pwmChannelB);    // attach the pwm channel to the output pin
    speedVal = analogRead(speedPot);
    pwmVal = map(speedVal, 4095, 0, 0, pwmMax);
    Serial.println(pwmVal);
    //initialize the PID variables we're linked to:
    Input = currentAngle;
    Setpoint = angleLock;
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-4000.0, 4000.0);
    /* Initialise the bno055 */
    if(!bno.begin())
    {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
    }
    delay(1000);
}

void loop() {
  powerOn = digitalRead(joyPowSwitch);
  while(!powerOn){
    digitalWrite(rMotPowOn, HIGH);  // turn on relay to power motor controller a (right)
    digitalWrite(lMotPowOn, HIGH);  // turn on relay to power motor controller b (left)
    digitalWrite(joyStatus, HIGH);  // turn on the power status light on the joystick
    run();
    powerOn = digitalRead(joyPowSwitch);
  }
  powerOn = digitalRead(joyPowSwitch);
  digitalWrite(rMotPowOn, LOW);  // turn on relay to power motor controller a (right)
  digitalWrite(lMotPowOn, LOW);  // turn on relay to power motor controller b (left)
  digitalWrite(joyStatus, LOW);  // turn on the power status light on the joystick
  digitalWrite(rMotFwd, LOW);    // pin for right motor fwd relay control
  digitalWrite(rMotRev, LOW);    // pin for right motor rev relay control
  digitalWrite(lMotFwd, LOW);    // pin for left motor fwd relay control
  digitalWrite(lMotRev, LOW);    // pin for leftmotor rev relay control
  
}

void run(){
  // future place holder for startup sequence and safety checks
  //Serial.println("run()");
  powerOn = digitalRead(joyPowSwitch);
  speedVal = analogRead(speedPot);
  pwmVal = map(speedVal, 4095, 0, 0, pwmMax);
  drive();
}

double xProcess(int xValue){
  // Use these if statements to create a deadband for the x value
  // You may need to experiment with these values based on your joystick setup
  if(xValue < 2100 && xValue > 1800){
    return 0;
  }
  xCMD = map(xValue, 0, 4096, -100, 100);
  return xCMD;
}

double yProcess(int yValue){
  // Use these if statements to create a deadband for the y value
  // You may need to experiment with these values based on your joystick setup
  if(yValue < 2100 && yValue > 1800){
    return 0;
  }
  else{
    yCMD = map(yValue, 0, 4096, -100, 100);
    return yCMD;
  }
}

float angleProcess(float angle){
  // process the BNO055 euler.x() value (0-360 degrees) to match the x/y coordinate control system
  if(angle <= 90 && angle >= 0){
    currentAngle = map(angle, 0, 90, 90, 0);
    return currentAngle;
  }
  else if(angle > 90 && angle <= 180){
    currentAngle = map(angle, 90, 180, 0, 270);
    return currentAngle;
  }
  else if(angle > 180 && angle <= 270){
    currentAngle = map(angle, 180, 270, 270, 180);
    return currentAngle;
  }
  else if(angle > 270 && angle <= 360){
    currentAngle = map(angle, 270, 360, 180, 90);
    return currentAngle;
  }
  else{
    return currentAngle;
  }
}

void drive(){
  //Serial.println("drive()");
  powerOn = digitalRead(joyPowSwitch);
  dutyCycleA = 0;
  dutyCycleB = 0;
  xValue = analogRead(xIn);
  yValue = analogRead(yIn);
  xCMD = xProcess(xValue);
  yCMD = yProcess(yValue);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angle = euler.x();
  delay(0.1);
  currentAngle = angleProcess(angle);
  if(yCMD == 0 && xCMD == 0){
    stop();
  }
  if((yCMD < 50 && yCMD > -50)  && xCMD == 100){
    zeroPtRight();
  }
  if((yCMD < 50 && yCMD > -50) && xCMD == -100){
    zeroPtLeft();
  }
  if(yCMD > 50){
    digitalWrite(rMotFwd, LOW);    // pin for right motor fwd relay control
    digitalWrite(rMotRev, LOW);    // pin for right motor rev relay control
    digitalWrite(lMotFwd, LOW);    // pin for left motor fwd relay control
    digitalWrite(lMotRev, LOW);    // pin for leftmotor rev relay control
    delay(0.5);
    digitalWrite(rMotFwd, HIGH);    // pin for right motor fwd relay control
    digitalWrite(rMotRev, LOW);    // pin for right motor rev relay control
    digitalWrite(lMotFwd, HIGH);    // pin for left motor fwd relay control
    digitalWrite(lMotRev, LOW);    // pin for leftmotor rev relay control
    //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    //angle = euler.x();
    currentAngle = angleProcess(angle);
    angleLock = currentAngle;
    powerOn = digitalRead(joyPowSwitch);
    absError = abs(error);
    if(xCMD > 0){
      cmdRIGHT = abs(xCMD);
      cmdLEFT = 0;
    }
    if(xCMD < 0){
      cmdLEFT = abs(xCMD);
      cmdRIGHT = 0;
    }
    rMotCalc = 100 - (cmdLEFT / 2);
    lMotCalc = 100 - (cmdRIGHT / 2);
    dutyCycleA = map(rMotCalc, 0, 100, 0, pwmVal);
    dutyCycleB = map(lMotCalc, 0, 100, 0, pwmVal);
    ledcWrite(pwmChannelA, dutyCycleA);  // right motor command
    ledcWrite(pwmChannelB, dutyCycleB);  // left motor command
    // Serial.print("  ");
    // Serial.print("dcycleA: ");
    // Serial.print(dutyCycleA);
    // Serial.print("  ");
    // Serial.print("dcycleB: ");
    // Serial.print(dutyCycleB);
    // Serial.print("  ");
    // Serial.print("Current Angle: ");
    // Serial.print(currentAngle);
    // Serial.print("  ");
    // Serial.print("Error: ");
    // Serial.print(error);
    // Serial.print("xCMD: ");
    // Serial.print(xCMD);
    // Serial.println("");
    // Serial.print("pwmVal: ");
    // Serial.print(pwmVal);
    // Serial.println("");
  }
  // add reverse directions  
  if(yCMD < -50){
    digitalWrite(rMotFwd, LOW);    // pin for right motor fwd relay control
    digitalWrite(rMotRev, LOW);    // pin for right motor rev relay control
    digitalWrite(lMotFwd, LOW);    // pin for left motor fwd relay control
    digitalWrite(lMotRev, LOW);    // pin for leftmotor rev relay control
    delay(0.5);
    digitalWrite(rMotFwd, LOW);    // pin for right motor fwd relay control
    digitalWrite(rMotRev, HIGH);    // pin for right motor rev relay control
    digitalWrite(lMotFwd, LOW);    // pin for left motor fwd relay control
    digitalWrite(lMotRev, HIGH);    // pin for leftmotor rev relay control
    absError = abs(error);
    if(xCMD > 0){
      cmdRIGHT = abs(xCMD);
      cmdLEFT = 0;
    }
    if(xCMD < 0){
      cmdLEFT = abs(xCMD);
      cmdRIGHT = 0;
    }
    rMotCalc = 100 - (cmdLEFT / 2);
    lMotCalc = 100 - (cmdRIGHT / 2);
    dutyCycleA = map(rMotCalc, 0, 100, 0, pwmVal);
    dutyCycleB = map(lMotCalc, 0, 100, 0, pwmVal);
    ledcWrite(pwmChannelA, dutyCycleA);  // set right motor to reverse direction
    ledcWrite(pwmChannelB, dutyCycleB);  // set left motor to reverse direction
  }
}

void stop(){
  digitalWrite(rMotFwd, LOW);    // pin for right motor fwd relay control
  digitalWrite(rMotRev, LOW);    // pin for right motor rev relay control
  digitalWrite(lMotFwd, LOW);    // pin for left motor fwd relay control
  digitalWrite(lMotRev, LOW);    // pin for leftmotor rev relay control
  delay(0.5);
  dutyCycleA = 0;
  dutyCycleB = 0;
  ledcWrite(pwmChannelA, dutyCycleA);  // right motor command
  ledcWrite(pwmChannelB, dutyCycleB);  // left motor command
}

void zeroPtRight(){
  digitalWrite(rMotFwd, LOW);    // pin for right motor fwd relay control
  digitalWrite(rMotRev, LOW);    // pin for right motor rev relay control
  digitalWrite(lMotFwd, LOW);    // pin for left motor fwd relay control
  digitalWrite(lMotRev, LOW);    // pin for leftmotor rev relay control
  delay(0.5);
  digitalWrite(rMotFwd, HIGH);    // pin for right motor fwd relay control
  digitalWrite(rMotRev, LOW);    // pin for right motor rev relay control
  digitalWrite(lMotFwd, LOW);    // pin for left motor fwd relay control
  digitalWrite(lMotRev, HIGH);    // pin for leftmotor rev relay control
  dutyCycleA = pwmVal;
  dutyCycleB = pwmVal;
  ledcWrite(pwmChannelA, dutyCycleA);
  ledcWrite(pwmChannelB, dutyCycleB);
}

void zeroPtLeft(){
  digitalWrite(rMotFwd, LOW);    // pin for right motor fwd relay control
  digitalWrite(rMotRev, LOW);    // pin for right motor rev relay control
  digitalWrite(lMotFwd, LOW);    // pin for left motor fwd relay control
  digitalWrite(lMotRev, LOW);    // pin for leftmotor rev relay control
  delay(0.5);
  digitalWrite(rMotFwd, LOW);    // pin for right motor fwd relay control
  digitalWrite(rMotRev, HIGH);    // pin for right motor rev relay control
  digitalWrite(lMotFwd, HIGH);    // pin for left motor fwd relay control
  digitalWrite(lMotRev, LOW);    // pin for leftmotor rev relay control
  dutyCycleA = pwmVal;
  dutyCycleB = pwmVal;;
  ledcWrite(pwmChannelA, dutyCycleA);
  ledcWrite(pwmChannelB, dutyCycleB);
}


