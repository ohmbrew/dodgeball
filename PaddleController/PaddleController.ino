/*
  Dodgeball Motor Controller

  LattePanda Arduino pinout
   B   C   E   F           D   A                   <- All these are P1 signals
  D0  D1  D2  D3  D4  D5  D6  A0  A1  A2  5V  5V
   |   |   |   |   |   |   |   |   |   |   |   | 

   |   |   |   |   |   |   |   |   |   |   |   |   
  D7  D8  D9  D10 D11 D12 D13 A3  A4  A5  GND GND
   H   I   K   L           J   G                   <- All these are P2 signals

  Player 1 signals:
  A: Pot P1
  B: Limit Switch P1
  C: Motor Direction P1
  D: Motor PWM P1
  E: Motor Encoder A P1
  F: Motor Encoder B P1

  Player 2 signals:
  G: Pot P2
  H: Limit Switch P2
  I: Motor Direction P2
  J: Motor PWM P2
  K: Motor Encoder A P2
  L: Motor Encoder B P2



  The circuit:
  - potentiometer control for paddle left/right
    - Ways to control paddle:
      1. use a pot that has left/right limits. Those limits equate to paddle far left/far right. As you move pot, it moves paddle.
      2. use a rotary optical encoder that can free spin. Faster the encoder spins, the faster the paddle moves in that direction. Auto stops at wall.
    - I like option 2. Feels the best...I bet spinning that thing as fast as you can will be popular.
    - center pin of the potentiometer to the analog input 0. One side pin (either one) to ground the other side pin to +5V
  - drive 2x pins (pwmP1 and pwmP2) with AnalogWrite (see below) to drive PWM signal to H-Bridge chip to motor
  - motor encoder signals (2 signals per motor = 4) going to inputs (M1=[D0,D1] and M2=[D2,D3]) used to calculate distance moved

  created by Greg Brault (Ohmbrew) for use in Dodge (pin) Ball game
  modified 5 Nov 2023
  modified 16 Nov - tested out with belted drive system. Uploading to 
  Notes:
  Using example Analog Serial Monitoring, pot currently reports 0 for full turned one way, 1023 for full turned other way. So use that as scale for
    PWM motor control
  PWM using pins (from https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm)
    The Arduino's programming language makes PWM easy to use; simply call analogWrite(pin, dutyCycle), where dutyCycle is a value from 0 to 255, and pin is one of the PWM pins (3, 5, 6, 9, 10, or 11).
    The analogWrite() function provides a simple interface to the hardware PWM, but doesn't provide any control over frequency.
*/

#include <Encoder.h>

#define SET_DIR_P1_LEFT digitalWrite(dirPinP1, LOW)
#define SET_DIR_P1_RIGHT digitalWrite(dirPinP1, HIGH)
#define SET_DIR_P2_LEFT digitalWrite(dirPinP2, LOW)
#define SET_DIR_P2_RIGHT digitalWrite(dirPinP2, HIGH)

#define DEAD_ZONE     100      // TODO: tune this empirically
#define DECEL_RANGE   250     // TODO: tune this empirically. Distance to start deceleration mapping. Otherwise, full speed ahead!
#define MIN_MOTOR_POS 100     // TODO: tune this empirically
#define HOMING_SPEED  25      // TODO: tune this empirically

// Player 1 signal pins
int potPinP1 = A0;      // pot connected to A0 on LattePanda board (Arduino section)
int pwmPinP1 = 6;       // motor PWM (connects to H-Bridge motor driver boards I bought)
int dirPinP1 = 1;       // motor direction signal (connects to H-Bridge motor driver boards I bought)
int limitSwitchP1 = 0;  // limit switch
int motorEncP1A = 2;    // motor encoder A
int motorEncP1B = 3;    // motor encoder B

// Player 2 signal pins
int potPinP2 = A3;      // pot connected to A3 on LattePanda board (Arduino section)
int pwmPinP2 = 13;      // motor PWM (connects to H-Bridge motor driver boards I bought)
int dirPinP2 = 8;       // motor direction signal (connects to H-Bridge motor driver boards I bought)
int limitSwitchP2 = 7;  // limit switch
int motorEncP2A = 9;    // motor encoder A
int motorEncP2B = 10;   // motor encoder B

int ledPin = 13;        // default red led. Will place blinks in certain parts of code

int potP1 = 0;          // player 1's pot A/D value (0-1023)
int potP2 = 0;          // player 2's pot A/D value (0-1023)

int speed = 40;

Encoder MotorP1(motorEncP1A, motorEncP1B);      // set to input...tho I think Encoder() does it too
Encoder MotorP2(motorEncP2A, motorEncP2B);      // set to input...tho I think Encoder() does it too

// these are configured during calcMaxTicks() and should only change when re-calibrating, or after a limit switch crash
const long settingsPosMinMotorP1 = MIN_MOTOR_POS; // hard set from empirical testing
const long settingsPosMinMotorP2 = MIN_MOTOR_POS;
long settingsPosCenterMotorP1 = 0;
long settingsPosCenterMotorP2 = 0;
long settingsPosMaxMotorP1 = -1;    // saved during calcMaxTicks()
long settingsPosMaxMotorP2 = -1;    // saved during calcMaxTicks()

// these track current motor position and set position. set position is changed based on analog pot input in loop()
long posMotorP1 = 0;
long posMotorP2 = 0;
long setPointMotorP1 = 0;
long setPointMotorP2 = 0;
int isHomed = 0;

void setup() {
  pinMode(ledPin, OUTPUT);  // make LED output

  // Player 1 signals I/O configure
  pinMode(potPinP1, INPUT);       // Player 1 Pot Analog
  pinMode(limitSwitchP1, INPUT_PULLUP);  // Player 1 Limit Switch
  pinMode(dirPinP1, OUTPUT);      // Player 1 Motor Direction
  pinMode(2, INPUT);              // Player 1 Motor Encoder A
  pinMode(3, INPUT);              // Player 1 Motor Encoder B
  analogWrite(pwmPinP1, 0);       // init Player 1 Motor PWM to 0

  // Player 2 signals I/O configure
  pinMode(potPinP2, INPUT);       // Player 2 Pot Analog
  pinMode(limitSwitchP2, INPUT_PULLUP);  // Player 2 Limit Switch
  pinMode(dirPinP2, OUTPUT);      // Player 2 Motor Direction
  pinMode(2, INPUT);              // Player 2 Motor Encoder A
  pinMode(3, INPUT);              // Player 2 Motor Encoder B
  analogWrite(pwmPinP1, 0);       // init Player 2 Motor PWM to 0

  Serial.begin(9600);             // initialize serial communication at 9600 bits per second
  //delay(1000);
  //Serial.println("Connected.");
  //homePaddle();
  //calcMaxTicks();
}

/*
  Homing Algorithm for a player 1:
  - Move motor left fast enough to get it moving at constant speed, but no more. We are trying to sense when limit switch is hit
  - when limit switch hit, stop motor
  - move motor right for 1/2 sec. This is to accomodate if limit switch was closed when routine first started. Want to back it off.
  - Move motor left again until limit switch hit
  - when limit switch hit, stop motor
  - reset motor position to 0
  - we know our max position from empirical testing
  - move to minPosMotorP1. This is as far left as we should go in our drive() algorithm!
*/
void homePaddle() {
  SET_DIR_P1_LEFT;
  analogWrite(pwmPinP1, HOMING_SPEED);  // start moving motor at specified homing speed
  Serial.print("P1 Motor going left at HOMING_SPEED ");
  Serial.print(HOMING_SPEED);
  Serial.println(". Waiting for P1 limit switch.");
  while (digitalRead(limitSwitchP1) == 1);  // loop until limit switch is hit
  analogWrite(pwmPinP1, 0);   // stop motor
  Serial.println("Stopped P1 Motor.");
  SET_DIR_P1_RIGHT;
  analogWrite(pwmPinP1, HOMING_SPEED);  // start moving right for 1 sec. Want to back it off if limit switch was down when started
  Serial.println("Backing off P1 motor.");
  delay(1000);  // give motor time to back off
  SET_DIR_P1_LEFT;
  analogWrite(pwmPinP1, HOMING_SPEED);  // start moving motor at specified homing speed again. This time we know we were off limit switch
  Serial.print("P1 Motor going left at HOMING_SPEED ");
  Serial.print(HOMING_SPEED);
  Serial.println(". Waiting for P1 limit switch.");
  while (digitalRead(limitSwitchP1) == 1);  // loop until limit switch is hit
  analogWrite(pwmPinP1, 0); // stop motor
  Serial.println("Stopped P1 Motor.");
  delay(1000);  // wait for any decel
  posMotorP1 = 0; // reset motor position to 0
  MotorP1.write(0);   // reset encoder position to 0
  settingsPosCenterMotorP1 = posMotorP1 + 100;   // just something farther out. Will be updated on calcMaxTix()
  settingsPosMaxMotorP1 = MIN_MOTOR_POS;  // setting these equal will flag other routines
  Serial.println("P1 Motor Homed.");
  isHomed = 1;
}

/*
 Calculate # of ticks from home position to specified right end of playfield area (controlled via pot/button)
 - Home motor
 - while pot is > 512 (halfway), move motor right fast enough (similar to homing speed) to get it moving at constant speed
 - when pot goes < 512, stop motor
 - save # encoder ticks from home to opposite side as maxPosMotorP1
 */
 // TODO: design in a limit switch on the right side so we can fully automate homing/max tick calculation
void calcMaxTicks() {
  homePaddle();
  Serial.println("Waiting for pot left.");
  while (analogRead(potPinP1) > 512); // wait for user to rotate to left half
  delay(500);
  Serial.println("Waiting for pot right.");
  while (analogRead(potPinP1) < 512); // wait for user to rotate to right half
  delay(500);
  SET_DIR_P1_RIGHT;
  analogWrite(pwmPinP1, HOMING_SPEED);  // start moving right
  Serial.print("P1 Motor going right at HOMING_SPEED ");
  Serial.print(HOMING_SPEED);
  Serial.println(". Waiting for pot left to stop motor at right end.");
  while (analogRead(potPinP1) > 512); // wait for user to rotate to left half - this signifies the end value
  analogWrite(pwmPinP1, 0); // stop motor
  delay(1000);  // wait for any decel
  settingsPosMaxMotorP1 = MotorP1.read();   // save # ticks it took to get to user-specified right end position
  Serial.print("Calculated settingsPosMaxMotorP1: ");
  Serial.print(settingsPosMaxMotorP1);
  Serial.println("\n");
}

// compare analog value of pot (0-1023)to min/max range of motor. This is our set point
void move() {
  potP1 = analogRead(potPinP1);   // read player 1 pot analog value - this is our set point
  setPointMotorP1 = map(potP1, 0, 1023, settingsPosMinMotorP1, settingsPosMaxMotorP1);   // calculate where pot is wrt motor min and max
  if (abs(setPointMotorP1 - posMotorP1) <  DEAD_ZONE) {
    analogWrite(pwmPinP1, 0);   // we are within dead zone of motor encoder reading vs set point - stop motor!
    return;   // we are done
  } else if (setPointMotorP1 - posMotorP1 < 0) {
    // need to move left
    SET_DIR_P1_LEFT;
  } else if (setPointMotorP1 - posMotorP1 > 0) {
    // need to move right
    SET_DIR_P1_RIGHT;
  }
  // if we got here, we are NOT in the dead zone and we've set our P1 motor direction
  // Now set PWM for P1 motor based on how far we are from set point
  // first check if we need to scale deceleration
  if (abs(setPointMotorP1 - posMotorP1) < DECEL_RANGE) {
    analogWrite(pwmPinP1, map(abs(setPointMotorP1 - posMotorP1), 0, DECEL_RANGE, HOMING_SPEED, 255)); // maps abs difference of our set point and motor position
                                                                            // to scaled PWM (min: HOMING_SPEED, max: 255). Larger difference = more power
  } else {
    analogWrite(pwmPinP1, 255); // not within decel range. Full steam ahead!
  }
  
}

void loop() {
  if (isHomed == 0) {
    delay(2000);
    Serial.println("");
    Serial.println("Connected to Dodgeball.\n");
    homePaddle();
    calcMaxTicks();
  }
  // get current motor encoder values to update posMotorP1
  posMotorP1 = MotorP1.read();
  move();
//  long newPosMotorP1, newPosMotorP2;
//
//  
//
//  if (potP1 >= 492 and potP1 <= 532) {  // P1 analog stick near middle?
//      analogWrite(pwmPinP1, 0);            // turn off P1 pwm 
//  } else if (potP1 < 492) {             // pot is somewhere in negative region
//    potP1 = map(potP1, 491, 0, 0, 255);     // scale it for use with PWM functions on pins 5 and 6.
//    digitalWrite(dirPinP1, 0);             // moving left
//    analogWrite(pwmPinP1, potP1);          // write scaled PWM based on how far we are from 491
//  } else {  // pot is somewhere in positve region
//    potP1 = map(potP1, 533, 1023, 0, 255);     // scale it for use with PWM functions on pins 5 and 6.
//    digitalWrite(dirPinP1, 1);             // moving right
//    analogWrite(pwmPinP1, potP1);          // write scaled PWM based on how far we are from 491
//  }
  //

  // for (int x = 0; x < 10; x++) {
  //   newPosMotorP1 = MotorP1.read();
  //   //newPosMotorP2 = MotorP2.read();
  //   if (newPosMotorP1 != currPosMotorP1 /*|| newPosMotorP2 != currPosMotorP2*/) {
  //     Serial.print("posMotorP1: ");
  //     Serial.print(newPosMotorP1);
  //     //Serial.print(" | posMotorP2: ");
  //     //Serial.print(newPosMotorP2);
  //     Serial.println();
  //     currPosMotorP1 = newPosMotorP1;
  //     currPosMotorP2 = newPosMotorP2;
  //   }
  //   if (x % 10 == 0) {
  //     speed += 10;
  //     analogWrite(pwmP1, speed);
  //   }
  //   delay(1000);
    
  //}
}
  
