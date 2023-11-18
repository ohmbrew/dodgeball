/*
  Dodgeball Motor Controller

  LattePanda Arduino pinout
   B   C   E   F   G       D   A                   <- All these are P1 signals
  D0  D1  D2  D3  D4  D5  D6  A0  A1  A2  5V  5V
   |   |   |   |   |   |   |   |   |   |   |   | 

   |   |   |   |   |   |   |   |   |   |   |   |   
  D7  D8  D9  D10 D11 D12 D13 A3  A4  A5  GND GND
   H   I   K   L           J   G                   <- All these are P2 signals

  Player 1 signals:
  A: Pot P1
  B: Limit Switch Left P1
  C: Motor Direction P1
  D: Motor PWM P1
  E: Motor Encoder A P1
  F: Motor Encoder B P1
  G: Limit Switch Right P1
  
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

#define IS_DEBUG 1
#define SET_DIR_P1_LEFT digitalWrite(dirPinP1, LOW)
#define SET_DIR_P1_RIGHT digitalWrite(dirPinP1, HIGH)
#define SET_DIR_P2_LEFT digitalWrite(dirPinP2, LOW)
#define SET_DIR_P2_RIGHT digitalWrite(dirPinP2, HIGH)

// These values were all tested empirically and they work well together
#define DEAD_ZONE     350     // 17 Nov. 350 is lowest we can go with no minute oscillations (with .5 pot filter).
#define DECEL_RANGE   2000     // TODO: tune this empirically. Distance to start deceleration mapping. Otherwise, full speed ahead!
#define EDGE_OFFSET   100     // 17 Nov. This seems good.
#define BACKOFF_TIME  500     // 18 Nov. This is good
#define HOMING_SPEED_SLOW 40  // 18 Nov this is good
#define HOMING_SPEED_FAST 200
#define HOMING_FAST_TIME  1800
#define INITIAL_PWM   40
#define ACCELERATION_INIT 5
#define DECELERATION_INIT 10
#define DECELERATION_VAL 1
#define ACCELERATION_VAL 4
#define ACCELERATION_UPDATES_PER_SECOND 30


#define STATE_M1_STOPPED  0
#define STATE_M1_TRACKING 1
#define STATE_M2_STOPPED  0
#define STATE_M2_TRACKING 1

// Player 1 signal pins
int potPinP1 = A0;        // pot connected to A0 on LattePanda board (Arduino section)
int pwmPinP1 = 6;         // motor PWM (connects to H-Bridge motor driver boards I bought)
int dirPinP1 = 1;         // motor direction signal (connects to H-Bridge motor driver boards I bought)
int limitSwitchP1L = 0;   // limit switch left
int limitSwitchP1R = 4;   // limit switch right
int motorEncP1A = 2;      // motor encoder A
int motorEncP1B = 3;      // motor encoder B


// Player 2 signal pins
int potPinP2 = A3;        // pot connected to A3 on LattePanda board (Arduino section)
int pwmPinP2 = 13;        // motor PWM (connects to H-Bridge motor driver boards I bought)
int dirPinP2 = 8;         // motor direction signal (connects to H-Bridge motor driver boards I bought)
int limitSwitchP2L = 7;   // limit switch left
int limitSwitchP2R = 11;  // limit switch right
int motorEncP2A = 9;      // motor encoder A
int motorEncP2B = 10;     // motor encoder B

int ledPin = 13;        // default red led. Will place blinks in certain parts of code

int potP1 = 0;          // player 1's pot A/D value (0-1023)
float potFilterParam = 0.5;
int potP1_smoothed = 0; // moving average low pass filter for PotP1
int stateM1 = STATE_M1_STOPPED;

int potP2 = 0;          // player 2's pot A/D value (0-1023)
int potP2_smoothed = 0; // moving average low pass filter for PotP2
int stateM2 = STATE_M2_STOPPED;

Encoder MotorP1(motorEncP1A, motorEncP1B);      // set to input...tho I think Encoder() does it too
Encoder MotorP2(motorEncP2A, motorEncP2B);      // set to input...tho I think Encoder() does it too

// these are configured during calcMaxTicks() and should only change when re-calibrating, or after a limit switch crash
const long settingsPosMinMotorP1 = EDGE_OFFSET;   // hard set from empirical testing
const long settingsPosMinMotorP2 = EDGE_OFFSET;
long settingsPosCenterMotorP1 = 0;
long settingsPosCenterMotorP2 = 0;
long settingsPosMaxMotorP1 = -1;    // saved during calcMaxTicks()
long settingsPosMaxMotorP2 = -1;    // saved during calcMaxTicks()

// these track current motor position and set position. set position is changed based on analog pot input in loop()
long posMotorP1 = 0;
long posMotorP2 = 0;
long setPointMotorP1 = 0;
long setPointMotorP2 = 0;
int pwmSpeedMotorP1 = 0;
int pwmSpeedMotorP2 = 0;
int isHomed = 0;

int is_accelerating = 0;  // used to set initial acceleration
int acceleration = 0;   // set this to whatever min speed you want to start at. It will increase it each move() until 255.

long update_time = 0;
long update_acceleration = 0;

void setup() {
  pinMode(ledPin, OUTPUT);  // make LED output

  // Player 1 signals I/O configure
  pinMode(potPinP1, INPUT);       // Player 1 Pot Analog
  pinMode(limitSwitchP1L, INPUT_PULLUP);  // Player 1 Limit Switch Left input with pullup
  pinMode(limitSwitchP1R, INPUT_PULLUP);  // Player 1 Limit Switch Right input with pullup
  pinMode(dirPinP1, OUTPUT);      // Player 1 Motor Direction
  pinMode(2, INPUT);              // Player 1 Motor Encoder A
  pinMode(3, INPUT);              // Player 1 Motor Encoder B
  analogWrite(pwmPinP1, 0);       // init Player 1 Motor PWM to 0

  // Player 2 signals I/O configure
  pinMode(potPinP2, INPUT);       // Player 2 Pot Analog
  pinMode(limitSwitchP2L, INPUT_PULLUP);  // Player 2 Limit Switch Left input with pullup
  pinMode(limitSwitchP2R, INPUT_PULLUP);  // Player 2 Limit Switch Right input with pullup
  pinMode(dirPinP2, OUTPUT);      // Player 2 Motor Direction
  pinMode(2, INPUT);              // Player 2 Motor Encoder A
  pinMode(3, INPUT);              // Player 2 Motor Encoder B
  analogWrite(pwmPinP1, 0);       // init Player 2 Motor PWM to 0

  Serial.begin(9600);             // initialize serial communication at 9600 bits per second
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
  // zero our position
  SET_DIR_P1_LEFT;
  if (IS_DEBUG) Serial.print("P1 Motor tracking left until limit switch hit. HOMING_SPEED_SLOW: ");
  if (IS_DEBUG) Serial.print(HOMING_SPEED_SLOW);
  if (IS_DEBUG) Serial.println("");
  analogWrite(pwmPinP1, HOMING_SPEED_SLOW);  // start moving motor aleft
  while (digitalRead(limitSwitchP1L) == 1);  // loop until left limit switch is hit
  analogWrite(pwmPinP1, 0);   // stop motor
  if (IS_DEBUG) Serial.println("Stopped P1 Motor. Backing it off.");
  SET_DIR_P1_RIGHT;
  analogWrite(pwmPinP1, HOMING_SPEED_SLOW);  // start moving right for 1 sec. Want to back it off if limit switch was down when started
  delay(BACKOFF_TIME);  // give motor time to back off
  SET_DIR_P1_LEFT;
  if (IS_DEBUG) Serial.print("P1 Motor tracking left until limit switch hit. HOMING_SPEED_SLOW: ");
  if (IS_DEBUG) Serial.print(HOMING_SPEED_SLOW);
  if (IS_DEBUG) Serial.println("");
  analogWrite(pwmPinP1, HOMING_SPEED_SLOW);  // start moving motor at specified homing speed again. This time we know we were off limit switch
  while (digitalRead(limitSwitchP1L) == 1);  // loop until limit switch is hit
  analogWrite(pwmPinP1, 0); // stop motor
  if (IS_DEBUG) Serial.println("Stopped P1 Motor. Resetting position to 0.");
  delay(500);  // wait for any decel
  posMotorP1 = 0; // reset motor position to 0
  MotorP1.write(0);   // reset encoder position to 0

  // calculate max ticks. We are homed, so starting at 0.
  SET_DIR_P1_RIGHT;
  if (IS_DEBUG) Serial.print("P1 Motor tracking right. HOMING_SPEED_FAST: ");
  if (IS_DEBUG) Serial.print(HOMING_SPEED_FAST);
  if (IS_DEBUG) Serial.println("");
  analogWrite(pwmPinP1, HOMING_SPEED_FAST);  // start moving right fast
  delay(HOMING_FAST_TIME);
  if (IS_DEBUG) Serial.print("P1 Motor tracking right until limit switch hit. HOMING_SPEED_SLOW: ");
  if (IS_DEBUG) Serial.print(HOMING_SPEED_SLOW);
  if (IS_DEBUG) Serial.println("");
  analogWrite(pwmPinP1, HOMING_SPEED_SLOW);  // slow down after HOMING_FAST_TIME time
  //Serial.print("P1 Motor going right at HOMING_SPEED ");
  //Serial.print(HOMING_SPEED);
  //Serial.println(". Waiting for pot left to stop motor at right end.");
  //while (analogRead(potPinP1) > 512); // wait for user to rotate to left half - this signifies the end value
  while (digitalRead(limitSwitchP1R) == 1); // loop until right limit switch is hit
  analogWrite(pwmPinP1, 0); // stop motor
  if (IS_DEBUG) Serial.println("Stopped P1 Motor. Setting max position to # total ticks - EDGE_OFFSET: ");
  delay(500);  // wait for any decel
  settingsPosMaxMotorP1 = MotorP1.read() - EDGE_OFFSET;   // save # ticks it took minus some edge padding
  settingsPosCenterMotorP1 = (settingsPosMaxMotorP1 + settingsPosMinMotorP1) / 2 ; // save our center position
  if (IS_DEBUG) Serial.print(settingsPosMaxMotorP1);
  if (IS_DEBUG) Serial.println("\nP1 Motor Homed.");
  isHomed = 1;
}

// This routine must be called quite periodically for good response!
// compare analog value of pot (0-1023)to min/max range of motor. This is our set point
void move() {
  posMotorP1 = MotorP1.read();  // get current motor encoder values to update posMotorP1
  potP1 = analogRead(potPinP1);   // read player 1 pot analog value - this is our set point
  potP1_smoothed = (potFilterParam * potP1) + ((1 - potFilterParam) * potP1_smoothed);
  setPointMotorP1 = map(potP1_smoothed, 0, 1023, settingsPosMinMotorP1, settingsPosMaxMotorP1);   // calculate where pot is wrt motor min and max
  if (IS_DEBUG) {
    if (millis() - update_time > 250) {
      update_time = millis();
      Serial.print("M1 state: ");
      if (stateM1 == STATE_M1_STOPPED) Serial.print("stopped. ");
      else if (stateM1 == STATE_M1_TRACKING) Serial.print("tracking. ");
      Serial.print(" M1 set point: ");
      Serial.print(setPointMotorP1);
      Serial.print(". M1 position: ");
      Serial.print(posMotorP1);
      Serial.print(". Smoothed pot val: ");
      Serial.print(potP1_smoothed);
      Serial.println("");
    }
  }

  // TODO: Acceleration is solved. But on stop, it should decelerate
  if (stateM1 == STATE_M1_STOPPED) {
    // set point needs to be > 1/2 length of dead zone. i.e., we track until we hit the set point exactly, and now to start moving again we need to be outside of dead zone range
    if (setPointMotorP1 - posMotorP1 < (-1*DEAD_ZONE/2)) {
      // need to move left from a stopped position. set our acceleration and direction
      SET_DIR_P1_LEFT;
      pwmSpeedMotorP1 = INITIAL_PWM;
      acceleration = ACCELERATION_INIT;
      analogWrite(pwmPinP1, pwmSpeedMotorP1);  // set initial speed to 50 and acceleration to 5. TRACKING will take care of adding 5 to acceleration at specified rate, and add to pwmSpeedMotorP1
      update_acceleration = millis();
      stateM1 = STATE_M1_TRACKING;
    } else if (setPointMotorP1 - posMotorP1 > (DEAD_ZONE/2)) {
      // need to move right from a stopped position. set our acceleration and direction
      SET_DIR_P1_RIGHT;
      pwmSpeedMotorP1 = INITIAL_PWM;
      acceleration = ACCELERATION_INIT;
      analogWrite(pwmPinP1, pwmSpeedMotorP1);  // set initial speed to 50 and acceleration to 5. TRACKING will take care of adding 5 to acceleration at specified rate, and add to pwmSpeedMotorP1
      update_acceleration = millis();
      stateM1 = STATE_M1_TRACKING;
    }
  } else if (stateM1 == STATE_M1_TRACKING) {
    // check if we need to go left, right, or stop
    if (setPointMotorP1 - posMotorP1 < (-1 * DEAD_ZONE/8)) {  // -1 because we are checking if we are within 1/4 dead zone on left
      // need to move left
      SET_DIR_P1_LEFT;
      // how far are we? If far, full acceleration as below. If "near", let's decelerate
      if (abs(setPointMotorP1 - posMotorP1) > DECEL_RANGE) {
        // we are far away...proceed with acceleration. but first check if it's time to accelerate
        if (millis() - update_acceleration > (1000/ACCELERATION_UPDATES_PER_SECOND)) {
          update_acceleration = millis();
          pwmSpeedMotorP1 += acceleration;
          if (pwmSpeedMotorP1 > 255) {
            pwmSpeedMotorP1 = 255;
            acceleration = DECELERATION_INIT;
          }
          acceleration += ACCELERATION_VAL;
          analogWrite(pwmPinP1, pwmSpeedMotorP1); // set to whatever our current pwm speed value is
        }
      } else {
        // we are near...proceed with deceleration, but first check if it's time to decelerate
        if (millis() - update_acceleration > (1000/ACCELERATION_UPDATES_PER_SECOND)) {
          update_acceleration = millis();
          pwmSpeedMotorP1 -= acceleration;
          if (pwmSpeedMotorP1 < HOMING_SPEED_SLOW) {
            pwmSpeedMotorP1 = HOMING_SPEED_SLOW;
            acceleration = 255;
          }
          acceleration -= DECELERATION_VAL;
          analogWrite(pwmPinP1, pwmSpeedMotorP1); // set to whatever our current pwm speed value is
        }
      }
    } else if (setPointMotorP1 - posMotorP1 > (DEAD_ZONE/8)) {
      // need to move right
      SET_DIR_P1_RIGHT;
      // check if it's time to accelerate
      if (millis() - update_acceleration > (1000/ACCELERATION_UPDATES_PER_SECOND)) {
        update_acceleration = millis();
        pwmSpeedMotorP1 += acceleration;
        if (pwmSpeedMotorP1 > 255) {
          pwmSpeedMotorP1 = 255;
          acceleration = 255;
        }
        acceleration += ACCELERATION_VAL;
        analogWrite(pwmPinP1, pwmSpeedMotorP1); // set to whatever our current pwm speed value is
      }
    } else {
      // we are within 1/4 of center of dead zone on both sides. Stop motor and move state to stopped
      analogWrite(pwmPinP1, 0);
      stateM1 = STATE_M1_STOPPED;
    }
  } 
}

void loop() {
  if (isHomed == 0) {
    delay(2000);
    Serial.println("");
    Serial.println("Connected to Dodgeball.\n");
    homePaddle();
    update_time = millis();
  }
  
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
  
