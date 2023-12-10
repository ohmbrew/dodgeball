/*
  Dodgeball Motor Controller

  Arduino connected to SBC over USB - provides monitor
  I2C comms to SBC
  - Gonna use bitbang I2C on D20/D23 (SDA/SCL): https://www.arduino.cc/reference/en/libraries/bitbang_i2c/
  - Make a cable that connects to I2C on LattePanda to the I2C bus on SBC. I do not believe they're connected at this time
  - MPF can talk "native i2c", so code a custom python module for that
    - https://missionpinball.org/hardware/i2c_platforms/

  The circuit:

  LattePanda Arduino pinout

                 
    *     *     *     *   
   E2A   E2B   E1A   E1B   SR1   SL1  PWM1  POT1         SDA
   D0    D1    D2    D3    D4    D5    D6    A0    D19   D20    5V    5V
    |     |     |     |     |     |     |     |     |     |     |     | 

    |     |     |     |     |     |     |     |     |     |     |     |   
   D7    D8    D9    D10   D11   D12   D13   A3    D22   D23   GND   GND
  DIR1  DIR2        PWM2   SR2   SL2        POT2         SCL
    *

  * = digital interrupt capable. Prefer to put motor encoder signals on these pins. M1 is already good (connected to 2 and 3)
      but, M2 is currently wired to 9,10, which aren't interrupt capable. Looks like we'll have to break the wiring symmetry...

  Player 1 signals:
  POT1  [A0]: Pot P1
  SL1   [D5]: Limit Switch Left P1     * UPDATED - MUST REWIRE
  DIR1  [D1]: Motor Direction P1
  PWM1  [D6]: Motor PWM P1
  E1A   [D2]: Motor Encoder A P1
  E1B   [D3]: Motor Encoder B P1
  SR1   [D4]: Limit Switch Right P1
  
  Player 2 signals:
  POT2  [A3]: Pot P2
  SL2   [D12]: Limit Switch Left P2      * UPDATED - MUST REWIRE
  DIR2  [D8]: Motor Direction P2
  PWM2 [D10]: Motor PWM P2
  E2A   [D0]: Motor Encoder A P2        * UPDATED - MUST REWIRE
  E2B   [D7]: Motor Encoder B P2        * UPDATED - MUST REWIRE
  SR2  [D11]: Limit Switch Right P2


  Operation
  - potentiometer control for paddle left/right
    - Control paddle by using a pot that has left/right limits. Those limits equate to paddle far left/far right. As you move pot, it moves paddle.
    - center pin of the potentiometer to the analog input 0. One side pin (either one) to ground the other side pin to +5V
  - drive 2x pins (pwmPinP1 and pwmPinP2) with AnalogWrite (see below) to drive PWM signal to H-Bridge chip to motor
  - motor encoder signals (2 signals per motor = 4) going to inputs (M1=[D2,D3] and M2=[D0,D7]) used to calculate distance moved
  - regularly send paddle position info over i2c for MPF python module to listen to and send events accordingly
  - regulary receive info like setting paddle position in the case of AI player 2

  created by Greg Brault (Ohmbrew) for use in Dodge (pin) Ball game
  created 5 Nov 2023
  modified 16 Nov - tested out with belted drive system. Uploading to github
  modified 30 Nov - All physical wiring of P1 and P2 is done. Updating code to move/home paddles simultaneously
  modified 1 Dec - Code updated to home() and move() both paddles simultaneously in real time
  modified 3 Dec - Added info about how to communicate back to MPF. I'll use bit bang i2c, connected to SBC I2C. MPF can read that using "native i2c". Custom python module.
  modified 6 Dec - Ditched I2C. I just read from serial port which is hooked up to both...
  Notes:
  Using example Analog Serial Monitoring, pot currently reports 0 for full turned one way, 1023 for full turned other way. So use that as scale for
    PWM motor control
  PWM using pins (from https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm)
    The Arduino's programming language makes PWM easy to use; simply call analogWrite(pin, dutyCycle), where dutyCycle is a value from 0 to 255, and pin is one of the PWM pins (3, 5, 6, 9, 10, or 11).
    The analogWrite() function provides a simple interface to the hardware PWM, but doesn't provide any control over frequency.

  Motor color code:
  Red: Motor Power
  Black: Motor Power
  Green: Encoder GND
  Blue: Encoder VCC
  Yellow: Encoder A
  White: Encoder B
*/

#include <Encoder.h>
// #include <BitBang_I2C.h>

#define IS_DEBUG 0
#define SET_DIR_P1_LEFT digitalWrite(dirPinP1, LOW)
#define SET_DIR_P1_RIGHT digitalWrite(dirPinP1, HIGH)
#define SET_DIR_P2_LEFT digitalWrite(dirPinP2, LOW)
#define SET_DIR_P2_RIGHT digitalWrite(dirPinP2, HIGH)

// These values were all tested empirically and they work well together
#define UPDATE_SERIAL_TIME  1000  // how often (ms) to send serial port updates of status
#define DEAD_ZONE     400     // 17 Nov. 350 is lowest we can go with no minute oscillations (with .5 pot filter).
#define DECEL_RANGE   2000     // TODO: tune this empirically. Distance to start deceleration mapping. Otherwise, full speed ahead!
#define EDGE_OFFSET   150     // 17 Nov. This seems good.
#define BACKOFF_TIME  500     // 18 Nov. This is good
#define DECEL_TIME    500
#define HOMING_SPEED_SLOW 40  // 18 Nov this is good
#define HOMING_SPEED_FAST 200
#define HOMING_FAST_TIME  1600
#define INITIAL_PWM   40
#define ACCELERATION_INIT 5
#define DECELERATION_INIT 10
#define DECELERATION_VAL 1
#define ACCELERATION_VAL 4
#define ACCELERATION_UPDATES_PER_SECOND 30

// These states used by both paddles during homing and active playing. I think I have them all?
#define STATE_STOPPED             0   // stopped
#define STATE_HOMING_LEFT1        1   // moving left slow until limit switch hit
#define STATE_HOMING_RIGHT1       2   // moving right slow for BACKOFF_TIME
#define STATE_HOMING_LEFT2        3   // moving left slow until limit switch hit
#define STATE_HOMING_LEFT2_DECEL  4   // delaying for DECEL_TIME after left limit switch hit
#define STATE_HOMING_RIGHT2       5   // moving right fast for HOMING_FAST_TIME
#define STATE_HOMING_RIGHT3       6   // slow homing speed after moving right fast for HOMING_FAST_TIME
#define STATE_HOMING_RIGHT3_DECEL 7   // delaying for DECEL_TIME after right limit switch hit
#define STATE_HOMED               8

#define STATE_TRACKING            9   // paddle is moving toward position target

// Define as -1, -1 to use the Wire library over the default I2C interface
// #define SDA_PIN 20
// #define SCL_PIN 23
// #define BITBANG true

// Player 1 signal pins
int potPinP1 = A0;        // pot connected to A0 on LattePanda board (Arduino section)
int pwmPinP1 = 6;         // motor PWM (connects to H-Bridge motor driver boards I bought)
int dirPinP1 = 1;         // motor direction signal (connects to H-Bridge motor driver boards I bought)
int limitSwitchP1L = 5;   // limit switch left
int limitSwitchP1R = 4;   // limit switch right
int motorEncP1A = 2;      // motor encoder A
int motorEncP1B = 3;      // motor encoder B


// Player 2 signal pins
int potPinP2 = A3;        // pot connected to A3 on LattePanda board (Arduino section)
int pwmPinP2 = 10;        // motor PWM (connects to H-Bridge motor driver boards I bought). Moved to D10 since D13 was the onboard LED, fucked with motor PWM signal
int dirPinP2 = 8;         // motor direction signal (connects to H-Bridge motor driver boards I bought)
int limitSwitchP2L = 12;   // limit switch left
int limitSwitchP2R = 11;  // limit switch right
int motorEncP2A = 0;      // motor encoder A
int motorEncP2B = 7;      // motor encoder B

int ledPin = 13;          // default red led. Will place blinks in certain parts of code

float potFilterParam = 0.5; // used for both player pots

int potP1 = 0;          // player 1's pot A/D value (0-1023)
int potP1_smoothed = 0; // moving average low pass filter for PotP1
int stateP1 = STATE_STOPPED;

int potP2 = 0;          // player 2's pot A/D value (0-1023)
int potP2_smoothed = 0; // moving average low pass filter for PotP2
int stateP2 = STATE_STOPPED;

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
int isHomedP1 = 0;
int isHomedP2 = 0;

int is_acceleratingP1 = 0;  // used to set initial acceleration
int accelerationP1 = 0;     // set this to whatever min speed you want to start at. It will increase it each move() until 255.
int is_acceleratingP2 = 0;  // used to set initial acceleration
int accelerationP2 = 0;     // set this to whatever min speed you want to start at. It will increase it each move() until 255.

long update_serial = 0;
long update_timeP1 = 0;
long update_accelerationP1 = 0;
long update_timeP2 = 0;
long update_accelerationP2 = 0;

// i2c vars
// BBI2C     bbi2c;
// uint8_t   i2c_map[16];
// char      szTemp[32];
// uint8_t   i;
// int       iDevice, iCount;
// uint32_t  u32Caps;

void setup() {
  pinMode(ledPin, OUTPUT);        // make on-board LED output

  // Player 1 signals I/O configure
  pinMode(potPinP1, INPUT);       // Player 1 Pot Analog
  pinMode(limitSwitchP1L, INPUT_PULLUP);  // Player 1 Limit Switch Left input with pullup
  pinMode(limitSwitchP1R, INPUT_PULLUP);  // Player 1 Limit Switch Right input with pullup
  pinMode(dirPinP1, OUTPUT);      // Player 1 Motor Direction
  pinMode(motorEncP1A, INPUT);    // Player 1 Motor Encoder A
  pinMode(motorEncP1B, INPUT);    // Player 1 Motor Encoder B
  pinMode(pwmPinP1, OUTPUT);      // Player 1 Motor PWM set as output then drive low. Trying to prevent paddles smashing into sides on reset
  digitalWrite(pwmPinP1, LOW);    // 0 PWM - motor off
  analogWrite(pwmPinP1, 0);       // init Player 1 Motor PWM to 0

  // Player 2 signals I/O configure
  pinMode(potPinP2, INPUT);       // Player 2 Pot Analog
  pinMode(limitSwitchP2L, INPUT_PULLUP);  // Player 2 Limit Switch Left input with pullup
  pinMode(limitSwitchP2R, INPUT_PULLUP);  // Player 2 Limit Switch Right input with pullup
  pinMode(dirPinP2, OUTPUT);      // Player 2 Motor Direction
  pinMode(motorEncP2A, INPUT);    // Player 2 Motor Encoder A
  pinMode(motorEncP2B, INPUT);    // Player 2 Motor Encoder B
  pinMode(pwmPinP2, OUTPUT);      // Player 1 Motor PWM set as outpuet then drive low. Trying to prevent paddles smashing into sides on reset
  digitalWrite(pwmPinP2, 0);      // 0 PWM - motor off
  analogWrite(pwmPinP2, 0);       // init Player 2 Motor PWM to 0

  // memset(&bbi2c, 0, sizeof(bbi2c));
  // bbi2c.bWire = BITBANG;          // use bit bang, not wire library
  // bbi2c.iSDA = SDA_PIN;
  // bbi2c.iSCL = SCL_PIN;
  // I2CInit(&bbi2c, 100000L);       // SDA=pin 20, SCL=pin 23, 100K clock
  // delay(100);                     // allow devices to power up
  
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


void homePaddles() {
  // init our vars so we start in the right state
  isHomedP1 = 0;
  isHomedP2 = 0;
  stateP1 = STATE_STOPPED;
  stateP2 = STATE_STOPPED;
  Serial.print("X\n"); 
  // loop until both paddles are homed
  while (!isHomedP1 || !isHomedP2) {
    // handle P1 paddle
    switch (stateP1) {
    case STATE_STOPPED:
      if (IS_DEBUG) Serial.print("M1 tracking left until limit switch hit. HOMING_SPEED_SLOW: ");
      if (IS_DEBUG) Serial.print(HOMING_SPEED_SLOW);
      if (IS_DEBUG) Serial.println("");
      SET_DIR_P1_LEFT;
      analogWrite(pwmPinP1, HOMING_SPEED_SLOW);  // start moving motor left
      stateP1 = STATE_HOMING_LEFT1;
      break;
    case STATE_HOMING_LEFT1:
      // left limit switch hit yet?
      if (digitalRead(limitSwitchP1L) == 0) {
        analogWrite(pwmPinP1, 0);   // stop motor
        if (IS_DEBUG) Serial.println("Stopped M1. Backing it off.");
        SET_DIR_P1_RIGHT;
        analogWrite(pwmPinP1, HOMING_SPEED_SLOW);  // start moving right for BACKOFF_TIME. Want to back it off if limit switch was active when started
        update_timeP1 = millis();  // used to implement a delay
        stateP1 = STATE_HOMING_RIGHT1;
      }
      break;
    case STATE_HOMING_RIGHT1:
      // have we delayed BACKOFF_TIME moving right after initially hitting left limit switch?
      if (millis() - update_timeP1 > BACKOFF_TIME) {
        if (IS_DEBUG) Serial.print("M1 tracking left until limit switch hit. HOMING_SPEED_SLOW: ");
        if (IS_DEBUG) Serial.print(HOMING_SPEED_SLOW);
        if (IS_DEBUG) Serial.println("");
        SET_DIR_P1_LEFT;
        analogWrite(pwmPinP1, HOMING_SPEED_SLOW);  // start moving motor at specified homing speed again. This time we know we were off limit switch
        stateP1 = STATE_HOMING_LEFT2;
      }
      break;
    case STATE_HOMING_LEFT2:
      if (digitalRead(limitSwitchP1L) == 0) {
        analogWrite(pwmPinP1, 0); // stop motor
        if (IS_DEBUG) Serial.println("Stopped M1. Resetting position to 0.");
        update_timeP1 = millis();  // used to implement a delay
        stateP1 = STATE_HOMING_LEFT2_DECEL;
      }
      break;
    case STATE_HOMING_LEFT2_DECEL:
      if (millis() - update_timeP1 > DECEL_TIME) {
        posMotorP1 = 0; // reset motor position to 0
        MotorP1.write(0);   // reset encoder position to 0
        // We are homed far left. Calculate max ticks to right limit switch. We are starting at 0.
        if (IS_DEBUG) Serial.print("M1 tracking right. HOMING_SPEED_FAST: ");
        if (IS_DEBUG) Serial.print(HOMING_SPEED_FAST);
        if (IS_DEBUG) Serial.println("");
        SET_DIR_P1_RIGHT;
        analogWrite(pwmPinP1, HOMING_SPEED_FAST);  // start moving right fast
        update_timeP1 = millis();  // used to implement delay
        stateP1 = STATE_HOMING_RIGHT2;
      }
      break;
    case STATE_HOMING_RIGHT2:
      if (millis() - update_timeP1 > HOMING_FAST_TIME) {
        analogWrite(pwmPinP1, HOMING_SPEED_SLOW);  // slow down after HOMING_FAST_TIME time
        if (IS_DEBUG) Serial.print("M1 tracking right until limit switch hit. HOMING_SPEED_SLOW: ");
        if (IS_DEBUG) Serial.print(HOMING_SPEED_SLOW);
        if (IS_DEBUG) Serial.println("");
        stateP1 = STATE_HOMING_RIGHT3;
      }
      break;
    case STATE_HOMING_RIGHT3:
      if (digitalRead(limitSwitchP1R) == 0) {
        analogWrite(pwmPinP1, 0); // stop motor
        if (IS_DEBUG) Serial.print("Stopped M1. Setting max position to # total ticks - EDGE_OFFSET: ");
        update_timeP1 = millis();  // used to implement a delay
        stateP1 = STATE_HOMING_RIGHT3_DECEL;
      }
      break;
    case STATE_HOMING_RIGHT3_DECEL:
      if (millis() - update_timeP1 > DECEL_TIME) {
        settingsPosMaxMotorP1 = MotorP1.read() - EDGE_OFFSET;   // save # ticks it took minus some edge padding
        settingsPosCenterMotorP1 = (settingsPosMinMotorP1 + settingsPosMaxMotorP1) / 2 ; // save our center position
        if (IS_DEBUG) Serial.print(settingsPosMaxMotorP1);
        if (IS_DEBUG) Serial.println("\nM1Homed.");
        isHomedP1 = 1;
        stateP1 = STATE_HOMED;
      }
    default:
      break;
    }

    // handle P2 paddle
    switch (stateP2) {
    case STATE_STOPPED:
      if (IS_DEBUG) Serial.print("M2 tracking left until limit switch hit. HOMING_SPEED_SLOW: ");
      if (IS_DEBUG) Serial.print(HOMING_SPEED_SLOW);
      if (IS_DEBUG) Serial.println("");
      SET_DIR_P2_LEFT;
      analogWrite(pwmPinP2, HOMING_SPEED_SLOW);  // start moving motor left
      stateP2 = STATE_HOMING_LEFT1;
      break;
    case STATE_HOMING_LEFT1:
      // left limit switch hit yet?
      if (digitalRead(limitSwitchP2L) == 0) {
        analogWrite(pwmPinP2, 0);   // stop motor
        if (IS_DEBUG) Serial.println("Stopped M2. Backing it off.");
        SET_DIR_P2_RIGHT;
        analogWrite(pwmPinP2, HOMING_SPEED_SLOW);  // start moving right for BACKOFF_TIME. Want to back it off if limit switch was active when started
        update_timeP2 = millis();  // used to implement a delay
        stateP2 = STATE_HOMING_RIGHT1;
      }
      break;
    case STATE_HOMING_RIGHT1:
      // have we delayed BACKOFF_TIME moving right after initially hitting left limit switch?
      if (millis() - update_timeP2 > BACKOFF_TIME) {
        if (IS_DEBUG) Serial.print("M2 tracking left until limit switch hit. HOMING_SPEED_SLOW: ");
        if (IS_DEBUG) Serial.print(HOMING_SPEED_SLOW);
        if (IS_DEBUG) Serial.println("");
        SET_DIR_P2_LEFT;
        analogWrite(pwmPinP2, HOMING_SPEED_SLOW);  // start moving motor at specified homing speed again. This time we know we were off limit switch
        stateP2 = STATE_HOMING_LEFT2;
      }
      break;
    case STATE_HOMING_LEFT2:
      if (digitalRead(limitSwitchP2L) == 0) {
        analogWrite(pwmPinP2, 0); // stop motor
        if (IS_DEBUG) Serial.println("Stopped M2. Resetting position to 0.");
        update_timeP2 = millis();  // used to implement a delay
        stateP2 = STATE_HOMING_LEFT2_DECEL;
      }
      break;
    case STATE_HOMING_LEFT2_DECEL:
      if (millis() - update_timeP2 > DECEL_TIME) {
        posMotorP2 = 0; // reset motor position to 0
        MotorP2.write(0);   // reset encoder position to 0
        // We are homed far left. Calculate max ticks to right limit switch. We are starting at 0.
        if (IS_DEBUG) Serial.print("M2 tracking right. HOMING_SPEED_FAST: ");
        if (IS_DEBUG) Serial.print(HOMING_SPEED_FAST);
        if (IS_DEBUG) Serial.println("");
        SET_DIR_P2_RIGHT;
        analogWrite(pwmPinP2, HOMING_SPEED_FAST);  // start moving right fast
        update_timeP2 = millis();  // used to implement delay
        stateP2 = STATE_HOMING_RIGHT2;
      }
      break;
    case STATE_HOMING_RIGHT2:
      if (millis() - update_timeP2 > HOMING_FAST_TIME) {
        analogWrite(pwmPinP2, HOMING_SPEED_SLOW);  // slow down after HOMING_FAST_TIME time
        if (IS_DEBUG) Serial.print("M2 tracking right until limit switch hit. HOMING_SPEED_SLOW: ");
        if (IS_DEBUG) Serial.print(HOMING_SPEED_SLOW);
        if (IS_DEBUG) Serial.println("");
        stateP2 = STATE_HOMING_RIGHT3;
      }
      break;
    case STATE_HOMING_RIGHT3:
      if (digitalRead(limitSwitchP2R) == 0) {
        analogWrite(pwmPinP2, 0); // stop motor
        if (IS_DEBUG) Serial.print("Stopped M2. Setting max position to # total ticks - EDGE_OFFSET: ");
        update_timeP2 = millis();  // used to implement a delay
        stateP2 = STATE_HOMING_RIGHT3_DECEL;
      }
      break;
    case STATE_HOMING_RIGHT3_DECEL:
      if (millis() - update_timeP2 > DECEL_TIME) {
        settingsPosMaxMotorP2 = MotorP2.read() - EDGE_OFFSET;   // save # ticks it took minus some edge padding
        settingsPosCenterMotorP2 = (settingsPosMinMotorP2 + settingsPosMaxMotorP2) / 2 ; // save our center position
        if (IS_DEBUG) Serial.print(settingsPosMaxMotorP2);
        if (IS_DEBUG) Serial.println("\nM2Homed.");
        isHomedP2 = 1;
        stateP2 = STATE_HOMED;
        Serial.print("H\n"); 
      }
    default:
      break;
    }
  }
  stateP1 = STATE_STOPPED;
  stateP2 = STATE_STOPPED;
  //update_serial = millis();
}

// This routine must be called quite periodically for good response!
// compare analog value of pot (0-1023) to min/max range of motor. This is our set point
void move() {
  posMotorP1 = MotorP1.read();  // get current motor encoder values to update posMotorP1
  potP1 = analogRead(potPinP1);   // read player 1 pot analog value - this is our set point
  potP1_smoothed = (potFilterParam * potP1) + ((1 - potFilterParam) * potP1_smoothed);
  setPointMotorP1 = map(potP1_smoothed, 0, 1023, settingsPosMinMotorP1, settingsPosMaxMotorP1);   // calculate where pot is wrt motor min and max

  posMotorP2 = MotorP2.read();  // get current motor encoder values to update posMotorP2
  potP2 = analogRead(potPinP2);   // read player 2 pot analog value - this is our set point
  potP2_smoothed = (potFilterParam * potP2) + ((1 - potFilterParam) * potP2_smoothed);
  setPointMotorP2 = map(potP2_smoothed, 0, 1023, settingsPosMinMotorP2, settingsPosMaxMotorP2);   // calculate where pot is wrt motor min and max
  
  
  if (millis() - update_serial > UPDATE_SERIAL_TIME) {
    update_serial = millis();
    if (stateP1 == STATE_STOPPED) Serial.print("P,S,");       // P = Paddle Update
    else if (stateP1 == STATE_TRACKING) Serial.print("P,T,");
    Serial.print(setPointMotorP1);
    Serial.print(",");
    Serial.print(posMotorP1);
    Serial.print(",");
    if (stateP2 == STATE_STOPPED) Serial.print("S,");
    else if (stateP2 == STATE_TRACKING) Serial.print("T,");
    Serial.print(setPointMotorP2);
    Serial.print(",");
    Serial.print(posMotorP2);
    Serial.print("\n");
  }
  

  // move M1 with acceleration
  if (stateP1 == STATE_STOPPED) {
    // set point needs to be > 1/2 length of dead zone. i.e., we track until we hit the set point exactly, and now to start moving again we need to be outside of dead zone range
    if (setPointMotorP1 - posMotorP1 < (-1*DEAD_ZONE/2)) {
      // need to move left from a stopped position. set our acceleration and direction
      SET_DIR_P1_LEFT;
      pwmSpeedMotorP1 = INITIAL_PWM;
      accelerationP1 = ACCELERATION_INIT;
      analogWrite(pwmPinP1, pwmSpeedMotorP1);  // set initial speed and acceleration. TRACKING will take care of adding to acceleration at specified rate, and add to pwmSpeedMotorP1
      update_accelerationP1 = millis();
      stateP1 = STATE_TRACKING;
    } else if (setPointMotorP1 - posMotorP1 > (DEAD_ZONE/2)) {
      // need to move right from a stopped position. set our acceleration and direction
      SET_DIR_P1_RIGHT;
      pwmSpeedMotorP1 = INITIAL_PWM;
      accelerationP1 = ACCELERATION_INIT;
      analogWrite(pwmPinP1, pwmSpeedMotorP1);  // set initial speed to 50 and acceleration to 5. TRACKING will take care of adding 5 to acceleration at specified rate, and add to pwmSpeedMotorP1
      update_accelerationP1 = millis();
      stateP1 = STATE_TRACKING;
    } else {
      analogWrite(pwmPinP1, 0); // we are in a stopped state, so make sure we are stopped! I still hear micro adjustments even with this algorithm. Which, shouldn't happen...
    }
  } else if (stateP1 == STATE_TRACKING) {
    // check if we need to go left, right, or stop
    if (setPointMotorP1 - posMotorP1 < (-1 * DEAD_ZONE/8)) {  // -1 because we are checking if we are within 1/4 dead zone on left
      // need to move left
      SET_DIR_P1_LEFT;
      // how far are we? If far, full acceleration as below. If "near", let's decelerate
      if (abs(setPointMotorP1 - posMotorP1) > DECEL_RANGE) {
        // we are far away...proceed with acceleration. but first check if it's time to accelerate
        if (millis() - update_accelerationP1 > (1000/ACCELERATION_UPDATES_PER_SECOND)) {
          update_accelerationP1 = millis();
          pwmSpeedMotorP1 += accelerationP1;
          if (pwmSpeedMotorP1 > 255) {
            pwmSpeedMotorP1 = 255;
            accelerationP1 = DECELERATION_INIT;
          }
          accelerationP1 += ACCELERATION_VAL;
          analogWrite(pwmPinP1, pwmSpeedMotorP1); // set to whatever our current pwm speed value is
        }
      } else {
        // we are near...proceed with deceleration, but first check if it's time to decelerate
        if (millis() - update_accelerationP1 > (1000/ACCELERATION_UPDATES_PER_SECOND)) {
          update_accelerationP1 = millis();
          pwmSpeedMotorP1 -= accelerationP1;
          if (pwmSpeedMotorP1 < HOMING_SPEED_SLOW) {
            pwmSpeedMotorP1 = HOMING_SPEED_SLOW;
            accelerationP1 = 255;
          }
          accelerationP1 -= DECELERATION_VAL;
          analogWrite(pwmPinP1, pwmSpeedMotorP1); // set to whatever our current pwm speed value is
        }
      }
    } else if (setPointMotorP1 - posMotorP1 > (DEAD_ZONE/8)) {
      // need to move right
      SET_DIR_P1_RIGHT;
      // check if it's time to accelerate
      if (millis() - update_accelerationP1 > (1000/ACCELERATION_UPDATES_PER_SECOND)) {
        update_accelerationP1 = millis();
        pwmSpeedMotorP1 += accelerationP1;
        if (pwmSpeedMotorP1 > 255) {
          pwmSpeedMotorP1 = 255;
          accelerationP1 = 255;
        }
        accelerationP1 += ACCELERATION_VAL;
        analogWrite(pwmPinP1, pwmSpeedMotorP1); // set to whatever our current pwm speed value is
      }
    } else {
      // we are within 1/4 of center of dead zone on both sides. Stop motor and move state to stopped
      analogWrite(pwmPinP1, 0);
      stateP1 = STATE_STOPPED;
    }
  } 

  // move M2 with acceleration
  if (stateP2 == STATE_STOPPED) {
    // set point needs to be > 1/2 length of dead zone. i.e., we track until we hit the set point exactly, and now to start moving again we need to be outside of dead zone range
    if (setPointMotorP2 - posMotorP2 < (-1*DEAD_ZONE/2)) {
      // need to move left from a stopped position. set our acceleration and direction
      SET_DIR_P2_LEFT;
      pwmSpeedMotorP2 = INITIAL_PWM;
      accelerationP2 = ACCELERATION_INIT;
      analogWrite(pwmPinP2, pwmSpeedMotorP2);  // set initial speed and acceleration. TRACKING will take care of adding to acceleration at specified rate, and add to pwmSpeedMotorP1
      update_accelerationP2 = millis();
      stateP2 = STATE_TRACKING;
    } else if (setPointMotorP2 - posMotorP2 > (DEAD_ZONE/2)) {
      // need to move right from a stopped position. set our acceleration and direction
      SET_DIR_P2_RIGHT;
      pwmSpeedMotorP2 = INITIAL_PWM;
      accelerationP2 = ACCELERATION_INIT;
      analogWrite(pwmPinP2, pwmSpeedMotorP2);  // set initial speed to 50 and acceleration to 5. TRACKING will take care of adding 5 to acceleration at specified rate, and add to pwmSpeedMotorP1
      update_accelerationP2 = millis();
      stateP2 = STATE_TRACKING;
    } else {
      analogWrite(pwmPinP2, 0); // we are in a stopped state, so make sure we are stopped! I still hear micro adjustments even with this algorithm. Which, shouldn't happen...
    }
  } else if (stateP2 == STATE_TRACKING) {
    // check if we need to go left, right, or stop
    if (setPointMotorP2 - posMotorP2 < (-1 * DEAD_ZONE/8)) {  // -1 because we are checking if we are within 1/4 dead zone on left
      // need to move left
      SET_DIR_P2_LEFT;
      // how far are we? If far, full acceleration as below. If "near", let's decelerate
      if (abs(setPointMotorP2 - posMotorP2) > DECEL_RANGE) {
        // we are far away...proceed with acceleration. but first check if it's time to accelerate
        if (millis() - update_accelerationP2 > (1000/ACCELERATION_UPDATES_PER_SECOND)) {
          update_accelerationP2 = millis();
          pwmSpeedMotorP2 += accelerationP2;
          if (pwmSpeedMotorP2 > 255) {
            pwmSpeedMotorP2 = 255;
            accelerationP2 = DECELERATION_INIT;
          }
          accelerationP2 += ACCELERATION_VAL;
          analogWrite(pwmPinP2, pwmSpeedMotorP2); // set to whatever our current pwm speed value is
        }
      } else {
        // we are near...proceed with deceleration, but first check if it's time to decelerate
        if (millis() - update_accelerationP2 > (1000/ACCELERATION_UPDATES_PER_SECOND)) {
          update_accelerationP2 = millis();
          pwmSpeedMotorP2 -= accelerationP2;
          if (pwmSpeedMotorP2 < HOMING_SPEED_SLOW) {
            pwmSpeedMotorP2 = HOMING_SPEED_SLOW;
            accelerationP2 = 255;
          }
          accelerationP2 -= DECELERATION_VAL;
          analogWrite(pwmPinP2, pwmSpeedMotorP2); // set to whatever our current pwm speed value is
        }
      }
    } else if (setPointMotorP2 - posMotorP2 > (DEAD_ZONE/8)) {
      // need to move right
      SET_DIR_P2_RIGHT;
      // check if it's time to accelerate
      if (millis() - update_accelerationP2 > (1000/ACCELERATION_UPDATES_PER_SECOND)) {
        update_accelerationP2 = millis();
        pwmSpeedMotorP2 += accelerationP2;
        if (pwmSpeedMotorP2 > 255) {
          pwmSpeedMotorP2 = 255;
          accelerationP2 = 255;
        }
        accelerationP2 += ACCELERATION_VAL;
        analogWrite(pwmPinP2, pwmSpeedMotorP2); // set to whatever our current pwm speed value is
      }
    } else {
      // we are within 1/4 of center of dead zone on both sides. Stop motor and move state to stopped
      analogWrite(pwmPinP2, 0);
      stateP2 = STATE_STOPPED;
    }
  } 
}

// sends paddle position data to SBC over i2c
//void sendUpdate() {
  
//}

void loop() {
  if (isHomedP1 == 0) {
    //delay(2000);
    //Serial.println("");
    //Serial.println("Connected to Dodgeball.\nScanning for I2C devices...");

    // scan for I2C devices. Looking for SBC
    // I2CScan(&bbi2c, i2c_map); // get bitmap of connected I2C devices
    // if (i2c_map[0] == 0xfe) {
    //   // something is wrong with the I2C bus
    //   Serial.println("I2C pins are not correct or the bus is being pulled low by a bad device; unable to run scan.");
    // } else {
    //   iCount = 0;
    //   for (i = 1; i < 128; i++) // skip address 0 (general call address) since more than 1 device can respond
    //   {
    //     if (i2c_map[i>>3] & (1 << (i & 7))) {
    //       // device found, print info about it
    //       iCount++;
    //       Serial.print("Device found at 0x");
    //       Serial.print(i, HEX);
    //       iDevice = I2CDiscoverDevice(&bbi2c, i, &u32Caps);
    //       Serial.print(", type = ");
    //       I2CGetDeviceName(iDevice, szTemp);
    //       Serial.print(szTemp); // show the device name as a string
    //       Serial.print(", capability bits = 0x");
    //       Serial.println(u32Caps, HEX);
    //     }
    //   }
    //   Serial.print(iCount, DEC);
    //   Serial.println(" device(s) found");
    // }
  
    homePaddles();
    update_timeP1 = millis();
    update_timeP2 = millis();
  }
  
  move(); // once we've homed, this is just called repeatedly. Both paddles will determine how to move based on set points and current positions
}
  
