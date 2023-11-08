/*
  Dodgeball Motor Controller

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

  Notes:
  Using example Analog Serial Monitoring, pot currently reports 0 for full turned one way, 1023 for full turned other way. So use that as scale for
    PWM motor control
  PWM using pins (from https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm)
    The Arduino's programming language makes PWM easy to use; simply call analogWrite(pin, dutyCycle), where dutyCycle is a value from 0 to 255, and pin is one of the PWM pins (3, 5, 6, 9, 10, or 11).
    The analogWrite() function provides a simple interface to the hardware PWM, but doesn't provide any control over frequency.
*/

#include <Encoder.h>

Encoder MotorP1(0, 1);  // D0/D1 must be set to input
//Encoder MotorP2(2, 3);
long currPosMotorP1 = -999;
long currPosMotorP2 = -999;

int potP1Pin = A0;    // pot connected to A0 on LattePanda board (Arduino section)
int potP2Pin = A1;    // pot connected to A1 on LattePanda board (Arduino section)
int pwmP1 = 6;        // TODO: swap this and pwmP2. Want to be number consistent with D0,D1 being encoder signals for M1, etc. motor PWM signal for player 1 (connects to H-Bridge motor driver boards I bought)
int dirP1 = 12;        // direction signal for player 1 (connects to H-Bridge motor driver boards I bought)
int pwmP2 = 5;        // motor PWM signal for player 2 (connects to H-Bridge motor driver boards I bought)
int dirP2 = 0;        // direction signal for player 2 (connects to H-Bridge motor driver boards I bought)

int ledPin = 13;      // default red led. Will place blinks in certain parts of code

int potP1 = 0;        // store pot analog value for player 1's controller
int potP2 = 0;        // store pot analog value for player 2's controller
int speed = 40;

void setup() {
  pinMode(ledPin, OUTPUT);  // make LED output
  pinMode(0, INPUT);  // M1 Encoder A
  pinMode(1, INPUT);  // M1 Encoder B
  analogWrite(pwmP1, 0);

  Serial.begin(9600);       // initialize serial communication at 9600 bits per second
}

void loop() {
  long newPosMotorP1, newPosMotorP2;

  potP1 = analogRead(potP1Pin);   // read player 1 pot analog value  

  if (potP1 >= 492 and potP1 <= 532) {  // P1 analog stick near middle?
      analogWrite(pwmP1, 0);            // turn off P1 pwm 
  } else if (potP1 < 492) {             // pot is somewhere in negative region
    potP1 = map(potP1, 491, 0, 0, 255);     // scale it for use with PWM functions on pins 5 and 6.
    digitalWrite(dirP1, 0);             // moving left
    analogWrite(pwmP1, potP1);          // write scaled PWM based on how far we are from 491
  } else {  // pot is somewhere in positve region
    potP1 = map(potP1, 533, 1023, 0, 255);     // scale it for use with PWM functions on pins 5 and 6.
    digitalWrite(dirP1, 1);             // moving right
    analogWrite(pwmP1, potP1);          // write scaled PWM based on how far we are from 491
  }
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
  

  
  

  //Serial.println(val);
  //delay(1);  // delay in between reads for stability
