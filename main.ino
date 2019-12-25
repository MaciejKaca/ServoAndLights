#include <Arduino.h>
#include <Servo.h>

#include <interface.h>
#include "lightsAndServo.h"

Servo servo;
LightsAndServoMsg message;
TurnSignalCommand turnSignalCommmand;

void setup()
{
  Serial.begin(9600);

  pinMode(LEFT_BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(RIGHT_BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(REVERSE_LIGHT_PIN, OUTPUT);
  pinMode(LEFT_BLINKER_LIGHT_PIN, OUTPUT);
  pinMode(RIGHT_BLINKER_LIGHT_PIN, OUTPUT);
  pinMode(HEADLIGHT_PIN, OUTPUT);

  servo.attach(SERVO_PIN);

  digitalWrite(LEFT_BRAKE_LIGHT_PIN, LOW);
  digitalWrite(RIGHT_BRAKE_LIGHT_PIN, LOW);
  digitalWrite(REVERSE_LIGHT_PIN, LOW);
  digitalWrite(LEFT_BLINKER_LIGHT_PIN, LOW);
  digitalWrite(RIGHT_BLINKER_LIGHT_PIN, LOW);
  digitalWrite(HEADLIGHT_PIN, LOW);

  servo.write(CENTER_SERVO);
  turnSignalCommmand = TURN_SIGNAL_OFF;
}

void loop()
{
    if( isMessagePresent() )
    {
      receiveMessage(message);
      handleMessage(message, servo, turnSignalCommmand);
    }

    blinkWithoutDelay(turnSignalCommmand);
}
