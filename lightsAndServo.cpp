#include <Arduino.h>
#include <stdlib.h>
#include <Servo.h>

#include "lightsAndServo.h"

bool isMessagePresent()
{
    if(Serial.available() )
    {
        return true;
    }
    return false;
}

void receiveMessage(LightsAndServoMsg &message)
{
    //Serial.readBytes((char *) &message, sizeof(LightsAndServoMsg));
}

void toggleLED(DevicesPins ledPin)
{
  boolean ledstate = digitalRead(ledPin); // get LED state
  ledstate ^= 1;   // toggle LED state using xor
  digitalWrite(ledPin, ledstate); // write inversed state back
}

void setLightBrigtness(DevicesPins ledPin, uint8_t brightness)
{
    if(ledPin != SERVO_PIN && brightness >= MIN_LIGHT_BRIGHTNESS && brightness <= MAX_LIGHT_BRIGHTNESS)
    {
        analogWrite(ledPin, brightness);
    }
}

void blinkWithoutDelay(TurnSignalCommand &turnSingalLightCommand)
{
  unsigned long previousTime = 0;

  while(true)
  {
    unsigned long currentTime = millis();
    
    if (currentTime - previousTime >= BLINKER_LIGHTS_FREQUENCY)
    {
        previousTime = currentTime;

        switch (turnSingalLightCommand)
        {
            case LEFT:
                setLightBrigtness(RIGHT_BLINKER_LIGHT_PIN, TURN_SIGNAL_OFF);
                toggleLED(LEFT_BLINKER_LIGHT_PIN);
                break;
            
            case RIGHT:
                setLightBrigtness(LEFT_BLINKER_LIGHT_PIN, TURN_SIGNAL_OFF);
                toggleLED(RIGHT_BLINKER_LIGHT_PIN);
                break;
            
            case HAZARD_LIGHTS:
                toggleLED(LEFT_BLINKER_LIGHT_PIN);
                toggleLED(RIGHT_BLINKER_LIGHT_PIN);
                break;
            
            case TURN_SIGNAL_OFF:
                setLightBrigtness(RIGHT_BLINKER_LIGHT_PIN, TURN_SIGNAL_OFF);
                setLightBrigtness(LEFT_BLINKER_LIGHT_PIN, TURN_SIGNAL_OFF);
            
            default:
                break;
        }
    }
  }
}

uint8_t turnServo(Servo &servo, int8_t degree)
{
    uint8_t degreeToTurn = CENTER_SERVO;

    if(0 > degree)
    {
        if( (CENTER_SERVO + degree) >= LEFT_WHEEL_MAX )
        {
            degreeToTurn = (CENTER_SERVO + degree);
        }
        else
        {
            degreeToTurn = LEFT_WHEEL_MAX;
        }
    }
    else
    {
        if( (CENTER_SERVO + (uint8_t)degree) <= RIGHT_WHEEL_MAX )
        {
            degreeToTurn = (CENTER_SERVO + (uint8_t)degree);
        }
        else
        {
            degreeToTurn = RIGHT_WHEEL_MAX;
        }
    }
    
    servo.write(degreeToTurn);
    return degreeToTurn;
}

void centerServo(Servo &servo, int8_t degree)
{
    CENTER_SERVO = turnServo(servo, degree);
}

void handleMessage(LightsAndServoMsg message, Servo &servo, TurnSignalCommand &turnSignalCommand)
{
    switch (message.device)
    {
        case BRAKE_LIGHT:
            setLightBrigtness(LEFT_BRAKE_LIGHT_PIN, message.brakeLightsCommand);
            setLightBrigtness(RIGHT_BRAKE_LIGHT_PIN, message.brakeLightsCommand);
            break;

        case REVERSE_LIGHT:
            setLightBrigtness(REVERSE_LIGHT_PIN, message.reverseLightCommand);
            break;

        case TURN_SIGNAL:
            if(message.turnSignalCommand == HAZARD_LIGHTS)
            {
                setLightBrigtness(LEFT_BLINKER_LIGHT_PIN, TURN_SIGNAL_OFF);
                setLightBrigtness(LEFT_BLINKER_LIGHT_PIN, TURN_SIGNAL_OFF);
            }
            turnSignalCommand = message.turnSignalCommand;
            break;

        case HEADLIGHT:
            setLightBrigtness(HEADLIGHT_PIN, message.headLightCommand);

        case SERVO:
            if(message.servoInfo.command == TURN)
            {
                turnServo(servo, message.servoInfo.degrees);
            }
            else if(message.servoInfo.command == SET_NEW_ZERO)
            {
                centerServo(servo, message.servoInfo.degrees);
            }
            break;

        default:
            break;
    }
}