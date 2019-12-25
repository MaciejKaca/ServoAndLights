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
    if(ledPin != SERVO_PIN &&  OFF <= brightness && brightness <= MAX_LIGHT_BRIGHTNESS)
    {
        analogWrite(ledPin, brightness);
    }
}

void blinkWithoutDelay(BlinkerLights &lights)
{
  unsigned long previousTime = 0;

  while(true)
  {
    unsigned long currentTime = millis();
    
    if (currentTime - previousTime >= BLINKER_LIGHTS_FREQUENCY)
    {
        previousTime = currentTime;

        switch (lights)
        {
        case LEFT:
            setLightBrigtness(RIGHT_BLINKER_LIGHT_PIN, OFF);
            toggleLED(LEFT_BLINKER_LIGHT_PIN);
            break;
        
        case RIGHT:
            setLightBrigtness(LEFT_BLINKER_LIGHT_PIN, OFF);
            toggleLED(RIGHT_BLINKER_LIGHT_PIN);
            break;
        
        case BOTH:
            toggleLED(LEFT_BLINKER_LIGHT_PIN);
            toggleLED(RIGHT_BLINKER_LIGHT_PIN);
            break;
        
        case NONE:
            setLightBrigtness(RIGHT_BLINKER_LIGHT_PIN, OFF);
            setLightBrigtness(LEFT_BLINKER_LIGHT_PIN, OFF);
        
        default:
            break;
        }
    }
  }
}

int turnServo(Servo &servo, int8_t degree)
{
    int8_t degreeToTurn = CENTER_SERVO;

    if(degree < 0)
    {
        if( (CENTER_SERVO + degree) >= LEFT_WHEEL_MAX )
        {
            degreeToTurn = (CENTER_SERVO + degree);
        }
        else
        {
            degreeToTurn = (LEFT_WHEEL_MAX);
        }
    }
    else
    {
        if( (CENTER_SERVO + degree) <= RIGHT_WHEEL_MAX )
        {
            degreeToTurn = (CENTER_SERVO + degree);
        }
        else
        {
            degreeToTurn = (RIGHT_WHEEL_MAX);
        }
    }
    
    servo.write(degreeToTurn);
    return degreeToTurn;
}

void centerServo(Servo &servo, int8_t degree)
{
    CENTER_SERVO = turnServo(servo, degree);
}

void handleMessage(LightsAndServoMsg message, Servo &servo, BlinkerLights &blinkerLights)
{
    switch (message.device)
    {
    case BRAKE_LIGHT:
        setLightBrigtness(LEFT_BRAKE_LIGHT_PIN, message.command);
        setLightBrigtness(RIGHT_BRAKE_LIGHT_PIN, message.command);
        break;
    
    case REVERSE_LIGHT:
        setLightBrigtness(REVERSE_LIGHT_PIN, message.command);
        break;

    case LEFT_BLINKER_LIGHT:
        if(message.command == ON)
        {
            blinkerLights = LEFT;
        }
        else if(message.command == OFF)
        {
            blinkerLights = NONE;
        }
        break;

    case RIGHT_BLINKER_LIGHT:
        if(message.command == ON)
        {
            blinkerLights = RIGHT;
        }
        else if(message.command == OFF)
        {
            blinkerLights = NONE;
        }
        break;
    
    case HAZARD_LIGHTS:
        if(message.command == ON)
        {
            blinkerLights = BOTH;
        }
        else if(message.command == OFF)
        {
            blinkerLights = NONE;
        }
        break;

    case HEADLIGHT:
        setLightBrigtness(HEADLIGHT_PIN, message.command);
        break;

    case SERVO:
        if(message.command == TURN)
        {
            turnServo(servo, message.degrees);
        }
        else if(message.command == CENTER_SERVO)
        {
            centerServo(servo, message.degrees);
        }
        break;

    default:
        break;
    }
}