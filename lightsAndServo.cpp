#include "lightsAndServo.h"

#include <pt.h>
#include <Arduino.h>
#include <stdlib.h>

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
     Serial.readBytes((char*)&message, sizeof(LightsAndServoMsg));
}

void toggleLED(Devices LED_PIN)
{
  boolean ledstate = digitalRead(LED_PIN); // get LED state
  ledstate ^= 1;   // toggle LED state using xor
  digitalWrite(LED_PIN, ledstate); // write inversed state back
}

void setLightBrigtness(DevicesPins ledPin, uint8_t brightness)
{
    if(ledPin != SERVO_PIN && brightness >= OFF && brightness <= MAX_LIGHT_BRIGHTNESS)
    {
        analogWrite(ledPin, brightness);
    }
}

static int blinkProtothread(struct pt *pt, BlinkerLights &light)
{
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(true)
  { // never stop 
    /* each time the function is called the second boolean
    *  argument "millis() - timestamp > BLINKER_LIGHTS_FREQUENCY" is re-evaluated
    *  and if false the function exits after that. */
    PT_WAIT_UNTIL(pt, millis() - timestamp > BLINKER_LIGHTS_FREQUENCY );
    timestamp = millis(); // take a new timestamp
    
    switch (light)
    {
    case LEFT:
        toggleLED(LEFT_BLINKER_LIGHT);
        break;
    
    case RIGHT:
        toggleLED(RIGHT_BLINKER_LIGHT);
        break;
    
    case BOTH:
        toggleLED(LEFT_BLINKER_LIGHT);
        toggleLED(RIGHT_BLINKER_LIGHT);
        break;
    
    default:
        break;
    }
  }
  PT_END(pt);
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
            degreeToTurn = (RIGHT_WHEEL_MAX);
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
            degreeToTurn = (LEFT_WHEEL_MAX);
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
    
    case LEFT_BLINKER_LIGHT:
        if(message.command == ON)
        {
            blinkerLights = LEFT;
        }
        else if(message.command == OFF)
        {
            blinkerLights = NONE;
        }

    case RIGHT_BLINKER_LIGHT:
        if(message.command == ON)
        {
            blinkerLights = RIGHT;
        }
        else if(message.command == OFF)
        {
            blinkerLights = NONE;
        }
    
    case HAZARD_LIGHTS:
        if(message.command == ON)
        {
            blinkerLights = BOTH;
        }
        else if(message.command == OFF)
        {
            blinkerLights = NONE;
        }

    case SERVO:
        if(message.command == TURN)
        {
            turnServo(servo, message.degrees);
        }
        else if(message.command == CENTER_SERVO)
        {
            centerServo(servo, message.degrees);
        }

    default:
        break;
    }
}