#pragma once

#include <stdint.h>
#include <Servo.h>

#include <interface.h>

enum DevicesPins
{
    LEFT_BRAKE_LIGHT_PIN = 6,
    RIGHT_BRAKE_LIGHT_PIN = 5,
    REVERSE_LIGHT_PIN = 11,
    LEFT_BLINKER_LIGHT_PIN = 10,
    RIGHT_BLINKER_LIGHT_PIN = 9,
    HEADLIGHT_PIN = 3,
    SERVO_PIN = 2,
};

const uint8_t MAX_LIGHT_BRIGHTNESS = 255;
const uint8_t MIN_LIGHT_BRIGHTNESS = 0;
const uint16_t BLINKER_LIGHTS_FREQUENCY = 500; //milliseconds
const uint8_t MAX_WHEEL_TURN_DEGREE = 40;
const uint8_t LEFT_WHEEL_MAX = 50;
const uint8_t RIGHT_WHEEL_MAX = 135;
static uint8_t CENTER_SERVO = 95;

bool isMessagePresent();

void handleMessage(LightsAndServoMsg message, Servo &servo, TurnSignalCommand &turnSignalCommand);

void receiveMessage(LightsAndServoMsg &message);

void blinkWithoutDelay(TurnSignalCommand &lights);