#ifndef __PENUMBRA_VARIABLE_SETTINGS_H__
#define __PENUMBRA_VARIABLE_SETTINGS_H__

// Define additional settings based on the drive system of choice.

#if DRIVE_SYSTEM == DRIVE_SYSTEM_SABER
  #if DOME_DRIVE == DOME_DRIVE_SABER
    #define DOME_DRIVE_SERIAL    Serial1
  #endif
  #define CHANNEL_MIXING       true   // set to true premix channels before sending commands
  #define DRIVE_BAUD_RATE      9600
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_PWM
  #define NEED_DRIVE_PWM_PINS
  #define CHANNEL_MIXING       false  // set to false if motor controller will mix the channels
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM
  #define NEED_DRIVE_PWM_PINS
  #define CHANNEL_MIXING       false  // set to false if motor controller will mix the channels
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_SERIAL
  #define DRIVE_BAUD_RATE      115200
  #define CHANNEL_MIXING       false  // set to false if motor controller will mix the channels
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL
  #define NEED_DRIVE_PWM_PINS
  #define DRIVE_BAUD_RATE      115200
  #define CHANNEL_MIXING       false  // set to false if motor controller will mix the channels
#else
  #error Unsupported DRIVE_SYSTEM
#endif

// Set the dome serial address when using Syren10 for
// the dome but not using Sabertooth for the drive.

#if DOME_DRIVE == DOME_DRIVE_SABER && !defined(DOME_DRIVE_SERIAL)
  #define DOME_DRIVE_SERIAL    Serial2
#endif

// Create some helper definitions.

#if DRIVE_SYSTEM == DRIVE_SYSTEM_PWM || \
    DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM || \
    DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL
  #define NEED_DRIVE_PWM_PINS
#endif

#if DOME_DRIVE == DOME_DRIVE_PWM
  #define NEED_DOME_PWM_PINS
#endif

// Allow MY_BT_ADDR and DRIVE_STICK_BT_ADDR to be undefined above.

#ifndef MY_BT_ADDR
  #define MY_BT_ADDR nullptr
#endif
#ifndef DRIVE_STICK_BT_ADDR
  #define DRIVE_STICK_BT_ADDR nullptr
#endif
#ifndef DOME_STICK_BT_ADDR
  #define DOME_STICK_BT_ADDR nullptr
#endif

// Set PWM index and settings.

#ifdef NEED_DRIVE_PWM_PINS
  #define LEFT_MOTOR_PWM_INDEX  0
  #define RIGHT_MOTOR_PWM_INDEX 1
  #ifdef THROTTLE_MOTOR_PWM
    #define THROTTLE_PWM_INDEX 2
    #define DRIVE_PWM_SETTINGS LEFT_MOTOR_PWM_INDEX, RIGHT_MOTOR_PWM_INDEX, THROTTLE_PWM_INDEX
  #else
    #define DRIVE_PWM_SETTINGS LEFT_MOTOR_PWM_INDEX, RIGHT_MOTOR_PWM_INDEX
  #endif
#endif

#ifdef NEED_DOME_PWM_PINS
  #ifdef NEED_DRIVE_PWM_PINS
    #ifdef THROTTLE_MOTOR_PWM
      #define DOME_PWM_INDEX 3
    #else
      #define DOME_PWM_INDEX 2
    #endif
  #else
    #define DOME_PWM_INDEX 0
  #endif
  #define DOME_PWM_SETTINGS DOME_PWM_INDEX
#endif

// ****************************************
// ****************************************
#ifdef BLACBOX

#ifdef USE_OLED
// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
#elif defined(ARDUINO_NRF52832_FEATHER)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840, esp32-s2 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
#endif

#include "JoystickController.h"
struct Gamepad
{
  JoystickController::Button button;
  JoystickController::AnalogStick stick;
};

#endif	// #ifdef USE_OLED
#endif	// #ifdef BLACBOX
// ****************************************
// ****************************************

#endif