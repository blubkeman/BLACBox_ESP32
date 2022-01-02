#ifndef __BLACBOX_BODY_MASTER_USER_SETTINGS_H__
#define __BLACBOX_BODY_MASTER_USER_SETTINGS_H__

/* ================================================================================
 * Change Log
 *  ----------
 *  2021-12-08 bel  Changed DRIVE_SYSTEM to DRIVE_SYSTEM_ROBOTEQ_PWM
 *                  Changed DOME_DRIVE to DOME_DRIVE_SABER
 *                  Changed MY_BT_ADDR to my ESP32 Bluetooth address
 *                  Changed DRIVE_CONTROLLER_TYPE to kPS4
 *                  Changed LEFT_MOTOR_PWM to 14
 *                  Changed RIGHT_MOTOR_PWM to 32
 *                  Changed THROTTLE_MOTOR_PWM to 15
 *                  Changed SERIAL1_RX_PIN to 16 and SERIAL1_TX_PIN to 17
 *                  Changed DOME_DRIVE_SERIAL to Serial1
 *                  Added USE_OLED for Adafruit OLED 128x64 display FeatherWing
 *  2021-12-28 bel  Added USE_ESPNOW to utilize the ESP-NOW feature of the ESP32 chip
 *                  Added USE_RELAY for Adafruit Mini Relay FeatherWing  (latching or unlatching
 *
 *  Adafruit HUZZAH32 Pinout - Body Master
 *  --------------------------------------
 *  23/SDA  I2C bus           21/      Speed Profile Script
 *  22/SCL  I2C bus           17/TX    Syren10 (Sabertooth) for dome
 *  14/A6   OLED Button A     16/RX    Syren10 (Sabertooth) for dome
 *  32/A7   OLED Button B     19/MISO  
 *  15/A8   OLED Button C     18/MOSI  
 *  33/A9   Left motor         5/SCK   
 *  27/A10  Right motor        4/A5    
 *  12^/A11 Throttle Script   36#/A4   
 *  13/A12  built-in LED      39#/A3
 *                            34#/A2
 *                            25/A1    Latching Relay Set
 *                            26/A0    Latching Relay Unset
 *  # = input only pins
 *  ^ = has built-in pull-down resistor
 *  
 *  Adafruit SSID = 'adafruit' IP = 10.0.1.23
 * ================================================================================ */

// =============================================================

#include "src/Fixed_Settings.h"    // Do not move this line's placement.

// =============================================================

// ------------------------------
// BLACBox specific settings

// ========== USE_BLACBOX ==========

// Uncomment when this controller is using BLACBox.
#define BLACBOX
#ifdef BLACBOX

// ========== USE_ESPNOW ==========

// Uncomment to enabe ESP-NOW in the sender role.
//#define USE_ESPNOW

// Define more settings for ESP-NOW
#ifdef USE_ESPNOW

// Set the maximum number of receivers. This is used to allocate memory in arrays.
#define MAX_RECEIVERS 2

// Identify the MAC addresses of the receiver ESP32 boards.
typedef struct {
  uint8_t address[6];
} MAC_Address_Struct;

static const MAC_Address_Struct broadcastTo[MAX_RECEIVERS] = {
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // Dome ESP32 MAC address
 ,{0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}  // Body FX ESP32 MAC address
};
#endif  // #ifdef USE_ESPNOW

// ========== USE_OLED ==========

// Uncomment to enable the Adafruit OLED 128x64 display FeatherWing
#define USE_OLED

#ifdef USE_OLED
#define LINE_SIZE 10;
#endif

// ========== USE_MINI_RELAY ==========

// Uncomment to enable an Adafruit Mini Relay Latching (latching or unlatching)
#define USE_MINI_RELAY

// Define more settings for ESP-NOW
#ifdef USE_MINI_RELAY

// Uncomment one of the following options
//#define UNLATCHING
#define LATCHING
// Select the pins to be used by the relay
#define RELAY_SET_PIN    25   // Used by both latching and unlatching
#define RELAY_UNSET_PIN  26   // Used by latching only; ignore for unlatching
#endif  // #ifdef USE_MINI_RELAY

// ==========

#endif  // #ifdef BLACBOX

// ------------------------------
// Debug modes

// Uncomment a line to enable the debugging mode.

#define USE_DEBUG
//#define USE_MOTOR_DEBUG
//#define USE_SERVO_DEBUG
//#define USE_VERBOSE_SERVO_DEBUG

// ------------------------------
// Bluetooth

// Bluetooth address of this ESP32 device. If you already have a Shadow system configured
// the easiest thing is reuse the address of your USB Bluetooth dongle here. Alternatively,
// you can use sixaxispair to pair your controllers with the ESP32.

//#define MY_BT_ADDR             "7c:9e:bd:d8:eB:00"    // My Adafruit HUZZAH32 MAC address
#define MY_BT_ADDR             "ac:67:b2:6c:70:8c"  // My Sparkfun ESP32 Thing Plus MAC address
//#define MY_BT_ADDR             "00:15:83:F3:61:A6"  // My SHADOW controller's USB Dongle MAC address

// Assign a Bluetooth address here if you want a specific controller as the
// drive stick or dome stick otherwise it will be first come first serve.

#define DRIVE_STICK_BT_ADDR    nullptr
//#define DRIVE_STICK_BT_ADDR  "00:07:04:09:b4:05"

#define DOME_STICK_BT_ADDR     nullptr
//#define DOME_STICK_BT_ADDR   "e0:ae:5e:9b:e1:04"

// ------------------------------
// Controller type

// Uncomment one of the following that is the type of PS controller you are using (default PS3)

//#define DRIVE_CONTROLLER_TYPE   kPS3Nav
//#define DRIVE_CONTROLLER_TYPE   kPS3
#define DRIVE_CONTROLLER_TYPE   kPS4

// ------------------------------
// Drive system

// Select an option from the following.
// Choose from:
//  DRIVE_SYSTEM_PWM
//  DRIVE_SYSTEM_SABER
//  DRIVE_SYSTEM_ROBOTEQ_PWM
//  DRIVE_SYSTEM_ROBOTEQ_SERIAL
//  DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL

#define DRIVE_SYSTEM    DRIVE_SYSTEM_ROBOTEQ_PWM

// ------------------------------
// Dome system

// Select an option from the following.
// Choose from:
//  DOME_DRIVE_NONE
//  DOME_DRIVE_PWM
//  DOME_DRIVE_SABER

#define DOME_DRIVE    DOME_DRIVE_SABER

// Uncomment to enable dome controller gestures.
// Press joystick button L3. Will collect a series of button presses and joystick movements.
// Press joystick button L3 to recognize gesture.

#define DOME_CONTROLLER_GESTURES

// ------------------------------
// SYK radio controller

// Uncomment to enable the optional SYK base radio controller

//#define USE_RADIO

// ------------------------------
// Assign pins

#define SERIAL1_RX_PIN 16   // Set to your board's TX pin for Serial1.
#define SERIAL1_TX_PIN 17   // Set to your board's RX pin for Serial1.

#define SERIAL2_RX_PIN 18   // Set to your board's TX pin for Serial2. 
#define SERIAL2_TX_PIN 19   // Set to your board's RX pin for Serial2.

// Uncomment the following to enable use of Software Serial for Marcduino

//#include "SoftwareSerial.h"
//#define SERIAL_MARCDUINO_TX_PIN 5 // Set to the pin you use for Software Serial

// Set the following only if you use PWM for your drive system.
// You may ignore this if you do not use PWM for your drive system.

#define LEFT_MOTOR_PWM      33  // Set to a PWM pin for left foot motor.
#define RIGHT_MOTOR_PWM     27  // Set to a PWM pin for right foot motor.
#define THROTTLE_MOTOR_PWM  12  // Optional Roboteq pin used for MicroBasic scripts running on the Roboteq controller to change the throttle.
#define SPEED_PROFILE_PWM   21  // If the Microbasic script is not runnig this PWM signal will have no effect.

// Set the following only if you use PWM for your dome system.
// You may ignore this if you do not use PWM for your dome system.

#define DOME_MOTOR_PWM      27  // Set to a PWM pin used to control the dome motor

// ------------------------------
// Enable WiFi

// Comment this line out if you do not want to use the WiFi to
// change preferences via a web application on your phone.

#define USE_WIFI

// Configure this section only if the WiFi is enabled.

#ifdef USE_WIFI

// Uncomment to enable mDNS
#define USE_MDNS

// Uncomment to enable OTA update
//#define USE_OTA

// Uncomment to enable Web interface
#define USE_WIFI_WEB

#define WIFI_ENABLED            true
// Set these to your desired credentials.
#define WIFI_AP_NAME            "R2D2"
#define WIFI_AP_PASSPHRASE      "Astromech"
#define WIFI_ACCESS_POINT       true  /* true if access point: false if joining existing wifi */

// Alternatively join an existing AP
//#define WIFI_AP_NAME          "MyWifi"
//#define WIFI_AP_PASSPHRASE    "MyPassword"
//#define WIFI_ACCESS_POINT     false  /* true if access point: false if joining existing wifi */
#endif

// =============================================================

#include "src/Variable_Settings.h"    // Do not move this line's placement.

// =============================================================

// ------------------------------
// Baud rates

#ifndef DRIVE_BAUD_RATE
  #define DRIVE_BAUD_RATE      9600
#endif

#ifndef MARCDUINO_BAUD_RATE
  #define MARCDUINO_BAUD_RATE  9600
#endif

// ------------------------------
// Gesture settings

#ifndef MAX_GESTURE_LENGTH
  #define MAX_GESTURE_LENGTH   20
#endif

#ifndef GESTURE_TIMEOUT_MS
  #define GESTURE_TIMEOUT_MS   2000
#endif

// =============================================================

#endif
