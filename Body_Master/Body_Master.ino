// =======================================================================================
//         Penumbra: Minimal drive system for the ESP32
//          BLACBox: Penumbra enhanced into "Brian Lubkeman's Astromech Controller"
// =======================================================================================
//  Written by: skelmir
//  Modified by: Brian "Scruffy" Lubkeman
//
//  Change Log
//  ----------
//  2021-12-30      Added Adafruit FeatherWing 128x64 OLED display
//  2021-12-17 bel  Added ESP-NOW sender support
//  2021-12-12 bel  Copied from latest release
//                  Redefined controls within drive controller class
//                  Added speed profiles walk, jog, run, sprint
//                  Added function getEnabled()
// =======================================================================================
//
//         This program is distributed in the hope that it will be useful 
//         as a courtesy to fellow astromech club members wanting to develop
//         their own droid control system.
//
//         IT IS OFFERED WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//         You are using this software at your own risk and, as a fellow club member, it is
//         expected you will have the proper experience / background to handle and manage that 
//         risk appropriately.  It is completely up to you to insure the safe operation of
//         your droid and to validate and test all aspects of your droid control system.
//
// =======================================================================================
//   Note: You will need an ESP32 (either WROOM or WRover).
//
//   You will need to install the Reeltwo library. 
//      https://github.com/reeltwo/Reeltwo
//
//   To use Roboteq SBL2360 as your motor controller. Define:
//     #define DRIVE_SYSTEM         DRIVE_SYSTEM_ROBOTEQ_PWM
//
//   To use Roboteq SBL2360 as your motor controller and enable turtle command control. Define:
//     #define DRIVE_SYSTEM         DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL
//
//   To use Sabertooth 2x32 as your motor controller. Define:
//     #define DRIVE_SYSTEM         DRIVE_SYSTEM_SABER
//
//   To use a PWM based motor controller. Define:
//     #define DRIVE_SYSTEM         DRIVE_SYSTEM_PWM
//
//   To use a PWM based motor controller. Define:
//     #define DRIVE_SYSTEM         DRIVE_SYSTEM_PWM
//
//   Change this macro to change the dome motor system:
//     #define DOME_DRIVE           DOME_DRIVE_SABER
//
//    To disable the dome motor support change it to:
//      #define DOME_DRIVE          DOME_DRIVE_NONE
//
//   Define MY_BT_ADDR to the address stored in your PS3 Navication controller. Or update
//   your PS3 Nav controller to match the BT address of your ESP32.
//     #define MY_BT_ADDR          "24:6f:28:44:a5:ae"
//
// =======================================================================================
//
//  PS3 Controll button mapping:
//
//    L2: Throttle
//    L1: Hard brake
//    Joystick: Drive
//
//  PS3 Dome controller:
//
//    Joystick: Left/right spin dome
//
//  All other buttons are unmapped and will print a debug message for you to repurpose
// =======================================================================================
//
// Gesture codes for buttons are as follows:
//
//    X: Cross
//    O: Circle
//    U: Up
//    R: Down
//    L: Left
//    P: PS
//
// Gesture codes for the analog stick are as follows:
//
//    1 2 3
//    4 5 6
//    7 8 9
//
// For example:
//
//    GESTURE: UDU    (KeyPad: Up-Down-Up)
//    GESTURE: 252    (Analog stick Up-Center-Up)
//
// You can disable gesture support by commenting or deleting the following:
//
// #define DOME_CONTROLLER_GESTURES
// =======================================================================================
//
#include "User_Settings.h"
#include "ReelTwo.h"
#include "drive/TankDrivePWM.h"
#include "drive/TankDriveRoboteq.h"
#if DRIVE_SYSTEM == DRIVE_SYSTEM_SABER
#include "drive/TankDriveSabertooth.h"
#endif
#if DOME_DRIVE != DOME_DRIVE_NONE
#include "drive/DomeDrivePWM.h"
#if DOME_DRIVE == DOME_DRIVE_SABER
#include "drive/DomeDriveSabertooth.h"
#endif
#endif
#include "ServoDispatchDirect.h"
#include "ServoEasing.h"
#include "src/Images.h"
#include <Preferences.h>
#ifdef USE_WIFI
#include "wifi/WifiAccess.h"
#include <ESPmDNS.h>
#ifdef USE_WIFI_WEB
#include "wifi/WifiWebServer.h"
#endif
#endif
#include "bt/PSController/PSController.h"

#ifdef USE_OTA
#include <ArduinoOTA.h>
#endif

// ****************************************
// ****************************************
#ifdef USE_MINI_RELAY
#include "src/Adafruit_Mini_Relay_FeatherWing.h"
#endif

#ifdef USE_ESPNOW
#include "wifi/WifiESPNOWSender.h"
ESPNOW_Sender espNow(broadcastTo, (sizeof(broadcastTo) / sizeof(broadcastTo[0].address)) );
#endif

#ifdef USE_OLED
#include "src/Adafruit_SH110X_OLED.h"
SH110X_OLED display;
#endif
// ****************************************
// ****************************************

////////////////////////////////

// Group ID is used by the ServoSequencer and some ServoDispatch functions to
// identify a group of servos.
//
//     Pin,                Min,  Max,  Group ID
const ServoSettings servoSettings[] PROGMEM = {
#ifdef NEED_DRIVE_PWM_PINS
     { LEFT_MOTOR_PWM,      800, 2200, 0 }
    ,{ RIGHT_MOTOR_PWM,     800, 2200, 0 }
  #ifdef THROTTLE_MOTOR_PWM
    ,{ THROTTLE_MOTOR_PWM, 1000, 2000, 0 }
  #endif
  #ifdef NEED_DOME_PWM_PINS
    ,{ DOME_MOTOR_PWM,      800, 2200, 0 }
  #endif
#endif
};
ServoDispatchDirect<SizeOfArray(servoSettings)> servoDispatch(servoSettings);

////////////////////////////////

TaskHandle_t eventTask;
Preferences preferences;
#ifdef SERIAL_MARCDUINO_TX_PIN
SoftwareSerial marcSerial;
#endif

////////////////////////////////////

void enableController();
void disableController();
void emergencyStop();
void enableDomeController();
void disableDomeController();
void domeEmergencyStop();

// ****************************************
// ****************************************
#ifdef BLACBOX

bool getEnabled();

JoystickController::Button getButton();
JoystickController::AnalogStick getStick();

#ifdef USE_OLED
uint8_t getPage();
void setControllerNone();
void setControllerActive();
#endif

#endif
// ****************************************
// ****************************************

////////////////////////////////////

class DriveController : public PSController, public Adafruit_Mini_Relay_FeatherWing
{
public:
    DriveController(const char* mac = nullptr) : PSController(mac, DRIVE_CONTROLLER_TYPE) {}
    virtual void notify() override
    {
/*
        // ****************************************
        // ****************************************
        #if defined(BLACBOX) && defined(USE_OLED)
        if ( !isConnected() )
        {
          if ( getPage() != CONTROLLER_NONE )
          {
            setControllerNone();
          }
          return;
        }
        else
        {
          if ( getPage() != CONTROLLER_ACTIVE )
          {
            setControllerActive();
          }
        }
        #endif
        // ****************************************
        // ****************************************
*/
        uint32_t currentTime = millis();
        uint32_t lagTime = (currentTime > fLastTime) ? currentTime - fLastTime : 0;
        if (lagTime > 5000)
        {
            DEBUG_PRINTLN("More than 5 seconds. Disconnect");
            emergencyStop();
            disconnect();
        }
        else if (lagTime > 500)
        {
            DEBUG_PRINTLN("It has been 500ms. Shutdown motors");
            emergencyStop();
        }
        else
        {
            // ****************************************
            // ****************************************
            #ifdef BLACBOX
            interpretController();
            #endif
            // ****************************************
            // ****************************************
        }
        fLastTime = currentTime;
    }
    
    virtual void onConnect() override
    {
        DEBUG_PRINTLN("Drive Stick Connected");
        setPlayer(1);
        enableController();
        fLastTime = millis();
        // ****************************************
        // ****************************************
        #if defined(BLACBOX) && defined(USE_OLED)
        DEBUG_PRINTLN("Updating OLED - ACTIVE");
        setControllerActive();
        #endif
        // ****************************************
        // ****************************************
    }
    
    virtual void onDisconnect() override
    {
        DEBUG_PRINTLN("Drive Stick Disconnected");
        disableController();
        // ****************************************
        // ****************************************
        #if defined(BLACBOX) && defined(USE_OLED)
        DEBUG_PRINTLN("Updating OLED - NONE");
        setControllerNone();
        #endif
        // ****************************************
        // ****************************************
    }

private:
    uint32_t fLastTime = 0;

// ****************************************
// ****************************************
#if defined(BLACBOX)

    #if DRIVE_CONTROLLER_TYPE == kPS4
    enum Speed_Profile_e
    {
      WALK,
      JOG,
      RUN,
      SPRINT
    };
    uint8_t fCurrentSpeed = WALK;
    #endif  // #if DRIVE_CONTROLLER_TYPE == kPS4

    #ifdef USE_MINI_RELAY
    Adafruit_Mini_Relay_FeatherWing fRelay;
    #endif  // #ifdef USE_MINI_RELAY

    void interpretController()
    {
        // ==============================
        //    Drive controls
        // PS+L2      = Disconnect controller
        // PS+SHARE   = Disable drive (or PS+SELECT)
        // PS+OPTIONS = Enable drive (or PS+START)
        // L1         = Stop
        // L1+L3      = Change speed profile
        // L2         = Throttle
        // Left Stick = Steering
        // ==============================
        //    Dome controls
        // R1          = Stop
        // Right Stick = Dome rotation
        // ==============================

        if ( state.button.ps ) {
            if ( event.button_down.l2) {
                DEBUG_PRINTLN("PS+L2 - Disconnect");
                emergencyStop();
                disconnect();
            } else if ( event.button_down.share || event.button_down.select ) {
                if ( getEnabled() ) {
                    DEBUG_PRINTLN("PS+SHARE - Disable drive");
                    disableController();
                    #if defined(LATCHING)
                    fRelay.unset();
                    #endif
                    updateLED();
                }
            } else if ( event.button_down.options || event.button_down.start ) {
                if ( !getEnabled() ) {
                    DEBUG_PRINTLN("PS+OPTIONS - Enable drive");
                    enableController();
                    #if defined(LATCHING)
                    fRelay.set();
                    #endif
                    updateLED();
                }
            }
        } else if ( event.button_down.l1 ) {
            DEBUG_PRINTLN("L1 - Hard stop");
            emergencyStop();
        } else if ( state.button.l1 && event.button_down.l3 ) {
        #if DRIVE_CONTROLLER_TYPE == kPS4
            if ( fCurrentSpeed == SPRINT ) {
              fCurrentSpeed = WALK;
            } else {
              fCurrentSpeed++;
            }
            DEBUG_PRINT("L1+L3 - Next speed ");  DEBUG_PRINTLN(fCurrentSpeed);
            updateLED();
        #endif
        } else if ( event.button_down.r1 ) {
            DEBUG_PRINTLN("R1 - Dome stop");
            domeEmergencyStop();
        }

        #ifdef USE_ESPNOW
        espNow.LoadControllerData(&state, &event);
        #endif
    }

    void updateLED()
    {
    #if DRIVE_CONTROLLER_TYPE == kPS4
        if (!getEnabled())
        {
            setLED(255, 0, 0);      // Red
            return;
        }
        switch (fCurrentSpeed) {
            case Speed_Profile_e::WALK :
            {
                setLED(255, 255, 0);  // Yellow
                break;
            }
            case Speed_Profile_e::JOG :
            {
                setLED(0, 255, 0);    // Green
                break;
            }
            case Speed_Profile_e::RUN :
            {
                setLED(0, 0, 255);    // Blue
                break;
            }
            case Speed_Profile_e::SPRINT :
            {
                setLED(255, 0, 255);  // Purple
                break;
            }
        }
    #endif  // kPS4
    }
#endif  // #ifdef BLACBOX
// ****************************************
// ****************************************
};

DriveController driveStick(DRIVE_STICK_BT_ADDR);

class DomeController : public PSController
{
public:
    DomeController(const char* mac = nullptr) : PSController(mac) {}
    virtual void notify() override
    {
        uint32_t currentTime = millis();
        uint32_t lagTime = (currentTime > fLastTime) ? currentTime - fLastTime : 0;
        if (lagTime > 5000)
        {
            DEBUG_PRINTLN("More than 5 seconds. Disconnect");
            domeEmergencyStop();
            disconnect();
        }
        else if (lagTime > 300)
        {
            DEBUG_PRINTLN("It has been 300ms. Shutdown motors");
            domeEmergencyStop();
        }
        else
        {
            process();
        }
        fLastTime = currentTime;
    }

    void process()
    {
        if (!fGestureCollect)
        {
        #ifdef DOME_CONTROLLER_GESTURES
            if (event.button_up.l3)
            {
                DEBUG_PRINTLN("GESTURE START COLLECTING\n");
                disableDomeController();
                fGestureCollect = true;
                fGesturePtr = fGestureBuffer;
                fGestureTimeOut = millis() + GESTURE_TIMEOUT_MS;
            }
        #else
            // Event handling map these actions for your droid.
            // You can choose to either respond to key down or key up
            if (event.button_down.l3)
            {
                DEBUG_PRINTLN("DOME L3 DOWN");
            }
            else if (event.button_up.l3)
            {
                DEBUG_PRINTLN("DOME L3 UP");
            }
        #endif
            if (event.button_down.cross)
            {
                DEBUG_PRINTLN("DOME X DOWN");
            }
            else if (event.button_up.cross)
            {
                DEBUG_PRINTLN("DOME X UP");
            }

            if (event.button_down.circle)
            {
                DEBUG_PRINTLN("DOME O DOWN");
            }
            else if (event.button_up.circle)
            {
                DEBUG_PRINTLN("DOME O UP");
            }

            if (event.button_down.up)
            {
                DEBUG_PRINTLN("DOME Started pressing the up button");
            }
            else if (event.button_up.up)
            {
                DEBUG_PRINTLN("DOME Released the up button");
            }

            if (event.button_down.right)
            {
                DEBUG_PRINTLN("DOME Started pressing the right button");
            }
            else if (event.button_up.right)
            {
                DEBUG_PRINTLN("DOME Released the right button");
            }

            if (event.button_down.down)
            {
                DEBUG_PRINTLN("DOME Started pressing the down button");
            }
            else if (event.button_up.down)
            {
                DEBUG_PRINTLN("DOME Released the down button");
            }

            if (event.button_down.left)
            {
                DEBUG_PRINTLN("DOME Started pressing the left button");
            }
            else if (event.button_up.left)
            {
                DEBUG_PRINTLN("DOME Released the left button");
            }

            if (event.button_down.ps)
            {
                DEBUG_PRINTLN("DOME PS DOWN");
            }
            else if (event.button_up.ps)
            {
                DEBUG_PRINTLN("DOME PS UP");
            }
            return;
        }
        else if (fGestureTimeOut < millis())
        {
            DEBUG_PRINTLN("GESTURE TIMEOUT\n");
            enableDomeController();
            fGesturePtr = fGestureBuffer;
            fGestureCollect = false;
        }
        else
        {
            if (event.button_up.l3)
            {
                // delete trailing '5' from gesture
                unsigned glen = strlen(fGestureBuffer);
                if (glen > 0 && fGestureBuffer[glen-1] == '5')
                    fGestureBuffer[glen-1] = 0;
                DEBUG_PRINT("GESTURE: "); DEBUG_PRINTLN(fGestureBuffer);
                enableDomeController();
                fGestureCollect = false;
            }
            if (event.button_up.cross)
                addGesture('X');
            if (event.button_up.circle)
                addGesture('O');
            if (event.button_up.up)
                addGesture('U');
            if (event.button_up.right)
                addGesture('R');
            if (event.button_up.down)
                addGesture('D');
            if (event.button_up.left)
                addGesture('L');
            if (event.button_up.ps)
                addGesture('P');
            if (!fGestureAxis)
            {
                if (abs(state.analog.stick.lx) > 50 && abs(state.analog.stick.ly) > 50)
                {
                    // Diagonal
                    if (state.analog.stick.lx < 0)
                        fGestureAxis = (state.analog.stick.ly < 0) ? '1' : '7';
                    else
                        fGestureAxis = (state.analog.stick.ly < 0) ? '3' : '9';
                    addGesture(fGestureAxis);
                }
                else if (abs(state.analog.stick.lx) > 100)
                {
                    // Horizontal
                    fGestureAxis = (state.analog.stick.lx < 0) ? '4' : '6';
                    addGesture(fGestureAxis);
                }
                else if (abs(state.analog.stick.ly) > 100)
                {
                    // Vertical
                    fGestureAxis = (state.analog.stick.ly < 0) ? '2' : '8';
                    addGesture(fGestureAxis);
                }
            }
            if (fGestureAxis && abs(state.analog.stick.lx) < 10 && abs(state.analog.stick.ly) < 10)
            {
                addGesture('5');
                fGestureAxis = 0;   
            }
        }
    }

    virtual void onConnect() override
    {
        DEBUG_PRINTLN("Dome Stick Connected");
        setPlayer(2);
        enableDomeController();
        fLastTime = millis();
    }
    
    virtual void onDisconnect() override
    {
        DEBUG_PRINTLN("Dome Stick Disconnected");
        disableDomeController();
    }

    uint32_t fLastTime = 0;

protected:
    bool fGestureCollect = false;
    char fGestureBuffer[MAX_GESTURE_LENGTH+1];
    char* fGesturePtr = fGestureBuffer;
    char fGestureAxis = 0;
    uint32_t fGestureTimeOut = 0;

    void addGesture(char ch)
    {
        if (fGesturePtr-fGestureBuffer < sizeof(fGestureBuffer)-1)
        {
            *fGesturePtr++ = ch;
            *fGesturePtr = '\0';
            fGestureTimeOut = millis() + GESTURE_TIMEOUT_MS;
        }
    }
};

#if DOME_DRIVE != DOME_DRIVE_NONE && DRIVE_CONTROLLER_TYPE == kPS3Nav
// If dome drive is enabled and we are using PS3 Nav Controllers
DomeController domeStick(DOME_STICK_BT_ADDR);
#elif DOME_DRIVE != DOME_DRIVE_NONE
// Dome Drive enabled using either PS3 or PS4 controller
// the right side of the controller will be used to control the dome (left/right)
#define domeStick driveStick
#endif

#ifdef USE_RADIO
RadioController radioStick(Serial2);
#endif

// ****************************************
// ****************************************
#ifdef USE_OLED

uint8_t getPage()
{
  return display.getPage();
}
void setControllerNone()
{
  display.setControllerNone();
}

void setControllerActive()
{
  display.setControllerActive();
}
#endif
// ****************************************
// ****************************************

#if DRIVE_SYSTEM == DRIVE_SYSTEM_SABER
// Tank Drive assign:
//    Serial1 for Sabertooth packet serial commands
TankDriveSabertooth tankDrive(128, Serial1, driveStick);
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_PWM
// Tank Drive assign:
//    servo index 0 (LEFT_MOTOR_PWM)
//    servo index 1 (RIGHT_MOTOR_PWM)
//    servo index 2 (THROTTLE_MOTOR_PWM)
TankDrivePWM tankDrive(servoDispatch, DRIVE_PWM_SETTINGS, driveStick);
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM
// Tank Drive assign:
//    servo index 0 (LEFT_MOTOR_PWM)
//    servo index 1 (RIGHT_MOTOR_PWM)
//    servo index 2 (THROTTLE_MOTOR_PWM)
TankDriveRoboteq tankDrive(servoDispatch, DRIVE_PWM_SETTINGS, driveStick);
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_SERIAL
// Tank Drive assign:
//    servo index 0 (LEFT_MOTOR_PWM)
//    servo index 1 (RIGHT_MOTOR_PWM)
//    servo index 2 (THROTTLE_MOTOR_PWM)
TankDriveRoboteq tankDrive(Serial1, driveStick);
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL
// Tank Drive assign:
//    servo index 0 (LEFT_MOTOR_PWM)
//    servo index 1 (RIGHT_MOTOR_PWM)
//    servo index 2 (THROTTLE_MOTOR_PWM)
//    Serial1 for Roboteq serial commands
TankDriveRoboteq tankDrive(Serial1, servoDispatch, DRIVE_PWM_SETTINGS, driveStick);
#else
#error Unsupported DRIVE_SYSTEM
#endif

#ifdef NEED_DOME_PWM_PINS
DomeDrivePWM domeDrive(servoDispatch, DOME_PWM_SETTINGS, domeStick);
#elif DOME_DRIVE == DOME_DRIVE_SABER
DomeDriveSabertooth domeDrive(129, DOME_DRIVE_SERIAL, domeStick);
#endif

void enableController()
{
    tankDrive.setEnable(true);
}

void disableController()
{
    emergencyStop();
    tankDrive.setEnable(false);
}

void emergencyStop()
{
    tankDrive.stop();
}

void enableDomeController()
{
#if DOME_DRIVE != DOME_DRIVE_NONE
    domeDrive.setEnable(true);
#endif
}

void disableDomeController()
{
#if DOME_DRIVE != DOME_DRIVE_NONE
    domeEmergencyStop();
    domeDrive.setEnable(false);
#endif
}

void domeEmergencyStop()
{
#if DOME_DRIVE != DOME_DRIVE_NONE
    domeDrive.stop();
#endif
}

// ****************************************
// ****************************************
#ifdef BLACBOX
bool getEnabled()
{
    return tankDrive.getEnable();
}

#ifdef USE_OLED
JoystickController::Button getButton()
{
    return driveStick.state.button;
}

JoystickController::AnalogStick getStick()
{
    return driveStick.state.analog.stick;
}
#endif  // #ifdef USE_OLED
#endif  // #ifdef BLACBOX
// ****************************************
// ****************************************

#ifdef USE_WIFI
WifiAccess wifiAccess;
#endif

#ifdef USE_WIFI_WEB
////////////////////////////////
// Web configurable parameters. Strongly advice not to change web settings while motors are running
// unless the wheels are off the ground.
WElement mainContents[] = {
    W1("Drive Configuration"),
    WSlider("Max Speed", "maxspeed", 0, 100,
        []()->int { return tankDrive.getMaxSpeed()*100; },
        [](int val) { tankDrive.stop(); tankDrive.setMaxSpeed(val/100.0f); } ),
    WSlider("Guest Max Speed", "guestspeed", 0, 100,
        []()->int { return tankDrive.getGuestSpeedModifier()*100; },
        [](int val) { tankDrive.stop(); tankDrive.setGuestSpeedModifier(val/100.0f); } ),
    WSlider("Throttle Acceleration", "throttleAccel", 1, 400,
        []()->int { return tankDrive.getThrottleAccelerationScale(); },
        [](int val) { tankDrive.stop(); tankDrive.setThrottleAccelerationScale(val); } ),
    WSlider("Throttle Deceleration", "throttleDecel", 1, 400,
        []()->int { return tankDrive.getThrottleDecelerationScale(); },
        [](int val) { tankDrive.stop(); tankDrive.setThrottleDecelerationScale(val); } ),
    WSlider("Turn Acceleration", "turnAccel", 1, 400,
        []()->int { return tankDrive.getTurnAccelerationScale(); },
        [](int val) { tankDrive.stop(); tankDrive.setTurnAccelerationScale(val); } ),
    WSlider("Turn Deceleration", "turnDecl", 1, 400,
        []()->int { return tankDrive.getTurnDecelerationScale(); },
        [](int val) { tankDrive.stop(); tankDrive.setTurnDecelerationScale(val); } ),
    WCheckbox("Scale Throttle", "scaling",
        []() { return tankDrive.getScaling(); },
        [](bool val) { tankDrive.stop(); tankDrive.setScaling(val); } ),
    WCheckbox("Channel Mixing", "mixing",
        []() { return tankDrive.getChannelMixing(); },
        [](bool val) { tankDrive.stop(); tankDrive.setChannelMixing(val); } ),
    WCheckbox("Throttle Inverted", "throttleInvert",
        []() { return tankDrive.getThrottleInverted(); },
        [](bool val) { tankDrive.stop(); tankDrive.setThrottleInverted(val); } ),
    WCheckbox("Turn Inverted", "turnInvert",
        []() { return tankDrive.getTurnInverted(); },
        [](bool val) { tankDrive.stop(); tankDrive.setTurnInverted(val); } ),
    WButton("Save", "save", []() {
        preferences.putFloat(PREFERENCE_DRIVE_SPEED, tankDrive.getMaxSpeed());
        preferences.putFloat(PREFERENCE_DRIVE_THROTTLE_ACC_SCALE, tankDrive.getThrottleAccelerationScale());
        preferences.putFloat(PREFERENCE_DRIVE_THROTTLE_DEC_SCALE, tankDrive.getThrottleDecelerationScale());
        preferences.putFloat(PREFERENCE_DRIVE_TURN_ACC_SCALE, tankDrive.getTurnAccelerationScale());
        preferences.putFloat(PREFERENCE_DRIVE_TURN_DEC_SCALE, tankDrive.getTurnDecelerationScale());
        preferences.putFloat(PREFERENCE_DRIVE_GUEST_SPEED, tankDrive.getGuestSpeedModifier());
        preferences.putBool(PREFERENCE_DRIVE_SCALING, tankDrive.getScaling());
        preferences.putBool(PREFERENCE_DRIVE_MIXING, tankDrive.getChannelMixing());
        preferences.putBool(PREFERENCE_DRIVE_THROTTLE_INVERT, tankDrive.getThrottleInverted());
        preferences.putBool(PREFERENCE_DRIVE_TURN_INVERT, tankDrive.getTurnInverted());
    }),
    WButton("Restore", "restore", []() {
        tankDrive.setMaxSpeed(preferences.getFloat(PREFERENCE_DRIVE_SPEED, MAXIMUM_SPEED));
        tankDrive.setThrottleAccelerationScale(preferences.getFloat(PREFERENCE_DRIVE_THROTTLE_ACC_SCALE, ACCELERATION_SCALE));
        tankDrive.setThrottleDecelerationScale(preferences.getFloat(PREFERENCE_DRIVE_THROTTLE_DEC_SCALE, DECELRATION_SCALE));
        tankDrive.setTurnAccelerationScale(preferences.getFloat(PREFERENCE_DRIVE_TURN_ACC_SCALE, ACCELERATION_SCALE*2));
        tankDrive.setTurnDecelerationScale(preferences.getFloat(PREFERENCE_DRIVE_TURN_DEC_SCALE, DECELRATION_SCALE));
        tankDrive.setGuestSpeedModifier(preferences.getFloat(PREFERENCE_DRIVE_GUEST_SPEED, MAXIMUM_GUEST_SPEED));
        tankDrive.setScaling(preferences.getBool(PREFERENCE_DRIVE_SCALING, SCALING));
        tankDrive.setChannelMixing(preferences.getBool(PREFERENCE_DRIVE_MIXING, CHANNEL_MIXING));
        tankDrive.setThrottleInverted(preferences.getBool(PREFERENCE_DRIVE_THROTTLE_INVERT, THROTTLE_INVERTED));
        tankDrive.setTurnInverted(preferences.getBool(PREFERENCE_DRIVE_TURN_INVERT, TURN_INVERTED));
    }),
    WImage("astromech", ASTROMECH_IMAGE)
};
WPage pages[] = {
    WPage("/", mainContents, SizeOfArray(mainContents))
};
WifiWebServer<1,SizeOfArray(pages)> webServer(pages, wifiAccess);
#endif

void setup()
{
    REELTWO_READY();

    if (!preferences.begin("rseries", false))
    {
        DEBUG_PRINTLN("Failed to init prefs");
    }

#ifdef USE_WIFI_WEB
    // In preparation for adding WiFi settings web page
    wifiAccess.setNetworkCredentials(
        preferences.getString(PREFERENCE_WIFI_SSID, WIFI_AP_NAME),
        preferences.getString(PREFERENCE_WIFI_PASS, WIFI_AP_PASSPHRASE),
        preferences.getBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT),
        preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED));
#endif

#ifdef DRIVE_BAUD_RATE
    // Serial1 is used for drive system
    Serial1.begin(DRIVE_BAUD_RATE, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
#endif
#ifdef USE_RADIO
    Serial2.begin(57600, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
#endif
#ifdef SERIAL_MARCDUINO_TX_PIN
    // Transmit only software serial on SERIAL_MARCDUINO_TX_PIN
    marcSerial.begin(MARCDUINO_BAUD_RATE, SWSERIAL_8N1, -1, SERIAL_MARCDUINO_TX_PIN, false, 0);
#endif

    SetupEvent::ready();
    PSController::startListening(MY_BT_ADDR);

#ifdef USE_WIFI_WEB
    wifiAccess.notifyWifiConnected([](WifiAccess &wifi) {
        Serial.print("Connect to http://"); Serial.println(wifi.getIPAddress());
    #ifdef USE_MDNS
        // No point in setting up mDNS if R2 is the access point
        if (!wifi.isSoftAP())
        {
            String mac = wifi.getMacAddress();
            String hostName = mac.substring(mac.length()-5, mac.length());
            hostName.remove(2, 1);
            hostName = "RSeries-"+hostName;
            if (webServer.enabled())
            {
                Serial.print("Host name: "); Serial.println(hostName);
                if (!MDNS.begin(hostName.c_str()))
                {
                    DEBUG_PRINTLN("Error setting up MDNS responder!");
                }
            }
        }
    #endif
    });
#endif

#ifdef USE_OTA
    ArduinoOTA.onStart([]()
    {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        {
            type = "sketch";
        }
        else // U_SPIFFS
        {
            type = "filesystem";
        }
        DEBUG_PRINTLN("OTA START");
        // Kill the motors
        tankDrive.setEnable(false);
    })
    .onEnd([]()
    {
        DEBUG_PRINTLN("OTA END");
    })
    .onProgress([](unsigned int progress, unsigned int total)
    {
        float range = (float)progress / (float)total;
    })
    .onError([](ota_error_t error)
    {
        String desc;
        if (error == OTA_AUTH_ERROR) desc = "Auth Failed";
        else if (error == OTA_BEGIN_ERROR) desc = "Begin Failed";
        else if (error == OTA_CONNECT_ERROR) desc = "Connect Failed";
        else if (error == OTA_RECEIVE_ERROR) desc = "Receive Failed";
        else if (error == OTA_END_ERROR) desc = "End Failed";
        else desc = "Error: "+String(error);
        DEBUG_PRINTLN(desc);
    });
#endif

    tankDrive.setMaxSpeed(preferences.getFloat("maxspeed", MAXIMUM_SPEED));
    tankDrive.setThrottleAccelerationScale(preferences.getFloat("throttleaccscale", ACCELERATION_SCALE));
    tankDrive.setThrottleDecelerationScale(preferences.getFloat("throttledecscale", DECELRATION_SCALE));
    tankDrive.setTurnAccelerationScale(preferences.getFloat("turnaccscale", ACCELERATION_SCALE*2));
    tankDrive.setTurnDecelerationScale(preferences.getFloat("turndecscale", DECELRATION_SCALE));
#ifdef USE_RADIO
    tankDrive.setGuestStick(radioStick);
#endif
    tankDrive.setGuestSpeedModifier(preferences.getFloat("guestmaxspeed", MAXIMUM_GUEST_SPEED));
    tankDrive.setScaling(preferences.getBool("scaling", SCALING));
    tankDrive.setChannelMixing(preferences.getBool("mixing", CHANNEL_MIXING));
    tankDrive.setThrottleInverted(preferences.getBool("throttleinvert", THROTTLE_INVERTED));
    tankDrive.setTurnInverted(preferences.getBool("turninvert", TURN_INVERTED));

#ifdef USE_WIFI_WEB
    // For safety we will stop the motors if the web client is connected
    webServer.setConnect([]() {
        // Callback for each connected web client
        // DEBUG_PRINTLN("Hello");
    });
#endif
    // tankDrive.enterCommandMode();

    xTaskCreatePinnedToCore(
          eventLoopTask,
          "Events",
          5000,    // shrink stack size?
          NULL,
          1,
          &eventTask,
          0);

    DEBUG_PRINTLN("");
    DEBUG_PRINT("Total heap:  "); DEBUG_PRINTLN(ESP.getHeapSize());
    DEBUG_PRINT("Free heap:   "); DEBUG_PRINTLN(ESP.getFreeHeap());
    DEBUG_PRINT("Total PSRAM: "); DEBUG_PRINTLN(ESP.getPsramSize());
    DEBUG_PRINT("Free PSRAM:  "); DEBUG_PRINTLN(ESP.getFreePsram());
    DEBUG_PRINTLN();

#ifdef USE_RADIO
    radioStick.start();
#endif

// ****************************************
// ****************************************
#ifdef USE_OLED
    setControllerNone();
#endif
// ****************************************
// ****************************************

    DEBUG_PRINTLN("READY");
}

void eventLoopTask(void* arg)
{
    for (;;)
    {
        AnimatedEvent::process();
        vTaskDelay(1);
    }
}

void loop()
{
#ifdef USE_OTA
    ArduinoOTA.handle();
#endif
#ifdef USE_WIFI_WEB
    webServer.handle();
#endif
}
