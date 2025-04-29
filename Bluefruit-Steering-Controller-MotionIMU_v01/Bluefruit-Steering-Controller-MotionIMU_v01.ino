#include <BluefruitSteeringServer.h>

// ------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress DEBUG messages that help debugging...
// Uncomment general "#define DEBUG" to activate
#define DEBUG
// Include these debug utility macros in all cases!
#include "DebugUtils.h"
// ------------------------------------------------------------------------------------

// When a LIPO Battery is attched to the board UNcomment this include
#include "batterylevel.h"

// ------------------------------------------------------------------------------------
// Steering Angle variable
float steerAngle = 0;

// ------------------------------------------------------------------------------------
// Battery Level variable set default
uint8_t batteryPercentage = 90; // Default value

// -----------------------------------------------------------------------------------
// Include specific HCI device/sensor functions for input of steering state and actions
// -----------------------------------------------------------------------------------
#include "motionIMU.h"

// -----------------------------------------------------------------------------------

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
    while ( !Serial ) delay(10); // for nrf52840 with native usb, milliseconds
    Serial.flush();
    delay(1000); // Give Serial I/O time to settle
#endif
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("Open Virtual Steering MotionIMU v01");
    DEBUG_PRINTLN("-------  Feather nRF52840  --------");
    delay(500);

    // When "batterylevel.h" is included -> BATTERY is defined
#ifdef BATTERY
    setupBatteryState();
    batteryPercentage = getBatteryState();
    DEBUG_PRINTF("LiPo Battery Setup - Level: %3d%%\n", batteryPercentage);
#else
    DEBUG_PRINTF("NO LiPo Battery! - Default level: %3d%%\n", batteryPercentage);
#endif

    // Setup HCI device or sensor for steering    
    setupSteeringState(); 

    // Start BluefruitSteeringServer  
    BluefruitSteeringServer::getInstance().begin();
}

// Check update time of battery level after 30 seconds
bool isTimeToUpdateBatteryLevel(void) {
  static unsigned long updateDelayTime = 0;
  if(millis() >= updateDelayTime) {
    updateDelayTime = millis() + 30000; // Set next moment after 30 seconds
    return true;
  } 
  return false;
}

// Set MAX Number-of-Updates-per-Second to be sent to Client
#define UPDATESPERSECOND 8 
// Calculate time delay between updates
const unsigned long TIMESPANUPDATEDELAY = 1000/UPDATESPERSECOND; 

bool isTimeToUpdateSteerAngle(void) {
  static unsigned long updateDelayTime = 0;
  if(millis() >= updateDelayTime) {
    updateDelayTime = millis() + TIMESPANUPDATEDELAY; // Set next moment
    return true;
  } 
  return false;
}

void loop() {
    steerAngle = getSteeringState(steerAngle);  // Always fetch HCI input with minimal delay
    if(isTimeToUpdateBatteryLevel()) {          // Update present battery level value 
        // When "batterylevel.h" is included -> BATTERY is defined      
#ifdef BATTERY
        batteryPercentage = getBatteryState();
#endif
        BluefruitSteeringServer::getInstance().updateBatteryPercentage(batteryPercentage);
        }
    if(isTimeToUpdateSteerAngle()) {           // Update steer angle at max allowed pace!
        if(BluefruitSteeringServer::getInstance().updateSteeringValue(steerAngle))
            DEBUG_PRINTF("Steer Angle: %.1f\n", steerAngle);
    }
} // loop
