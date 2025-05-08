/*

*/

#include "Filters.h"

// A simplified one dimensional Kalman filter implementation
Kalman kalmanAngleZ(0.125, 32, 1023, 0); //suggested initial values for high noise filtering
Kalman kalmanAngleX(0.125, 32, 1023, 0); //suggested initial values for high noise filtering

// Exponential Moving Average EMA filter definition
// Should be between low (10-40) is maximal and high (50-90) is minimal filtering
#define EMA_ALPHA_ANGLEZ        10  // To determine stable Yaw from mpuAngleZ readings
EMA_Filter emaAngleZ(EMA_ALPHA_ANGLEZ);
#define EMA_ALPHA_ANGLEX         5  // To determine stable rollAngle from AngleX readings 
EMA_Filter emaAngleX(EMA_ALPHA_ANGLEX);

// Standard Deviations defined for Z-axis and X-axis
StdDeviation devAngleZ;
StdDeviation devAngleX;

#include <MPU6050_light.h>
// Instantiate MPU6050 sensor
MPU6050 mpu(Wire);

#define ROLLANGLETHRESHOLD     (2.0F)
#define ROCKINGSTDDEVTHRESHOLD (4.0F)
#define YAWANGLETHRESHOLD      (2.0F)
#define YAWSTDDEVTHRESHOLD     (0.4F) 

// Final Scaling variables
#define SCALEFACTOR         (1.02F) // Set the scaling factor to your preference
float scaleFactor = 1.0;            // Allows to tweak "Sensitiveness" of (leaning AND turning)

// YAW variables
#define INVERT_YAWANGLE_SIGN         // Invert(!) with Standard MPU mount -> left turn is negative, right turn is positive 
float mpuAngleZ = 0.0;               // -> MPU Reading for determination of Yaw value
float prevAngleZ = 0.0;              // Previous mpuAngleZ value
float Yaw = 0.0;                     // Stable (Yaw == 0) straight forward orientation value 
float yawAngle = 0.0;                // Deflection angle -> (mpuAngleZ - Yaw)

// ROLL variables
//#define INVERT_ROLLANGLE_SIGN      // UNcomment with NOT Standard MPU mount, X-axis rotated 180 degrees
bool isRocking = false;
float mpuAngleX = 0.0;               // -> MPU Reading for determination of Roll value
float prevAngleX = 0.0;              // Previous mpuAngleX value
float rollAngle = 0.0;               // Angle (Roll) when leaning left (negative) or right (positive) to steer

void setupSteeringState(void) {
// join I2C bus
  Wire.begin();
// ------------------------------------------------------------
// initialize IMU device, set status and prepare for operation
// ------------------------------------------------------------
  DEBUG_PRINTLN("Initializing IMU device...");
/*
 * Mapping of the different gyro and accelero configurations:
 *
 * GYRO_CONFIG_[0,1,2,3] range = +- [250, 500,1000,2000] deg/s
 *                       sensi =    [131,65.5,32.8,16.4] bit/(deg/s)
 *
 * ACC_CONFIG_[0,1,2,3] range = +- [    2,   4,   8,  16] times the gravity (9.81 m/s^2)
 *                      sensi =    [16384,8192,4096,2048] bit/gravity
*/
  byte status = mpu.begin(1, 0); // default: (int gyro_config_num=1, int acc_config_num=0)
  DEBUG_PRINTF("MPU6050 started --> status: [%d]\n", status);
  /* Status is derived from I2C Wire() -> endTransmission()
     0: success
     1: busy timeout upon entering endTransmission()
     2: START bit generation timeout
     3: end of address transmission timeout
     4: data byte transfer timeout
     5: data byte transfer succeeded, busy timeout immediately after
     6: timeout waiting for peripheral to clear stop bit
  */
  if(status != 0) {
    //char status_reason[10] = {0};
    //sprintf(status_reason, "Reason:%2X", status);
    DEBUG_PRINTLN("Hanging -> restart!");
    while(status != 0) { }          // Hang -> NO connection with MPU6050
  }
  DEBUG_PRINTLN("Calibrating starts, do not move MPU6050!");
  //mpu.upsideDownMounting = true;  // uncomment this line when the MPU6050 is mounted upside-down
  mpu.calcOffsets(true, true);      // gyro and accelero
  // Internal complementary f i l t e r is set by the gyroscope coefficient (from 0.0 to 1.0 )
  mpu.setFilterGyroCoef(0.05);      // 0.05 -> full emphasis on Accelero (full emphasis on Gyro -> 0.98)
  // Capture Stable values and init Yaw and Roll!
  for(int i = 0; i < 20; i++) {
    mpu.update();
    delay(5);
  }
  prevAngleZ = Yaw = mpuAngleZ = mpu.getAngleZ();    // Set valid start values for Z-axis
  prevAngleX = mpuAngleX = mpu.getAngleX();          // Set valid start values for X-axis to zero
  DEBUG_PRINTLN("MPU6050 is ready for use!");
// --------------------------------------------------------  
}


float setYawAngle(float Angle) {
  if( fabs(Angle) < YAWANGLETHRESHOLD ) { // below threshold value
      return 0.0; // NO turning
  }
  return Angle;
}

float setRollAngle(float Angle)
{
  // Within threshold zone should not lead to Roll steering!
  if ( fabs(Angle) < ROLLANGLETHRESHOLD ) {
    return 0.0; // No contribution...
  }
  return (1.5*Angle); // Give more weight to values

}

bool leftX(float x) {
  if (x < -1.0) return true;
  return false;
}

bool rightX(float x) {
  if (x > 1.0) return true;
  return false;
}

bool leftZ(float x) {
  if (x < 0.0) return true;
  return false;
}

bool rightZ(float x) {
  if (x > 0.0) return true;
  return false;
}

float sgn(float x) {
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return 0.0;
}

//----------------------------------------------------------------------------------    
// Get the contribution by Yaw & Roll movements from MPU6050 -----------------------
//----------------------------------------------------------------------------------
float getSteeringState(float Angle)
{
  // Set default Standard Deviation variables to zero
  float Std_Deviation_AngleZ = devAngleZ.ClearStdDeviation(); // Std Deviation of mpuAngleZ to zero
  float Std_Deviation_AngleX = devAngleX.ClearStdDeviation(); // Std Deviation of mpuAngleX to zero
#ifdef DEBUG
  unsigned long timeSpan = millis(); // to measure time spent in the next for loop
#endif    
#define NUMOFREADINGS 15 // 15 will take about 44 ms
  for(int i = 0; i < NUMOFREADINGS; i++) {
      mpu.update(); // Measurement cycle
      delay(2);     // 2 milliseconds to settle internal IMU processing
      // Determine position contribution due to ----------- Y A W --------------------------------------------------------
      // Depending on Z sensor axis orientation in the enclosure
      // Notice correct sign: left turn is negative, right turn is positive 
  #ifdef INVERT_YAWANGLE_SIGN
      mpuAngleZ = -mpu.getAngleZ();                            // Determine inverted MPU6050 angle in Z-direction
  #else
      mpuAngleZ = mpu.getAngleZ();                             // Determine actual MPU6050 angle in Z-direction 
  #endif      
      mpuAngleZ = kalmanAngleZ.getFilteredValue(mpuAngleZ);    // Kalman filter noise of mpuAngleZ
      Std_Deviation_AngleZ = devAngleZ.GetStdDeviation(prevAngleZ, mpuAngleZ); // Calculate STD Deviation of new value with Previous Angle Z
      mpuAngleZ = emaAngleZ.getFilteredValue(mpuAngleZ);       // Feed AngleZ-EMA-filter with new value
      // Determine position contribution due to ---------- R O L L --------------------------------------------------------
      // Determine rollAngle contribution when leaning left (negative) or right (positive)
      // Depending on X sensor axis orientation in the enclosure Normal <-> Upside Down and/or invert the sign
  #ifdef INVERT_ROLLANGLE_SIGN
      mpuAngleX = -mpu.getAngleX();                            // Determine inverted MPU6050 angle in X-direction
  #else
      mpuAngleX = mpu.getAngleX();                             // Determine actual MPU6050 angle in X-direction 
  #endif
      mpuAngleX = kalmanAngleX.getFilteredValue(mpuAngleX);    // Kalman filter noise of mpuAngleX
      Std_Deviation_AngleX = devAngleX.GetStdDeviation(prevAngleX, mpuAngleX); // Calculate STD Deviation of new value with Previous AngleX
      mpuAngleX = emaAngleX.getFilteredValue(mpuAngleX);       // Feed AngleX-EMA-Filter with new value
  } // for measuring loop
// ------------------------------------------- PROCESS Y A W -------------------------------------------------------------------------
  if( (leftZ(mpuAngleZ-Yaw) && leftX(mpuAngleX)) || (rightZ(mpuAngleZ-Yaw) && rightX(mpuAngleX)) ) { 
      // The handle bars and the bike have turned both to the left or right -> give a follow up!
      scaleFactor = SCALEFACTOR;                                // Set Scale Factor to default
      //DEBUG_PRINTF("Span: [%03d ms] mpuAngleZ: %05.1f Dev.: %02.1f Sign: [%.0f]\n", (millis()-timeSpan), mpuAngleZ, Std_Deviation_AngleZ, sgn(mpuAngleZ));
      //DEBUG_PRINTF("Span: [%03d ms] mpuAngleX: %05.1f Dev.: %02.1f Sign: [%.0f]\n", (millis()-timeSpan), mpuAngleX, Std_Deviation_AngleX, sgn(mpuAngleX));
  } else if (Std_Deviation_AngleZ > YAWSTDDEVTHRESHOLD) {       // Exceeding threshold? Update Yaw -> compensate for fast or spontaneous drift
      //DEBUG_PRINTF("--> Yaw updated! Old: [%4.1f] New: [%4.1f] Std Dev.: [%4.1f]\n", Yaw, mpuAngleZ, Std_Deviation_AngleZ);
      Yaw = mpuAngleZ;                                          // Set Yaw to new position
      scaleFactor = 1.00;                                       // NO scaling
  } else { 
      // Typical case of "micro steering" -> (leaning XOR turning)
      scaleFactor = 1.00;                                       // NO scaling
  }
  prevAngleZ = mpuAngleZ;                                       // Set previous AngleZ value for the next round       
  yawAngle = setYawAngle(mpuAngleZ-Yaw);                        // Only relative deflection counts for steering
//---------------------------------------------PROCESS R O L L ---------------------------------------------------------------------- 
  prevAngleX = mpuAngleX;                                       // Set previous AngleX value for the next round   
  rollAngle = setRollAngle(mpuAngleX);                          // Only relative deflection counts for roll steering 
  // Check for extreme left-right bike movement and handle accordingly
  if(Std_Deviation_AngleX > ROCKINGSTDDEVTHRESHOLD) {           // Rocking -> No Steering by handlebars and/or leaning!    
      isRocking = true;
      DEBUG_PRINTLN(" --> ROCKING!");
      return 0.0;
  } else {                                                       // Turning handlebars AND Leaning are in Synch!
      isRocking = false; 
      return (yawAngle + rollAngle) * scaleFactor;               // Scale the outcome!            
  }
} // end of getSteeringState()


