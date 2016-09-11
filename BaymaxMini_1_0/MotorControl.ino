#include <Arduino.h>

/*********************************************************
*  Motor control software to drive front end and calculate position and velocity
*  information from quad encoders.
*
*  By: Ryan Hamor
*  Data: 1/5/2016
***********************************************************/

//Includes
#include "MotorControl.h"

//Define constant motor hardware parameters
#define ENCODER_PPR                 400
#define MOTOR_GEAR_RATION           30
#define ENCODER_PULSE_PER_METER     30641
#define WHEEL_BASE_IN_M             0.250
#define WHEEL_RADIUS_M              0.060
#define BASE_DRIVE_PGAIN            200.0
#define BASE_DRIVE_IGAIN            400.0
#define BASE_DRIVE_DGAIN            0.0
#define BASE_DRIVE_MAX              80.0
#define BASE_DRIVE_MIN              -80.0
#define BASE_DRIVE_TS_MS            40
#define FEED_FWD_PERCENT            0.8
#define SABER_CMD_DEAD_ZONE         8
#define MAX_SABER_CMD               120
#define MIN_LINEAR_VEL_MS           0.2

/***********************************************************************************************
*
*  initMotorControl()
*
*  Description: Function to initialize motor control PID Loops and other misc
*
***********************************************************************************************/
void initMotorControl()
{
  //Setup the PID parameters for the right and left drives
  g_RightSidePID.SetMode(MANUAL);
  g_RightSidePID.SetOutputLimits(BASE_DRIVE_MIN, BASE_DRIVE_MAX);
  g_RightSidePID.SetTunings(BASE_DRIVE_PGAIN, BASE_DRIVE_IGAIN, BASE_DRIVE_DGAIN);
  g_RightSidePID.SetSampleTime(BASE_DRIVE_TS_MS);

  g_LeftSidePID.SetMode(MANUAL);
  g_LeftSidePID.SetOutputLimits(BASE_DRIVE_MIN, BASE_DRIVE_MAX);
  g_LeftSidePID.SetTunings(BASE_DRIVE_PGAIN, BASE_DRIVE_IGAIN, BASE_DRIVE_DGAIN);
  g_LeftSidePID.SetSampleTime(BASE_DRIVE_TS_MS);

}

void readEncoderTest()
{

  //Serial.print("Right Wheel Position: ");Serial.print(rightFrontWheelEncoder.read());
 //Serial.print("\tLeft Wheel Position: ");Serial.println(leftFrontWheelEncoder.read());

}

void calculateVelAndPos(unsigned long tasktime)
{

  float leftEncoderCnts = (float)leftFrontWheelEncoder.read();
  float rightEncoderCnts = (float)rightFrontWheelEncoder.read();
  float f32_tasktime = (float)tasktime / 1000;

  (void) calcPosition(leftEncoderCnts, rightEncoderCnts);

  //Single Kalman Filter Perseistents
  static KalmanVars leftMotorKVars;
  static KalmanVars rightMotorKVars;



  //Calc M-method velocities. Note: we have zeroed out encoder reads each time so no need to subtract.
  leftMotor_RPM = (60 * leftEncoderCnts) / (ENCODER_PPR * f32_tasktime);
  rightMotor_RPM = (60 * rightEncoderCnts) / (ENCODER_PPR * f32_tasktime);

  //Get Velocity Estimates through single dimensional kalman filter
  leftMotorKVars = EstimateVelocity(leftMotorKVars, f32_tasktime, leftMotor_RPM);
  rightMotorKVars = EstimateVelocity(rightMotorKVars, f32_tasktime, rightMotor_RPM);

  leftMotor_RPM = leftMotorKVars.FilteredVelocity;
  rightMotor_RPM = rightMotorKVars.FilteredVelocity;

  g_rightVelocity_ms = (double)((rightMotor_RPM * ENCODER_PPR) / 60) / ENCODER_PULSE_PER_METER;
  g_leftVelocity_ms = (double)((leftMotor_RPM * ENCODER_PPR) / 60) / ENCODER_PULSE_PER_METER;

  (void) updateXYVelocityEstimate();

  //Reset Encoders for next read
  rightFrontWheelEncoder.write(0);
  leftFrontWheelEncoder.write(0);

}

/*********************************************************************************************
*
* Description: This function updates an estimated x and y for the base unit.
*
*********************************************************************************************/
void calcPosition(float leftCnts, float rightCnts)
{
  float leftMeters = leftCnts/ENCODER_PULSE_PER_METER;
  float rightMeters = rightCnts/ENCODER_PULSE_PER_METER;
  float avgMeters = (leftMeters + rightMeters) * 0.5;

  g_xPos_m += avgMeters * cos(g_theta_rad);
  g_yPos_m += avgMeters * sin(g_theta_rad);
}

/*********************************************************************************************
*
* Description: This function updates an estimated x and y velocity for the base unit.
*
*********************************************************************************************/
void updateXYVelocityEstimate()
{
  float avgVelocityMS = (g_rightVelocity_ms + g_leftVelocity_ms) * 0.5;

  g_xVel_ms = avgVelocityMS * cos(g_theta_rad);
  g_yVel_ms = avgVelocityMS * sin(g_theta_rad);
  
}

/*********************************************************************************************
*
* DriveControl_Step
*
* Desc: This function when called computes the new base drive commands for left and right drives.
*
**********************************************************************************************/
void DriveControl_Step()
{

  int8_t RightMotorCmd = 0;
  int8_t LeftMotorCmd = 0;

  g_LeftSidePID.SetMode(AUTOMATIC);
  g_RightSidePID.SetMode(AUTOMATIC);

  //Compute new setpoints
  g_rightDriveVelocityCmd_ms = (g_LinearVelocityChecked + g_AngularVelocityChecked * WHEEL_BASE_IN_M / 2.0); // WHEEL_RADIUS_M;
  g_leftDriveVelocityCmd_ms = (g_LinearVelocityChecked - g_AngularVelocityChecked * WHEEL_BASE_IN_M / 2.0); // WHEEL_RADIUS_M;

  //Apply Deadzone check on velocity to make sure we move if commanded
  g_rightDriveVelocityCmd_ms = DeadZoneVelocity(g_rightDriveVelocityCmd_ms);
  g_leftDriveVelocityCmd_ms = DeadZoneVelocity(g_leftDriveVelocityCmd_ms);

  //Compute feedforward command
  g_RightDriveFeedFwdCmd = FEED_FWD_PERCENT * (g_rightDriveVelocityCmd_ms * 100);
  g_LeftDriveFeedFwdCmd = FEED_FWD_PERCENT * (g_leftDriveVelocityCmd_ms * 100);

  g_LeftSidePID.Compute();
  g_RightSidePID.Compute();

  //Handle the fact that the drives won't really move unless we have some deadzone we take care of.
  RightMotorCmd = g_rightDriveSaberCmd + g_RightDriveFeedFwdCmd;
  LeftMotorCmd = g_leftDriveSaberCmd + g_LeftDriveFeedFwdCmd;

  //Limit Commands to saber controller
  LeftMotorCmd = LimitSaberCmd( LeftMotorCmd );
  RightMotorCmd = LimitSaberCmd( RightMotorCmd );

  //Output Final Commands to Saber motor controller
  ST.motor(1, RightMotorCmd);
  ST.motor(2, LeftMotorCmd);

}

/*********************************************************************************************
*
* LimitSaberCmd
*
* Desc: This func will saturate saber motor controller commands such that they don't cause it
*       to go open loop.
*
**********************************************************************************************/
int8_t LimitSaberCmd( int8_t Cmd )
{

  if (Cmd < -MAX_SABER_CMD)
  {
    Cmd = -MAX_SABER_CMD;
  }
  else if (Cmd > MAX_SABER_CMD)
  {
    Cmd = MAX_SABER_CMD;
  }

  return Cmd;

}

/*********************************************************************************************
*
* DeadZoneVelocity
*
* Desc: This func will apply a deadzone where commands that are non-zero but less than the min
*        are set to the minimum speed the system is capable of.
*
**********************************************************************************************/
double DeadZoneVelocity( double Cmd )
{
   double DeadZoneCheckedCmd = 0.0;

   //1. Check Not Zero
   if ((Cmd < -0.0001) || (Cmd > 0.0001))
   {
      //Negative Command
      if ( Cmd < -0.0001 )
      {
         if ( Cmd > -MIN_LINEAR_VEL_MS )
         {
            DeadZoneCheckedCmd = -MIN_LINEAR_VEL_MS;
         }
         else
         {
            DeadZoneCheckedCmd = Cmd;
         }
      }
      //Positive Cmd
      else
      {
         if (Cmd < MIN_LINEAR_VEL_MS )
         {
            DeadZoneCheckedCmd = MIN_LINEAR_VEL_MS;
         }
         else
         {
            DeadZoneCheckedCmd = Cmd;
         }
      }
   }
   else
   {
      DeadZoneCheckedCmd = 0.0;
   }


  return DeadZoneCheckedCmd;

}
