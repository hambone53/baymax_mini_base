#include <Arduino.h>

/*********************************************************************************************
*
*  BaseStateController - Responsible for protecting the base unit.  It reads on base sensors
*                        and can make decisions to override commands from the main processor.
*
*  By: Ryan Hamor
*  Date: 3/3/2016
*
*********************************************************************************************/
#define EDGE_DIST_THRESH_CM      15.00
#define EDGE_DETECT_TIMEOUT_MS   80
#define DANGER_STOP_TIMEROUT_MS  500
#define CLEAR_DANGER_TIMEOUT_MS  10000
#define DANGER_CLEAR_DIST_M      0.25
#define REVERSE_DANGER_SPEED_MS  -0.3
#define FORWARD_DANGER_SPEED_MS  0.3
#define MAX_IR_RANGE_CM          100

//Base State Definitions
const int POST_STATE                = 0;
const int NORMAL_RUNTIME            = 1;
const int DANGER_DETECTED_STOP      = 2;
const int AVOID_EDGE_BACK_UP        = 3;
const int AVOID_EDGE_MOVE_FORWARD   = 4;


/***********************************************************************************************
*
*  read_gp2d12_range_cm(byte pin)
*
*  Description: function to read and return the value from the gp2d12 IR sensor in cm.
*
***********************************************************************************************/
void readIRSensors( unsigned long tasktime )
{
   static unsigned long FrontTimer = 0;
   static unsigned long RearTimer = 0;

   g_FrontIRDist_CM = read_gp2d12_range_cm(0);
   g_RearIRDist_CM = read_gp2d12_range_cm(1);

   //Saturate the sensor readings since we don't care...
   if(g_FrontIRDist_CM > MAX_IR_RANGE_CM  ){
      g_FrontIRDist_CM = MAX_IR_RANGE_CM;
   }

   if(g_RearIRDist_CM > MAX_IR_RANGE_CM  ){
      g_RearIRDist_CM = MAX_IR_RANGE_CM;
   }

   //Check both edges and notify when one of them is detected.
   if( g_FrontIRDist_CM > EDGE_DIST_THRESH_CM )
   {
      if ( FrontTimer >= EDGE_DETECT_TIMEOUT_MS )
      {
         g_FrontEdgeDetect = true;
      }
      else
      {
         FrontTimer += tasktime;
         g_FrontEdgeDetect = false;
      }
   }
   else
   {
      FrontTimer = 0;
      g_FrontEdgeDetect = false;
   }

   if( g_RearIRDist_CM > EDGE_DIST_THRESH_CM )
   {
      if ( RearTimer >= EDGE_DETECT_TIMEOUT_MS )
      {
         g_RearEdgeDetect = true;
      }
      else
      {
         RearTimer += tasktime;
         g_RearEdgeDetect = false;
      }
   }
   else
   {
      RearTimer = 0;
      g_RearEdgeDetect = false;
   }

   //Print out position
   /*Serial.print("Front: ");
   Serial.print(g_FrontIRDist_CM);
   Serial.print(" Rear: ");
   Serial.print(g_RearIRDist_CM);
   Serial.print(" Right Vel Cmd: ");
   Serial.print(g_rightDriveVelocityCmd_ms);
   Serial.print(" Left Vel Cmd: ");
   Serial.print(g_leftDriveVelocityCmd_ms);
   Serial.print(" Base State: ");
   Serial.println(g_BaseState);*/
}

/***********************************************************************************************
*
*  read_gp2d12_range_cm(byte pin)
*
*  Description: function to read and return the value from the gp2d12 IR sensor in cm.
*
***********************************************************************************************/
float read_gp2d12_range_cm(byte pin) {
	int tmp;

	tmp = analogRead(pin);
	if (tmp < 3)
		return -1; // invalid value

	return (6787.0 /((float)tmp - 3.0)) - 4.0;
}

/***********************************************************************************************
*
*  void base_step()
*
*  Description: function to run the main low level base statemachine for baymax-mini.  Main
*                responsibilities include:
*
*                  - Detect possible base damaging situations and don't allow damage.
*                  - limit the velocity commands to the minimum velocity.
*
***********************************************************************************************/
void base_step(unsigned long tasktime)
{
   static int state = POST_STATE;
   static boolean IR_EdgeSensorsOk = false;
   static unsigned long BaseStateTimer = 0;
   static float xPosStartAvoidRoutine = 0.0;

   switch (state)
   {
      //This is the power up state we will check here that any sensors needed for operation are online
      case POST_STATE:
         //Make sure to send out zero to the motor command until in normal operation.
         g_LinearVelocityChecked = 0.0;
         g_AngularVelocityChecked = 0.0;

         if (  (g_FrontIRDist_CM > 0.0) && (g_FrontIRDist_CM < EDGE_DIST_THRESH_CM) &&
               (g_RearIRDist_CM > 0.0) && (g_RearIRDist_CM < EDGE_DIST_THRESH_CM)   )
         {
               IR_EdgeSensorsOk = true;
         }

         if (IR_EdgeSensorsOk)
         {
            state = NORMAL_RUNTIME;
         }

         break;

      //This is the normal runtime state.  Here we simply pass on the commands to the base.
      case NORMAL_RUNTIME:

         g_LinearVelocityChecked = g_LinearVelocity;
         g_AngularVelocityChecked = g_AngularVelocity;

         /*Check for edge found. For now we will check both front and back but if one goes over
         the threshold then that is the one we will correct for; however, westill need to keep
         checking incase we get the other one. */
         if ( g_FrontEdgeDetect || g_RearEdgeDetect )
         {
            g_LinearVelocityChecked = 0.0;
            g_AngularVelocityChecked = 0.0;
            state = DANGER_DETECTED_STOP;
         }

         break;

      //This is the case where a dangerous edge is first detected we just want to stop for a moment
      //and stabilize the base.
      case DANGER_DETECTED_STOP:

         g_LinearVelocityChecked = 0.0;
         g_AngularVelocityChecked = 0.0;

         if ( BaseStateTimer >= DANGER_STOP_TIMEROUT_MS )
         {
            if ( g_FrontEdgeDetect )
            {
               state = AVOID_EDGE_BACK_UP;
               BaseStateTimer = 0;
               xPosStartAvoidRoutine = g_xPos_m;
               break;
            }
            else if( g_RearEdgeDetect )
            {
               state = AVOID_EDGE_MOVE_FORWARD;
               BaseStateTimer = 0;
               xPosStartAvoidRoutine = g_xPos_m;
               break;
            }
            else
            {
               state = POST_STATE;
               BaseStateTimer = 0;
               break;
            }
         }

         BaseStateTimer += tasktime;
         break;

      //This state is once we have detected danger we need to back away for so many meters.
      case AVOID_EDGE_BACK_UP:

         //Start commanding velocity in reverse
         g_LinearVelocityChecked = REVERSE_DANGER_SPEED_MS;
         g_AngularVelocityChecked = 0.0;

         if( abs(g_xPos_m - xPosStartAvoidRoutine) >= DANGER_CLEAR_DIST_M )
         {
            g_LinearVelocityChecked = 0.0;
            g_AngularVelocityChecked = 0.0;
            BaseStateTimer = 0;
            state = POST_STATE;
         }
         else if( BaseStateTimer >= CLEAR_DANGER_TIMEOUT_MS )
         {
            g_LinearVelocityChecked = 0.0;
            g_AngularVelocityChecked = 0.0;
            BaseStateTimer = 0;
            state = POST_STATE;
         }
         else
         {
            BaseStateTimer += tasktime;
         }

         break;

      //This state is once we have dtected danger and we need to move forward from it.
      case AVOID_EDGE_MOVE_FORWARD:

         //Starte commanding velocity forward
         g_LinearVelocityChecked = FORWARD_DANGER_SPEED_MS;
         g_AngularVelocityChecked = 0.0;

         if( abs(g_xPos_m - xPosStartAvoidRoutine) >= DANGER_CLEAR_DIST_M )
         {
            g_LinearVelocityChecked = 0.0;
            g_AngularVelocityChecked = 0.0;
            BaseStateTimer = 0;
            state = POST_STATE;
         }
         else if( BaseStateTimer >= CLEAR_DANGER_TIMEOUT_MS )
         {
            g_LinearVelocityChecked = 0.0;
            g_AngularVelocityChecked = 0.0;
            BaseStateTimer = 0;
            state = POST_STATE;
         }
         else
         {
            BaseStateTimer += tasktime;
         }

         break;

      default:
         break;
   }

   //Update the global var for state machine for debug.
   g_BaseState = state;
}
