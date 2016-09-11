#include <Arduino.h>

/*  TaskScheduler Test sketch - use of task IDs and watchdog timer to identify hung tasks
 *  Test case:
 *    Watchdog timer is set to 2 seconds (interrupt + reset)
 *    A hearbeat task (resetting the watchdog timer) is scheduled with 500 ms interval
 *    A number of tasks are running every 1 second and "rolling the dice" 0..19.  If 5, task is made to enter infinite loop
 *    Device should reset in 2 seconds after a task enters infinite loop
 *    A task id and a control point number are saved to EEPROM prior to device reset, and are displayed after reboot.
 *    In real life, device might chose to NOT activate certain tasks which failed previously (failed sensors for instance)
 */

//#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_WDT_IDS
#include <TaskScheduler.h>

#include <EEPROM.h>
#include <avr/wdt.h>
#include <SabertoothSimplified.h>

#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <PID_v1.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <baymax_mini_msgs/BaymaxMiniStatus.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

//Setup the global ROS handles and objects
ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel_msg;

baymax_mini_msgs::BaymaxMiniStatus baymax_mini_status_msg;
ros::Publisher botstatus("botstatus", &baymax_mini_status_msg);

nav_msgs::Odometry odom_msg;
ros::Publisher odomPub("odom", &odom_msg);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";

//Set up motor driver object
SabertoothSimplified ST;
//Set up task controller object
Scheduler ts;
//Set up wheel encoder objects
Encoder rightFrontWheelEncoder(2,4);
Encoder leftFrontWheelEncoder(3,5);

//Setup the IMU
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

//Temporary globals location until I figure out what todo:
float leftMotor_RPM = 0;
float rightMotor_RPM = 0;
uint8_t g_GyroCalibrated = 0;

// Pose estimate vars.
float g_xPos_m = 0;
float g_yPos_m = 0;
float g_theta_rad = 0;
double g_rightVelocity_ms = 0;
double g_leftVelocity_ms = 0;
float g_xVel_ms = 0;
float g_yVel_ms = 0;
float g_vTheta_rad_sec = 0;

//Motor controller variables
double g_rightDriveSaberCmd = 0;
double g_leftDriveSaberCmd = 0;
double g_rightDriveVelocityCmd_ms = 0;
double g_leftDriveVelocityCmd_ms = 0;
double g_LinearVelocity = 0.0;
double g_AngularVelocity = 0.0;
double g_RightDriveFeedFwdCmd = 0.0;
double g_LeftDriveFeedFwdCmd = 0.0;
double g_LinearVelocityChecked = 0.0;
double g_AngularVelocityChecked = 0.0;

//Base state machine variables that are global
int g_BaseState = 0;
boolean g_FrontEdgeDetect = false;
boolean g_RearEdgeDetect = false;

PID g_RightSidePID(&g_rightVelocity_ms, &g_rightDriveSaberCmd, &g_rightDriveVelocityCmd_ms, 0, 0, 0, DIRECT);
PID g_LeftSidePID(&g_leftVelocity_ms, &g_leftDriveSaberCmd, &g_leftDriveVelocityCmd_ms, 0, 0, 0, DIRECT);


//Setup ROS Subscriber information
void cmd_vel_Callback(const geometry_msgs::Twist& cmd_msg)
{
  g_LinearVelocity = cmd_msg.linear.x;
  g_AngularVelocity = cmd_msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> vel_cmd_sub("cmd_vel", cmd_vel_Callback);

//Base Controller Global Vars
float g_FrontIRDist_CM = 0;
float g_RearIRDist_CM = 0;


// Callback methods prototypes
void TaskCB();
void HB(); bool HBOn(); void HBOff();
void HB(); bool HBOn(); void HBOff();

// Three tasks emulating accidental infinite loop
Task tMotorControlTask(40, TASK_FOREVER, &MotorControl_Task, &ts, true);
Task tIMUProcessing(20, TASK_FOREVER, &IMUProcessing_Task, &ts, true);
Task tEncoderReadTask(20, TASK_FOREVER, &EncoderRead_Task, &ts, true);
Task tDebugTask(300, TASK_FOREVER, &Debug_Task, &ts, true);
Task tIRSensorTask(40, TASK_FOREVER, &ReadIRSensor_Task, &ts, true);
Task tBaseStateMachine(100, TASK_FOREVER, &BaseState_Task, &ts, true);
Task tRosInterface50ms(50, TASK_FOREVER, &ROS_Interface_50ms_Task, &ts, true);
Task tRosInterface100ms(100, TASK_FOREVER, &ROS_Interface_100ms_Task, &ts, true);
Task tRosInterface250ms(250, TASK_FOREVER, &ROS_Interface_250ms_Task, &ts, true);

// Heartbeat task - resetting the watchdog timer periodically
// Initiates WDT on enable, and deactivates it on disable
Task tHB(500, TASK_FOREVER, &HB, &ts, false, &HBOn, &HBOff);

/**
 * Motor Control Task Callback
 */
void MotorControl_Task() {

  DriveControl_Step();

}

/********************************
 * Encoder Read Task
 ********************************/
 void EncoderRead_Task() {
   Task& T = ts.currentTask();
   calculateVelAndPos(T.getInterval());
   //readEncoderTest();
 }

void IMUProcessing_Task() {
  IMURead();
}

/********************************************************************************************
*
* Desc Debug_Task
*
**********************************************************************************************/
void Debug_Task()
{
  DebugPrint();
}

/********************************************************************************************
*
* Desc ROS Serial Task
*
**********************************************************************************************/
void ROS_Interface_50ms_Task()
{
  spinROSNode();
}

/********************************************************************************************
*
* Desc ROS Serial Task
*
**********************************************************************************************/
void ROS_Interface_100ms_Task()
{
  updateOdomNode();
}

/********************************************************************************************
*
* Desc ROS Serial Task 250ms
*
**********************************************************************************************/
void ROS_Interface_250ms_Task()
{
  Proc_ROS_Interface_250ms();
}

/********************************************************************************************
*
* Desc IR Edge Detection Task 40ms
*
**********************************************************************************************/
void ReadIRSensor_Task()
{
   Task& T = ts.currentTask();
  readIRSensors(T.getInterval());
}

/********************************************************************************************
*
* Desc Base State Machine task at 100ms
*
**********************************************************************************************/
void BaseState_Task()
{
   Task& T = ts.currentTask();
   base_step(T.getInterval());
}

/**
 * This On Enable method sets up the WDT
 * for interrupt and reset after 2 seconds
 */
bool HBOn() {

  //disable interrupts
  cli();
  //reset watchdog
  wdt_reset();
  //set up WDT interrupt
  WDTCSR = (1<<WDCE)|(1<<WDE);
  //Start watchdog timer with aDelay prescaller
  WDTCSR = (1<<WDIE)|(1<<WDE)|(WDTO_2S & 0x2F);
//  WDTCSR = (1<<WDIE)|(WDTO_2S & 0x2F);  // interrupt only without reset
  //Enable global interrupts
  sei();
}

/**
 * This On Disable method disables WDT
 */
void HBOff() {
  wdt_disable();
}

/**
 * This is a periodic reset of WDT
 */
void HB() {
  wdt_reset();
}

/**
 * Watchdog timeout ISR
 *
 */
ISR(WDT_vect)
{
  Task& T = ts.currentTask();

  digitalWrite(13, HIGH);
  EEPROM.write(0, (byte)T.getId());
  EEPROM.write(1, (byte)T.getControlPoint());
  digitalWrite(13, LOW);
}

/**
 * Standard arduino setup routine
 */
void setup() {

  //Serial.begin(19200);
  SabertoothTXPinSerial.begin(19200); // This is the baud rate you chose with the DIP switches.

  //Startup ROS interface
  setupROSInterface();

  setupIMU();

  initMotorControl();

  //Start up watchdog ping
  tHB.enableDelayed();

}

/**
 * Not much is left for the loop()
 */
void loop() {
  ts.execute();
}
