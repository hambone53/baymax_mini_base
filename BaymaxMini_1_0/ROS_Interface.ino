#include <Arduino.h>

/*********************************************************************************************
*
*  ROS Interface - setup and handling of all ROS interaction to the robot.
*
*  By: Ryan Hamor
*  Date: 2/9/2016
*
*********************************************************************************************/

/********************************************************************************************
* SetupROS()
* Desc: Function to intialize ROS serial node and advertise all the topics.
*
*********************************************************************************************/
void setupROSInterface()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(botstatus);
  nh.advertise(odomPub);
  nh.subscribe(vel_cmd_sub);
  broadcaster.init(nh);
}

/********************************************************************************************
* spinROSNode()
* Desc: Function to send/receive all data for ROS serial.  This should be run with the fastest
*       ROS task.
*
*********************************************************************************************/
void spinROSNode()
{
  nh.spinOnce();
}

/********************************************************************************************
* Proc_ROS_Interface_250ms()
* Desc: Function to update all common ROS information at 500ms
*
*********************************************************************************************/
void Proc_ROS_Interface_250ms()
{
  baymax_mini_status_msg.BaseState = (uint8_t)g_BaseState;
  baymax_mini_status_msg.FrontIRSensorReading = g_FrontIRDist_CM;
  baymax_mini_status_msg.RearIRSensorReading = g_RearIRDist_CM;
  baymax_mini_status_msg.RightMotorVelocityCmdMS = (float)g_rightDriveVelocityCmd_ms;
  baymax_mini_status_msg.LeftMotorVelocityCmdMS = (float)g_leftDriveVelocityCmd_ms;
  baymax_mini_status_msg.xPos_m = (float)g_xPos_m;
  baymax_mini_status_msg.yPos_m = (float)g_yPos_m;
  baymax_mini_status_msg.zPos_rad = (float)g_theta_rad;
  baymax_mini_status_msg.xVel_ms = (float)g_xVel_ms;
  baymax_mini_status_msg.yVel_ms = (float)g_yVel_ms;
  baymax_mini_status_msg.zVel_rps = (float)g_vTheta_rad_sec;
  botstatus.publish( &baymax_mini_status_msg );
  nh.spinOnce();
}

/********************************************************************************************
* updateOdomNode()
* Desc: Function to perform and publish the odometry calcluation for the bot.
*
*********************************************************************************************/
void updateOdomNode()
{
   //g_xPos_m
   //g_yPos_m
   //g_theta_rad
   //g_xVel_ms
   //g_yVel_ms
   //g_vTheta_rad_sec

   //since all odometry is 6DOF we'll need a quaternion created from yaw
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(g_theta_rad);

   //first, we'll publish the transform over tf
   t.header.stamp = nh.now();
   t.header.frame_id = odom;
   t.child_frame_id = base_link;

   t.transform.translation.x = g_xPos_m;
   t.transform.translation.y = g_yPos_m;
   t.transform.translation.z = 0.0;
   t.transform.rotation = odom_quat;

   //Send the transform
   broadcaster.sendTransform(t);

   //Next, publish the odometry message over ROS
   odom_msg.header.stamp = nh.now();
   odom_msg.header.frame_id = odom;

   //set position
   odom_msg.pose.pose.position.x = g_xPos_m;
   odom_msg.pose.pose.position.y = g_yPos_m;
   odom_msg.pose.pose.position.z = 0.0;
   odom_msg.pose.pose.orientation = odom_quat;

   //set the velocity
   odom_msg.child_frame_id = base_link;
   odom_msg.twist.twist.linear.x = g_xVel_ms;
   odom_msg.twist.twist.linear.y = g_yVel_ms;
   odom_msg.twist.twist.linear.z = 0.0;
   odom_msg.twist.twist.angular.x = 0.0;
   odom_msg.twist.twist.angular.y = 0.0;
   odom_msg.twist.twist.angular.z = g_vTheta_rad_sec;

   //publish the message
   odomPub.publish( &odom_msg );

   nh.spinOnce();

}
