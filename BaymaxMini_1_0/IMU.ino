#include <Arduino.h>

/****************************************************************
*
*  IMU Code for Baymax Mini using Adafruit BNO055 board
*
*  By: Ryan Hamor
*  Data: 1/8/2016
*
*****************************************************************/
//#define PI   3.14159265359
//#define TWOPI  2*PI


void setupIMU() {

  Adafruit_BNO055::bno055_units_type_t unitSelect;

  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  /* Initialise the sensor */
  if(!bno.begin(bno.OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  //wait for gyro to calibrate
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* Change the units to give out angles in radians instead of degrees */
  unitSelect.accUnit = bno.ACC_UNIT_MS2;
  unitSelect.gyrUnit = bno.GYR_UNIT_RPS;
  unitSelect.eulUnit = bno.EUL_UNIT_RAD;
  unitSelect.tempUnit = bno.TEMP_UNIT_C;
  unitSelect.oriUnit = bno.ORI_UNIT_WIN;
  bno.setUnits(unitSelect);

  delay(1000);
  bno.setExtCrystalUse(true);

}

void IMURead() {

    // Quaternion data
  /*imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");*/

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Invert the theta from the IMU to follow the right hand rule of the robot where left is positive turn
  // this could also be done by configuring the IMU to switch the sign of z.
  g_theta_rad = (2 * PI) - euler.z();
  g_vTheta_rad_sec = gyro.z();

  /* Display the floating point data */
  /*Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");*/

  /* Display calibration status for each sensor. */
  /*uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);*/


}
