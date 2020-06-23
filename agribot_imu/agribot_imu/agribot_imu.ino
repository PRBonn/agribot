/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROSSerial Node-IMU BNO055 Bosch sensor(Leonardo-arduino) %
% by: Alireza Ahmadi                                       %
% University of Bonn- MSc Robotics & Geodetic Engineering  %
% Alireza.Ahmadi@uni-bonn.de                               %
% https://github.com/PRBonn                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#define USE_USBCON //use this line just incase of using Arduino-Leonardo
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
ros::Publisher pub("imu", &imu_msg);
char frame_id[] = "imu";

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(){ 

  nh.initNode();
  nh.advertise(pub);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  imu_msg.header.seq = 0;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = frame_id;


  imu_msg.orientation.x = 0;
  imu_msg.orientation.y = 0;
  imu_msg.orientation.z = 0;
  imu_msg.orientation.w = 0;

  imu_msg.orientation_covariance[0] = 0;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;

  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0;
  imu_msg.orientation_covariance[5] = 0;

  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0;


  imu_msg.angular_velocity.x = 0;
  imu_msg.angular_velocity.y = 0;
  imu_msg.angular_velocity.z = 0;

  imu_msg.angular_velocity_covariance[0] = 0;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;

  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0;
  imu_msg.angular_velocity_covariance[5] = 0;

  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0;


  imu_msg.linear_acceleration.x = 0;
  imu_msg.linear_acceleration.y = 0;
  imu_msg.linear_acceleration.z = 0;

  imu_msg.linear_acceleration_covariance[0] = 0;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;

  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0;
  imu_msg.linear_acceleration_covariance[5] = 0;

  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0;
}

void loop(){
  /* Get a new sensor event */
  sensors_event_t event;

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> Lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro_raw = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

   
  bno.getEvent(&event);
  
  imu_msg.header.seq++;
  imu_msg.header.stamp = nh.now();

  // Doesn't work:
  pub.publish(&imu_msg);

  imu_msg.orientation.x = event.orientation.z;
  imu_msg.orientation.y = event.orientation.y;
  imu_msg.orientation.z = event.orientation.x;

//  imu_msg.orientation_covariance[0] = 0.01;
//  imu_msg.orientation_covariance[1] = 0;
//  imu_msg.orientation_covariance[2] = 0;
//
//  imu_msg.orientation_covariance[3] = 0;
//  imu_msg.orientation_covariance[4] = 0.01;
//  imu_msg.orientation_covariance[5] = 0;
//
//  imu_msg.orientation_covariance[6] = 0;
//  imu_msg.orientation_covariance[7] = 0;
//  imu_msg.orientation_covariance[8] = 0.01;

  imu_msg.angular_velocity.x = gyro_raw.x();
  imu_msg.angular_velocity.y = gyro_raw.y();
  imu_msg.angular_velocity.z = gyro_raw.z();

//  imu_msg.angular_velocity_covariance[0] = 0.01;
//  imu_msg.angular_velocity_covariance[1] = 0;
//  imu_msg.angular_velocity_covariance[2] = 0;
//
//  imu_msg.angular_velocity_covariance[3] = 0;
//  imu_msg.angular_velocity_covariance[4] = 0.01;
//  imu_msg.angular_velocity_covariance[5] = 0;
//
//  imu_msg.angular_velocity_covariance[6] = 0;
//  imu_msg.angular_velocity_covariance[7] = 0;
//  imu_msg.angular_velocity_covariance[8] = 0.01;

  imu_msg.linear_acceleration.x = Lin_acc.x();
  imu_msg.linear_acceleration.y = Lin_acc.y();
  imu_msg.linear_acceleration.z = Lin_acc.z();
  
//  imu_msg.linear_acceleration_covariance[0] = 0.01;
//  imu_msg.linear_acceleration_covariance[1] = 0;
//  imu_msg.linear_acceleration_covariance[2] = 0;
//
//  imu_msg.linear_acceleration_covariance[3] = 0;
//  imu_msg.linear_acceleration_covariance[4] = 0.01;
//  imu_msg.linear_acceleration_covariance[5] = 0;
//
//  imu_msg.linear_acceleration_covariance[6] = 0;
//  imu_msg.linear_acceleration_covariance[7] = 0;
//  imu_msg.linear_acceleration_covariance[8] = 0.01;

  nh.spinOnce();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
