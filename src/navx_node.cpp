/*
 * navx_node.cpp
 * Runs the Kauai Labs NavX, using modified NavX library
 * VERSION: 1.0.0
 * Last changed: 2019-10-31
 * Authors: Jude Sauve <sauve031@umn.edu>
 * Maintainers: Nick Schatz <schat127@umn.edu>
 * MIT License
 * Copyright (c) 2019 UMN Robotics
 */

/* Interface: 
 *  Pub: 
 *   imu/data (sensor_msgs/Imu)
 *   imu/euler (geometry_msgs/Point)
 * Param: 
 *   frequency (double) 50.0; The frequency of the read loop
 *   publish_euler (bool) false; Whether to publish euler orientation
 *   device_path (string) /dev/ttyACM0; The device serial port path
 *   frame_id (string) imu_link; The Imu message header frame ID
 *   covar_samples (int) 100; The number of samples to store to calculate covariance
 */

// ROS Libs
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include "boost/array.hpp"

// Native_Libs
#include <string>
#include <cmath>

// Custom_Libs
#include "ahrs/AHRS.h"

static const float DEG_TO_RAD = M_PI / 180.0F;
static const float GRAVITY = 9.81F; // m/s^2, if we actually go to the moon remember to change this
typedef struct {
  float ypr[3];
  float ang_vel[3];
  float accel[3];
} OrientationEntry;

// Function Prototypes
bool calculate_covariance(boost::array<double, 9> &orientation_mat, 
                          boost::array<double, 9> &ang_vel_mat, 
                          boost::array<double, 9> &accel_mat);

// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher imu_pub;
ros::Publisher euler_pub;

// ROS Callbacks
void update_callback(const ros::TimerEvent&);

// ROS Params
double frequency;
bool publish_euler;
std::string device_path;
std::string frame_id;
int covar_samples;

// Global_Vars
AHRS* com;
int seq = 0;
std::vector<OrientationEntry> orientationHistory;

int main(int argc, char** argv) {
  // Init ROS
  ros::init(argc, argv, "navx_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");

  // Params
  pnh->param<double>("frequency", frequency, 50.0);
  pnh->param<bool>("publish_euler", publish_euler, false);
  pnh->param<std::string>("device_path", device_path, "/dev/ttyACM0");
  pnh->param<std::string>("frame_id", frame_id, "imu_link");
  pnh->param<int>("covar_samples", covar_samples, 100);

  // Init IMU
  com = new AHRS(device_path);
  orientationHistory.resize(covar_samples);

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);

  // Publishers
  imu_pub = nh->advertise<sensor_msgs::Imu>("imu/data", 10);
  euler_pub = nh->advertise<geometry_msgs::Point>("imu/euler", 10);

  // Spin
  ros::spin();
}

/**
 * Calculates the covariance matrices based on the orientation history and stores the results in the provided arrays
 * Returns true if the returned covariance is valid, otherwise false
 */
bool calculate_covariance(boost::array<double, 9> &orientation_mat, 
                          boost::array<double, 9> &ang_vel_mat, 
                          boost::array<double, 9> &accel_mat) {
  int count = std::min(seq-1, covar_samples);
  if (count < 2) {
    return false; // Did not calculate covariance
  }
  OrientationEntry avg = {};
  // Calculate averages
  for (int i = 0; i < count; i++) {
    for (int j = 0; j < 3; j++) {
      avg.ypr[j] += orientationHistory[i].ypr[j];
      avg.ang_vel[j] += orientationHistory[i].ang_vel[j];
      avg.accel[j] += orientationHistory[i].accel[j];
    }
  }
  for (int j = 0; j < 3; j++) {
    avg.ypr[j] /= count;
    avg.ang_vel[j] /= count;
    avg.accel[j] /= count;
  }
  // Calculate covariance
  // See https://en.wikipedia.org/wiki/Covariance#Calculating_the_sample_covariance
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      int idx = 3*x + y;
      orientation_mat[idx] = 0;
      ang_vel_mat[idx] = 0;
      accel_mat[idx] = 0;
      // Average mean error difference
      for (int i = 0; i < count; i++) {
        orientation_mat[idx] += (orientationHistory[i].ypr[x] - avg.ypr[x]) * (orientationHistory[i].ypr[y] - avg.ypr[y]);
        ang_vel_mat[idx] += (orientationHistory[i].ang_vel[x] - avg.ang_vel[x]) * (orientationHistory[i].ang_vel[y] - avg.ang_vel[y]);
        accel_mat[idx] += (orientationHistory[i].accel[x] - avg.accel[x]) * (orientationHistory[i].accel[y] - avg.accel[y]);
      }
      // Normalize
      orientation_mat[idx] /= count - 1;
      ang_vel_mat[idx] /= count - 1;
      accel_mat[idx] /= count - 1;
    }
  }
  return true;
}

void update_callback(const ros::TimerEvent&) {
  // Calculate pose
  OrientationEntry curOrientation;
  curOrientation.ypr[0] = com->GetRoll() * DEG_TO_RAD;
  curOrientation.ypr[1] = com->GetPitch() * DEG_TO_RAD;
  curOrientation.ypr[2] = com->GetYaw() * DEG_TO_RAD;
  curOrientation.ang_vel[0] = com->GetRollRate() * DEG_TO_RAD;
  curOrientation.ang_vel[1] = com->GetPitchRate() * DEG_TO_RAD;
  curOrientation.ang_vel[2] = com->GetYawRate() * DEG_TO_RAD;
  curOrientation.accel[0] = com->GetWorldLinearAccelX() * GRAVITY;
  curOrientation.accel[1] = com->GetWorldLinearAccelY() * GRAVITY;
  curOrientation.accel[2] = com->GetWorldLinearAccelZ() * GRAVITY;

  orientationHistory[seq % covar_samples] = curOrientation;

  // Publish IMU message

  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.seq = seq++;
  msg.header.frame_id = frame_id;
  
  msg.orientation.x = com->GetQuaternionX();
  msg.orientation.y = com->GetQuaternionY();
  msg.orientation.z = com->GetQuaternionZ();
  msg.orientation.w = com->GetQuaternionW();
  
  msg.angular_velocity.x = curOrientation.ang_vel[0];
  msg.angular_velocity.y = curOrientation.ang_vel[1];
  msg.angular_velocity.z = curOrientation.ang_vel[2];
  
  msg.linear_acceleration.x = curOrientation.accel[0];
  msg.linear_acceleration.y = curOrientation.accel[1];
  msg.linear_acceleration.z = curOrientation.accel[2];

  if (calculate_covariance(msg.orientation_covariance, 
                           msg.angular_velocity_covariance, 
                           msg.linear_acceleration_covariance)) {
    // Only publish a message if we have a valid covariance
    imu_pub.publish(msg);
  }

  if (publish_euler) {
    // Publish Euler message
    geometry_msgs::Point euler;
    euler.x = com->GetRoll();
    euler.y = com->GetPitch();
    euler.z = com->GetYaw();
    euler_pub.publish(euler);
  }
}
