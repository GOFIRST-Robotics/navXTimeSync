/*
 * navx_node.cpp
 * Runs the Kauai Labs NavX, using modified NavX library
 * VERSION: 1.0.0
 * Last changed: 2019-10-05
 * Authors: Jude Sauve <sauve031@umn.edu>
 * Maintainers: Nick Schatz <schat127@umn.edu>
 * MIT License
 * Copyright (c) 2019 UMN Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>

// Native_Libs
#include <string>

// Custom_Libs
#include "ahrs/AHRS.h"

#define DEG_TO_RAD ((2.0 * 3.14159) / 360.0)
#define GRAVITY 9.81 // m/s^2, if we actually go to the moon remember to change this
typedef struct {
  float ypr[3];
  float ang_vel[3];
  float accel[3];
} Pose;

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
std::vector<Pose> poseHistory;

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
  poseHistory.resize(covar_samples);

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);

  // Publishers
  imu_pub = nh->advertise<sensor_msgs::Imu>("imu/data", 10);
  euler_pub = nh->advertise<geometry_msgs::Point>("imu/euler", 10);

  // Spin
  ros::spin();
}

void calculate_covariance(boost::array<double, 9> &orientation_mat, boost::array<double, 9> &ang_vel_mat, boost::array<double, 9> &accel_mat) {
  int count = std::min(seq-1, covar_samples);
  if (count < 2) {
    return;
  }
  Pose avg = {};
  // Calculate averages
  for (int i = 0; i < count; i++) {
    for (int j = 0; j < 3; j++) {
      avg.ypr[j] += poseHistory[i].ypr[j];
      avg.ang_vel[j] += poseHistory[i].ang_vel[j];
      avg.accel[j] += poseHistory[i].accel[j];
    }
  }
  for (int j = 0; j < 3; j++) {
    avg.ypr[j] /= count;
    avg.ang_vel[j] /= count;
    avg.accel[j] /= count;
  }
  // Calculate covariance
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < 3; y++) {
      int idx = 3*x + y;
      orientation_mat[idx] = 0;
      ang_vel_mat[idx] = 0;
      accel_mat[idx] = 0;
      // Average mean error difference
      for (int i = 0; i < count; i++) {
        orientation_mat[idx] += (poseHistory[i].ypr[x] - avg.ypr[x]) * (poseHistory[i].ypr[y] - avg.ypr[y]);
        ang_vel_mat[idx] += (poseHistory[i].ang_vel[x] - avg.ang_vel[x]) * (poseHistory[i].ang_vel[y] - avg.ang_vel[y]);
        accel_mat[idx] += (poseHistory[i].accel[x] - avg.accel[x]) * (poseHistory[i].accel[y] - avg.accel[y]);
      }
      // Normalize
      orientation_mat[idx] /= count - 1;
      ang_vel_mat[idx] /= count - 1;
      accel_mat[idx] /= count - 1;
    }
  }

}

void update_callback(const ros::TimerEvent&) {
  // Calculate pose
  Pose curPose;
  curPose.ypr[0] = com->GetRoll() * DEG_TO_RAD;
  curPose.ypr[1] = com->GetPitch() * DEG_TO_RAD;
  curPose.ypr[2] = com->GetYaw() * DEG_TO_RAD;
  curPose.ang_vel[0] = com->GetRollRate() * DEG_TO_RAD;
  curPose.ang_vel[1] = com->GetPitchRate() * DEG_TO_RAD;
  curPose.ang_vel[2] = com->GetYawRate() * DEG_TO_RAD;
  curPose.accel[0] = com->GetWorldLinearAccelX() * GRAVITY;
  curPose.accel[1] = com->GetWorldLinearAccelY() * GRAVITY;
  curPose.accel[2] = com->GetWorldLinearAccelZ() * GRAVITY;

  poseHistory[seq % covar_samples] = curPose;

  // Publish IMU message

  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.seq = seq++;
  msg.header.frame_id = frame_id;
  
  msg.orientation.x = com->GetQuaternionX();
  msg.orientation.y = com->GetQuaternionY();
  msg.orientation.z = com->GetQuaternionZ();
  msg.orientation.w = com->GetQuaternionW();
  
  msg.angular_velocity.x = curPose.ang_vel[0];
  msg.angular_velocity.y = curPose.ang_vel[1];
  msg.angular_velocity.z = curPose.ang_vel[2];
  
  msg.linear_acceleration.x = curPose.accel[0];
  msg.linear_acceleration.y = curPose.accel[1];
  msg.linear_acceleration.z = curPose.accel[2];

  calculate_covariance(msg.orientation_covariance, msg.angular_velocity_covariance, msg.linear_acceleration_covariance);
  
  imu_pub.publish(msg);

  if (publish_euler) {
    // Publish Euler message
    geometry_msgs::Point euler;
    euler.x = com->GetRoll();
    euler.y = com->GetPitch();
    euler.z = com->GetYaw();
    euler_pub.publish(euler);
  }
}
