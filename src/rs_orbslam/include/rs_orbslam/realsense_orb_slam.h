#ifndef REALSENSE_ORB_SLAM_H
#define REALSENSE_ORB_SLAM_H

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <System.h>

class Realsense_ORB_SLAM
{
public:
  Realsense_ORB_SLAM(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~Realsense_ORB_SLAM();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber imu_sub_;
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPol;
  typedef message_filters::Synchronizer<SyncPol> SyncRgbd;
  boost::shared_ptr<SyncRgbd> sync_rgbd_;

  void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
  void grabRGBD(const sensor_msgs::ImageConstPtr& msg_rgb,const sensor_msgs::ImageConstPtr& msg_depth);

  ORB_SLAM2::System* slam_sys_;
  void readCameraParam(const string &xml_path);
  void rectDepth(const cv::Mat& img_depth_f32, cv::Mat& img_depth_rect);

  int frame_cout_;
  int image_skip_;
  cv::Mat pose_rotation_w2b_;
  cv::Mat pose_translation_w2b_;
  cv::Mat pose_rotation_b2w_;
  cv::Mat pose_translation_b2w_;

  // realsense camera parameters
  float ir_fx_;
  float ir_fy_;
  float ir_cx_;
  float ir_cy_;

  float rgb_fx_;
  float rgb_fy_;
  float rgb_cx_;
  float rgb_cy_;

  float min_valid_depth_;
  float max_valid_depth_;
  float delta_rgb_ir_;

};

#endif // REALSENSE_ORB_SLAM_H
