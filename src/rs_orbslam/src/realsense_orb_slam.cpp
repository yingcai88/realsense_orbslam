#include "rs_orbslam/realsense_orb_slam.h"

Realsense_ORB_SLAM::Realsense_ORB_SLAM(const ros::NodeHandle &nh, const ros::NodeHandle& pnh)
  :nh_(nh), pnh_(pnh)
{
  imu_sub_ = nh_.subscribe("/imu0", 2000, &Realsense_ORB_SLAM::imuCallback, this);
  rgb_sub_.subscribe(nh_, "/camera/rgb/image_raw", 1);
  depth_sub_.subscribe(nh_, "/camera/depth/image_raw", 1);
  sync_rgbd_.reset(new SyncRgbd(SyncPol(10), rgb_sub_, depth_sub_));
  sync_rgbd_->registerCallback(boost::bind(&Realsense_ORB_SLAM::grabRGBD,this,_1,_2));

  std::string voc_path, xml_path;
  pnh_.param<std::string>("voc_path", voc_path, "~/ORB_SLAM2/Vocabulary/ORBvoc.txt");
  pnh_.param<std::string>("xml_path", xml_path, "~/config/realsense.yaml");
  pnh_.param<int>("image_skip", image_skip_, (image_skip_>1) ? image_skip_ : 1);

  // read parames for depth rectification
  readCameraParam(xml_path);

  slam_sys_ = new ORB_SLAM2::System(voc_path, xml_path, ORB_SLAM2::System::RGBD, true);

  std::cout << "SLAM initialization done!" << std::endl;

  frame_cout_ = 0;
}

Realsense_ORB_SLAM::~Realsense_ORB_SLAM()
{
  delete slam_sys_;
}

void Realsense_ORB_SLAM::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
{
  cout << fixed;
  cout << imu_msg->header.stamp.toSec() << endl;
}


void Realsense_ORB_SLAM::grabRGBD(const sensor_msgs::ImageConstPtr& msg_rgb,const sensor_msgs::ImageConstPtr& msg_depth)
{
  frame_cout_ ++;

  if(frame_cout_ % image_skip_)
  {
    // std::cout << "image skip" << std::endl;
    return;
  }

  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try
  {
    cv_ptrRGB = cv_bridge::toCvShare(msg_rgb);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try
  {
    cv_ptrD = cv_bridge::toCvShare(msg_depth);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Get raw image
  cv::Mat img_mono  = cv_ptrRGB->image;
  cv::Mat img_depth = cv_ptrD->image;  // realsense_dual, depth/image_raw is 16U type

  // Copy image to opencv Mat
  double timestamp = image_skip_/20.0; //cv_ptrRGB->header.stamp.toSec();
  cv::Mat img_depth_f32 = cv::Mat::zeros(img_depth.rows, img_depth.cols, CV_32F);
  img_depth.convertTo(img_depth_f32, CV_32F, 1.0/1000);

  // Rectify depth image
  cv::Mat img_depth_rect = cv::Mat::zeros(img_mono.rows, img_mono.cols, CV_32F);
  rectDepth(img_depth_f32, img_depth_rect);

  // SLAM tracking
  cv::Mat pose_out = slam_sys_->TrackRGBD(img_mono, img_depth_rect, timestamp);

  // Tracking ok
  if(!pose_out.empty())
  {
    pose_rotation_w2b_ = pose_out.rowRange(0,3).colRange(0,3);
    pose_translation_w2b_ = pose_out.rowRange(0,3).col(3);
    pose_rotation_b2w_ = pose_rotation_w2b_.t();
    pose_translation_b2w_ = -pose_rotation_b2w_ * pose_translation_w2b_;

    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2);
    std::cout << timestamp << "," << pose_translation_b2w_.at<float>(0,0)
              << "," << pose_translation_b2w_.at<float>(0,1) << "," << pose_translation_b2w_.at<float>(0,2)<< std::endl;
  }
  else
  {
    ROS_WARN("Tracking fail in current frame!");
  }
}

void Realsense_ORB_SLAM::readCameraParam(const string &xml_path)
{
  cv::FileStorage fSettings(xml_path, cv::FileStorage::READ);
  rgb_fx_ = fSettings["Camera.fx"];
  rgb_fy_ = fSettings["Camera.fy"];
  rgb_cx_ = fSettings["Camera.cx"];
  rgb_cy_ = fSettings["Camera.cy"];

  ir_fx_ = fSettings["IR.fx"];
  ir_fy_ = fSettings["IR.fy"];
  ir_cx_ = fSettings["IR.cx"];
  ir_cy_ = fSettings["IR.cy"];

  delta_rgb_ir_ = fSettings["Delta_rgb_ir"];
  min_valid_depth_ = fSettings["Min_valid_depth"];
  max_valid_depth_ = fSettings["Max_valid_depth"];
}

void Realsense_ORB_SLAM::rectDepth(const cv::Mat& img_depth_f32, cv::Mat& img_depth_rect)
{
  for (int i = 0; i < img_depth_f32.rows; ++i)
  {
    for ( int j = 0; j < img_depth_f32.cols; ++j)
    {
      float d = img_depth_f32.at<float>(i,j);

      if(d>0.5 && d<8)
      {
        float x = d * (j-ir_cx_) / ir_fx_ - delta_rgb_ir_; //change to rgb camera frame
        float y = d * (i-ir_cy_) / ir_fy_;
        float z = d; // meter

        float xc = x / z;
        float yc = y / z;
        int rect_x = rgb_fx_ * xc + rgb_cx_;
        int rect_y = rgb_fy_ * yc + rgb_cy_;

        if(rect_x>=0 && rect_y>=0 && rect_x<img_depth_rect.cols && rect_y<img_depth_rect.rows )
        {
          img_depth_rect.at<float>(rect_y, rect_x) = d;
        }
      }
    }
  }
}

