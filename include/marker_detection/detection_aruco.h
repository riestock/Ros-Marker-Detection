#ifndef DETECTION_ARUCO_H
#define DETECTION_ARUCO_H

#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <compressed_image_transport/compressed_subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include "marker_detection/MarkerTransform.h"

namespace marker_detection
{

class DetectionAruco
{
private:
  int initialize();
  int readParameters();
  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();
  void setupTransform();
  void setupAruco();

  void subscriberCallbackImage(const sensor_msgs::ImageConstPtr &msg);
  void subscriberCallbackCamerainfo(const sensor_msgs::CameraInfoConstPtr &msg);
  void publisherMarkerTf();
  void publisherMarkerTfFromMSG(marker_detection::MarkerTransform &msg);
  bool serviceCallbackFixate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool readDetectorParameters(cv::Ptr<cv::aruco::DetectorParameters> &params);
  void printTransform(tf2:: Transform &transform);

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_image_;
  ros::Subscriber sub_camera_info_;
  ros::Publisher pub_marker_data_;
  ros::Publisher pub_rejected_marker_data_;
  ros::ServiceServer service_fixate_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster *broadcaster_ = nullptr;

  std::string topic_sub_image_;
  std::string topic_sub_camera_info_;
  std::string topic_prefix_pub_marker_data_;

  bool running_;
  bool debug_;
  bool generic_marker_name_;
  bool marker_is_child_;
  bool show_rejected_marker_;
  bool debug_window_;
  bool fixate_trigger_;
  bool publish_tranform_;
  bool init_;
  int dictionary_id_;
  int marker_id_;
  int wait_time_;
  float offset_rx_;
  float offset_ry_;
  float offset_rz_;
  float marker_size_;
  double rate_;
  double sleep_time_;


  std::string device_name_;
  std::string window_name_;
  std::string file_detector_params_;
  std::string image_encoding_;
  std::string optical_frame_;
  std::string optical_frame_suffix_;
  std::string target_frame_;
  std::string target_frame_suffix_;

  std::string marker_frame_;
  cv::Mat callback_image_;
  cv::Mat distortion_mat_;
  cv::Mat camera_mat_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  marker_detection::MarkerTransform marker_data_;
  marker_detection::MarkerTransform fixate_marker_;
  geometry_msgs::TransformStamped msg_camera_link_to_optical_frame_;

public:
  DetectionAruco(const ros::NodeHandle &nh_ = ros::NodeHandle(), const ros::NodeHandle &priv_nh_ = ros::NodeHandle("~"));
  ~DetectionAruco() = default;
  bool start();
  void stop();
};


}


#endif //DETECTION_ARUCO_H
