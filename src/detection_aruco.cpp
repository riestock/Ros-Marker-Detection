#include <marker_detection/detection_aruco.h>
#include <nodelet/nodelet.h>

namespace marker_detection
{

  /// detects aruco marker in images and publishes the resulting transformation as
  /// marker_detection::MarkerTransform or as geometry_msgs::TransformStamped into the tf2 subsystem.
  /// \param nh_ Nodehandle used for publisher and subscriber
  /// \param priv_nh_ Nodehandle used for parameter handling from roslaunch
  DetectionAruco::DetectionAruco(const ros::NodeHandle &nh_, const ros::NodeHandle &priv_nh_) : nh_(nh_),
                                                                                                priv_nh_(priv_nh_),
                                                                                                it_(nh_),
                                                                                                topic_sub_image_(""),
                                                                                                topic_sub_camera_info_(""),
                                                                                                topic_prefix_pub_marker_data_(""),
                                                                                                running_(false),
                                                                                                debug_(false),
                                                                                                generic_marker_name_(false),
                                                                                                marker_is_child_(false),
                                                                                                show_rejected_marker_(false),
                                                                                                debug_window_(false),
                                                                                                fixate_trigger_(false),
                                                                                                publish_tranform_(true),
                                                                                                init_(false),
                                                                                                dictionary_id_(0),
                                                                                                marker_id_(3),
                                                                                                wait_time_(10),
                                                                                                offset_rx_(0.0),
                                                                                                offset_ry_(0.0),
                                                                                                offset_rz_(0.0),
                                                                                                marker_size_(0.2),
                                                                                                rate_(1),
                                                                                                sleep_time_(2.0)
  {}

bool DetectionAruco::start()
{
  if(running_)
  {
    ROS_ERROR("[DetectionAruco] is already running!");
    return false;
  }
  if(!initialize())
  {
    ROS_ERROR("[DetectionAruco] Initialization failed!");
    return false;
  }
  running_ = true;

  return true;
}

void DetectionAruco::stop()
{
  if(!running_)
  {
    ROS_ERROR("[DetectionAruco] is not running!");
    return;
  }
  running_ = false;

  if(debug_window_)
  {
    cv::destroyWindow(window_name_);
  }

  sub_image_.shutdown();
  pub_marker_data_.shutdown();
  pub_rejected_marker_data_.shutdown();
  nh_.shutdown();
  priv_nh_.shutdown();
}

void DetectionAruco::setupTransform()
{
  static tf2_ros::TransformBroadcaster broadcaster;
  broadcaster_ = &broadcaster;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener listener(tfBuffer);

  target_frame_ = device_name_ + target_frame_suffix_;
  optical_frame_ = device_name_ + optical_frame_suffix_;

  int i = 10;
  while(i--)
  {
    try
    {
      msg_camera_link_to_optical_frame_ = tfBuffer.lookupTransform(target_frame_, optical_frame_, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("[DetectionAruco] %s", ex.what());
      ROS_WARN("[DetectionAruco] wait %f", sleep_time_);
      ros::Duration(sleep_time_).sleep();
    }
//    ROS_INFO("%f, %f",msg_camera_link_to_optical_frame_.transform.translation.y,msg_camera_link_to_optical_frame_.transform.translation.x);
  }
}

/// calls all sub-initalize functions
/// \return true if all sub-initalize functions were successful
int DetectionAruco::initialize()
{
  if(!readParameters())
  {
    ROS_ERROR("[DetectionAruco] Could not read parameters!");
  }

  window_name_ = "detection_aruco_" + device_name_;

  if (debug_window_)
  {
    cv::namedWindow(window_name_);
    cv::startWindowThread();
  }

  // init distortion and camera matrix
  distortion_mat_.create(1, 4, CV_32F);
  camera_mat_.create(3, 3, CV_32F);

  setupAruco();

  initializeSubscribers();
  initializePublishers();
  initializeServices();

  return true;
}

/// reads the parameter from roslaunch
/// \return true if all parameters were successfully received
int DetectionAruco::readParameters()
{
  return
      priv_nh_.getParam("debug", debug_) &&
      priv_nh_.getParam("rate", rate_) &&
      priv_nh_.getParam("debug_window", debug_window_) &&
      priv_nh_.getParam("topic_sub_image", topic_sub_image_) &&
      priv_nh_.getParam("topic_sub_camera_info", topic_sub_camera_info_) &&
      priv_nh_.getParam("generic_marker_name", generic_marker_name_) &&
      priv_nh_.getParam("marker_is_child", marker_is_child_) &&
      priv_nh_.getParam("show_rejected_marker", show_rejected_marker_) &&
      priv_nh_.getParam("publish_tranform", publish_tranform_) &&
      priv_nh_.getParam("dictionary_id", dictionary_id_) &&
      priv_nh_.getParam("marker_id", marker_id_) &&
      priv_nh_.getParam("offset_rx", offset_rx_) &&
      priv_nh_.getParam("offset_ry", offset_ry_) &&
      priv_nh_.getParam("offset_rz", offset_rz_) &&
      priv_nh_.getParam("marker_size", marker_size_) &&
      priv_nh_.getParam("device_name", device_name_) &&
      priv_nh_.getParam("file_detector_params", file_detector_params_) &&
      priv_nh_.getParam("image_encoding", image_encoding_) &&
      priv_nh_.getParam("optical_frame_suffix", optical_frame_suffix_) &&
      priv_nh_.getParam("target_frame_suffix", target_frame_suffix_);
}

/// calls readDetectorParameters and initlaize the aruco dictionary
void DetectionAruco::setupAruco()
{
  // initilize aruco DetectorParameters
  ROS_INFO("[DetectionAruco] detector parameter file: %s", file_detector_params_.c_str());
  detector_params_ = cv::aruco::DetectorParameters::create();
  bool readOk = readDetectorParameters(detector_params_);
  if (!readOk)
  {
    ROS_ERROR("[DetectionAruco] Invalid detector parameters file");
    ros::shutdown();
  }

  // initilize aruco g_dictionary
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id_));
}

void DetectionAruco::initializeSubscribers()
{
  ROS_INFO("[DetectionAruco] Initializing Subscribers");
  image_transport::TransportHints hints("compressed");
  sub_image_ = it_.subscribe(topic_sub_image_, 1, &marker_detection::DetectionAruco::subscriberCallbackImage, this);
  sub_camera_info_ = nh_.subscribe(topic_sub_camera_info_, 1, &marker_detection::DetectionAruco::subscriberCallbackCamerainfo, this);
}

void DetectionAruco::initializePublishers()
{
  ROS_INFO("[DetectionAruco] Initializing Publishers");
  pub_marker_data_ = nh_.advertise<marker_detection::MarkerTransform>(std::string(topic_sub_image_ + "/marker_data"), 3);
  pub_rejected_marker_data_ = nh_.advertise<marker_detection::MarkerTransform>(std::string(topic_sub_image_ + "/marker_data_rejected"), 3);
}

void DetectionAruco::initializeServices()
{
  ROS_INFO("[DetectionAruco] Initializing Services");
  service_fixate_ = nh_.advertiseService("detection_aruco_fixate", &marker_detection::DetectionAruco::serviceCallbackFixate, this);
}

bool DetectionAruco::readDetectorParameters(cv::Ptr<cv::aruco::DetectorParameters> &params) {
  cv::FileStorage fs(file_detector_params_, cv::FileStorage::READ);
  if(!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  return true;
}

void DetectionAruco::printTransform(tf2:: Transform &transform)
{
  std::cout <<  transform.getRotation().getX() << " " <<   transform.getRotation().getY() << " "  <<  transform.getRotation().getZ() << " " <<   transform.getRotation().getW() << std::endl;
  std::cout <<  transform.getOrigin().getX()<< " " <<   transform.getOrigin().getY() << " "  <<  transform.getOrigin().getZ() << std::endl;
}

// retrieve the distortion and camera matrix from camera_info topic for later marker pose estimation

/// receives the camera and distortion parameter from the camera info message
/// \param msg incomming camera info message
void DetectionAruco::subscriberCallbackCamerainfo(const sensor_msgs::CameraInfoConstPtr &msg)
{
  try
  {
    // camera matrix containing intrisic camera parameter
    camera_mat_.row(0).col(0) = msg->K[0];
    camera_mat_.row(0).col(1) = msg->K[1];
    camera_mat_.row(0).col(2) = msg->K[2];
    camera_mat_.row(1).col(0) = msg->K[3];
    camera_mat_.row(1).col(1) = msg->K[4];
    camera_mat_.row(1).col(2) = msg->K[5];
    camera_mat_.row(2).col(0) = msg->K[6];
    camera_mat_.row(2).col(1) = msg->K[7];
    camera_mat_.row(2).col(2) = msg->K[8];

    // distortion matrix
    distortion_mat_.row(0).col(0) = msg->D[0];
    distortion_mat_.row(0).col(1) = msg->D[1];
    distortion_mat_.row(0).col(2) = msg->D[2];
    distortion_mat_.row(0).col(3) = msg->D[3];
    distortion_mat_.row(0).col(3) = msg->D[4];

    optical_frame_ = msg->header.frame_id;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("[DetectionAruco] %s", e.what());
  }
}

/// detection of the aruco marker in the given image
/// \param msg incomming image
void DetectionAruco::subscriberCallbackImage(const sensor_msgs::ImageConstPtr &msg)
{
  /// on first call init the transformation listener of tf2
  if(!(init_))
  {
    setupTransform();
    init_ = true;
  }

  auto start = std::chrono::high_resolution_clock::now();

  if(image_encoding_ == "mono")
  {
    callback_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
  }
  else
  {
    callback_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  }

  try
  {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<double> reprojectionError;

    /// detects markers and estimate location of corners in the image
    cv::aruco::detectMarkers(callback_image_, dictionary_, corners, ids, detector_params_, rejected);
    if (ids.size() > 0)
    {
      /// estimate pose based on camera matrices and corners
      cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_mat_, distortion_mat_, rvecs, tvecs);
      for (size_t i = 0; i < ids.size(); i++)
      {
        if (ids[i] == marker_id_)
        {
          marker_data_.header.stamp = msg->header.stamp;
          marker_data_.header.frame_id = optical_frame_;
          marker_data_.m_id = ids[i];
          marker_data_.marker_size = marker_size_;
          marker_data_.Tx = tvecs.at(i)[0];
          marker_data_.Ty = tvecs.at(i)[1];
          marker_data_.Tz = tvecs.at(i)[2];
          marker_data_.Rx = rvecs.at(i)[0];
          marker_data_.Ry = rvecs.at(i)[1];
          marker_data_.Rz = rvecs.at(i)[2];

          marker_data_.Rx += offset_rx_;
          marker_data_.Rx += offset_ry_;
          marker_data_.Rx += offset_rz_;

          if (generic_marker_name_)
          {
            marker_frame_ = "marker_" + std::to_string(ids[i]);
          }
          else
          {
            marker_frame_ = optical_frame_ + "_marker" + "_" + std::to_string(ids[i]);
          }

          /// publishes marker transformations as custom ros message
          pub_marker_data_.publish(marker_data_);

          /// calls for publishing of marker to the tf2 subsystem
          if(publish_tranform_)
          {
            publisherMarkerTf();
          }
        }
        else
        {
          pub_rejected_marker_data_.publish(marker_data_);
        }
      }
    }

    if (debug_window_)
    {
      cv::Mat g_cb_image_debug;
      callback_image_.copyTo(g_cb_image_debug);

      if (ids.size() > 0)
      {
        cv::aruco::drawDetectedMarkers(g_cb_image_debug, corners, ids);
        for (unsigned int i = 0; i < ids.size(); i++)
          cv::aruco::drawAxis(g_cb_image_debug, camera_mat_, distortion_mat_, rvecs[i], tvecs[i], marker_size_*0.5f);
      }

      if (show_rejected_marker_ && rejected.empty())
        cv::aruco::drawDetectedMarkers(g_cb_image_debug, rejected, cv::noArray(), cv::Scalar(100, 0, 255));

      cv::imshow(window_name_, g_cb_image_debug);
      char key = (char)cv::waitKey(wait_time_);
      if (key == 27)
        return;
    }
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("[DetectionAruco] %s", e.what());
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  if (debug_)
  {
    ROS_INFO("[DetectionAruco] Time taken by function: %li microseconds", duration.count());
  }
}

/// decides which marker will be published
void DetectionAruco::publisherMarkerTf()
{
  if(fixate_trigger_)
  {
    if(fixate_marker_.Rx == 0.0)
    {
      ROS_INFO_ONCE("[DetectionAruco] Waiting for marker to be fixated!");
    }else
    {
      ROS_INFO_ONCE("[DetectionAruco] Start publishing fixate marker!");
      publisherMarkerTfFromMSG(fixate_marker_);
    }
  }else
  {
    if(marker_data_.Rx == 0.0)
    {
      ROS_INFO("[DetectionAruco] Wait for marker to be detected!");
    }else
    {
      ROS_INFO_ONCE("[DetectionAruco] Start publishing marker!");
      publisherMarkerTfFromMSG(marker_data_);
    }
  }
}

/// calculates the tranformation between camera_link and marker
/// \param msg transformation of the marker in the image
void DetectionAruco::publisherMarkerTfFromMSG(marker_detection::MarkerTransform &msg)
{

  tf2::Transform marker_to_optical_frame;
  tf2::Transform optical_frame_to_marker;
  tf2::Transform marker_to_optical_frame_;
  tf2::Transform marker_to_camera_link;
  tf2::Transform camera_link_to_optical_frame;
  tf2::Transform camera_link_to_marker;

  //  get transformation from MarkerTransform Message into tf2::Transform
  tf2::Quaternion qq;
  tf2::Vector3 axis(msg.Rx, msg.Ry, msg.Rz);
  double angle = axis.length();
  axis = axis.normalize();
  qq.setRotation(axis, angle);
  optical_frame_to_marker.setRotation(qq);
  optical_frame_to_marker.setOrigin(tf2::Vector3(msg.Tx, msg.Ty, msg.Tz));

  marker_to_optical_frame = optical_frame_to_marker.inverse();

  tf2::convert(msg_camera_link_to_optical_frame_.transform, camera_link_to_optical_frame);
  marker_to_camera_link = marker_to_optical_frame * camera_link_to_optical_frame.inverse();

  camera_link_to_marker = marker_to_camera_link.inverse();

  // tf2::Transform -> geometry_msgs::Transform -> geometry_msgs::TransformStamped
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();


//  printTransform(camera_link_to_optical_frame);
//  printTransform(optical_frame_to_marker);
//  printTransform(marker_to_optical_frame);
//  printTransform(marker_to_camera_link);
//  std::cout << transformStamped.transform.rotation.x << " " <<   transformStamped.transform.rotation.y << " "  <<  transformStamped.transform.rotation.z << " " <<   transformStamped.transform.rotation.w << std::endl;

  if (marker_is_child_)
  {
    tf2::convert(camera_link_to_marker, transformStamped.transform);
    transformStamped.header.frame_id = target_frame_;
    transformStamped.child_frame_id = marker_frame_;
  }
  else
  {
    tf2::convert(camera_link_to_marker.inverse(), transformStamped.transform);
    transformStamped.header.frame_id = marker_frame_;
    transformStamped.child_frame_id = target_frame_;
  }
  broadcaster_->sendTransform(transformStamped);
}

/// allows to fixate the marker. The last detected marker will then be published.
/// \param req unimportant
/// \param res unimportant
/// \return true if the called function is successfully
bool DetectionAruco::serviceCallbackFixate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{

  if (fixate_trigger_)
  {
    fixate_trigger_ = false;
    ROS_INFO("[DetectionAruco] Turn fixate marker off! Current marker will be published");
  }
  else
  {
    fixate_marker_ = marker_data_;
    fixate_trigger_ = true;
    ROS_INFO("[DetectionAruco] Turn fixate marker on! The now saved marker will be published");
  }

  res.success = true;
  res.message = ros::NodeHandle().getNamespace();
  return true;
}

/// nodelet class which inizalises the base class. Is necessary for intraprocess communication.
class DetectionArucoNodelet : public nodelet::Nodelet
{
private:
  DetectionAruco *pDetectionAruco;

public:
  DetectionArucoNodelet() : Nodelet(), pDetectionAruco(nullptr)
  {
    NODELET_INFO("[DetectionArucoNodelet] Constructor call");
  }

  ~DetectionArucoNodelet() override
  {
    NODELET_INFO("[DetectionArucoNodelet] Destructor call");
    if(pDetectionAruco)
    {
      NODELET_INFO("[DetectionArucoNodelet] Have something to destruct");
      pDetectionAruco->stop();
      delete pDetectionAruco;
    }
  }

  void onInit() override
  {
    NODELET_INFO("[DetectionArucoNodelet] onInit");
    pDetectionAruco = new DetectionAruco(getNodeHandle(), getPrivateNodeHandle());
    if(!pDetectionAruco->start())
    {
      delete pDetectionAruco;
      pDetectionAruco = nullptr;
      throw nodelet::Exception("[DetectionArucoNodelet] Could not start nodelet");
    }
  }
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(marker_detection::DetectionArucoNodelet, nodelet::Nodelet)


/// allows to include this file with main function into a library. hack, can breake.
#ifndef marker_detection_EXPORTS

/// init the base class if not used as nodelet
int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_marker_aruco");

  if(!ros::ok())
  {
    ROS_ERROR("ros::ok failed!");
    return -1;
  }

  marker_detection::DetectionAruco arucoDetector;
  if(arucoDetector.start())
  {
    ros::spin();

    arucoDetector.stop();
  }
  return 1;
}

#endif // marker_detection_EXPORTS