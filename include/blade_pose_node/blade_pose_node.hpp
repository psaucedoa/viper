/*
 * Copyright 2024 Construction Engineering Research Laboratory (CERL)
 * Engineer Reseach and Development Center (ERDC)
 * U.S. Army Corps of Engineers
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BLADE_POSE__BLADE_POSE_NODE_HPP_
#define BLADE_POSE__BLADE_POSE_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "ceres/ceres.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Core>
#include <mutex>
#include "ceres/ceres.h"
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>  // Include this for cv2eigen
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


// For more info on CUDA & OpenCV: https://docs.opencv.org/4.9.0/d8/d34/group__cudaarithm__elem.html

/**
 * Software Architecture
 * 
 *  Receive Depth Image & rgb left image
 *    In  sensor_msgs::msg::Image depth
 *    In  sensor_msgs::msg::Image rgb_left
 *    Out cv::Mat (gray)          depth
 *    Out cv::Mat (RGB)           rgb_left
 *      |
 *      |
 *      v
 *  Cut out background from depth image:
 *    In  cv::Mat (gray) depth
 *    Out cv::Mat (gray) depth_mask
 *    - Determine highest pixel value (furthest pixels acceptable) w blade furthest from camera
 *    - Remove pixels that are further away than that value (the background)
 *    ? Are the values stored in the depth image scaled consistently? 
 *    - Maybe that's in the /zed/calibration topic
 *      |
 *      |
 *      v
 *  Mask the rgb image
 *    In  cv::Mat (gray) depth_mask
 *    In  cv::Mat (RGB)  rgb_left
 *    Out cv::Mat (RGB)  rgb_left_masked 
 *    - Do some sort of && operation? 
 *    - If the pixel has a value (made it through the previous step)..
 *    - ...Then the corresponding pixel on the rgb image passes the filter
 *    - If the depth pixel has no value (got trimmed in last step) ...
 *    - ...Then the corresponding rgb pixel does not get passed (set to black or white or something)
 *      |
 *      |
 *      V
 *  2-View reconstruction to backout camera pose
 *    In  cv::Mat (RGB)             rgb_left_masked
 *    Out geometry_msgs::msg::Pose  blade_pose
 *    ! We should first do 2-view reconstruction using openCV's built in functions
 *    ? Do we want to use some optimizer here? Such as symforce? 
 *    - ...We would have to if we do 2-view by hand
 *    ? How long do we want our buffer to be for 2-view reconstruction?
 *      |
 *      |
 *      V
 *  Get Joint states from camera pose (do forward kinematics)
 *    In  geometry_msgs::msg::Pose      blade_pose
 *    Out sensor_msgs::msg::JointState  arm joints(s)
 *    - Not necessary for minimum deliverable for project
 *    - However, might be helpful for visualization on the model? Idk how that works
*/

/**
CLass structure
  - Image
  - Image pair? that gets image stuff done to it
  - Cloud
  - Cloud pair? that gets cloud stuff done to it

  - flag for first iteration (instantiation?)
  - 

*/

using namespace std::chrono_literals;

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

namespace blade_pose
{

class BladeImage
{
public:
  BladeImage();
  ~BladeImage();

  // camera intrinsics
  Eigen::Matrix3f K;

  sensor_msgs::msg::Image::SharedPtr depth_image_msg;
  cv::Mat depth_image_current;
  cv::Mat depth_image_previous;
  cv::Mat depth_image_mask;

  cv::Mat rgb_image_current;
  cv::Mat rgb_image_previous;
  cv::Mat rgb_image_current_masked;
  cv::Mat rgb_image_previous_masked;

  cv::Mat keypoints_image_current;
  cv::Mat keypoints_image_previous;
  cv::Mat descriptors_image_current;
  cv::Mat descriptors_image_previous;
  
  std::vector<cv::KeyPoint> keypoints_vector_current;
  std::vector<cv::KeyPoint> keypoints_vector_previous;
  std::vector<cv::DMatch> matches;
  std::vector<cv::DMatch> good_matches;
  std::vector<Eigen::Vector3i> points_pixels_current;
  std::vector<Eigen::Vector3i> points_pixels_previous;
  geometry_msgs::msg::PoseStamped blade_pose_absolute_;
  geometry_msgs::msg::PoseStamped prev_blade_pose_relative;

  // publishers
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Image>> pub_masked_depth_image_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Image>> pub_masked_rgb_image_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Image>> pub_keypoints_image_;

  void pushbackDepthImage(const sensor_msgs::msg::Image::SharedPtr depth_image);
  void pushbackRGBImage(const sensor_msgs::msg::Image::SharedPtr rgb_image);
  void createMask();

  /**
   * @brief Masks RGB image to produce color iamge with background masked, leaving blade
  */
  void maskRGBImage();

  /**
   * @brief Gets the good keypoint matches and their pixel coordinates
   * @param image_1 (input) the previouse frame (to match to)
   * @param image_2 (input) the current frame
   * @param points_pixels_1 (output) the pixel coordinates of the matched keypoints in image 1
   * @param points_pixels_2 (output) the pixel coordinates of the matched keypoints in image 2
  */
  void getKeypoints();
  void getKeypointCoords();
  void publishMatchImage();
  // void publishKeypointImage();
  void publishMaskedDepthImage();
  void publishMaskedRGBImage();

  /**
   * @brief extracts the depth at the input x and y pixel coordinates
   * @param x (input) The x coordinate in pixels (row)
   * @param y (input) The y coordinate in pixels (col)
   * @param depth (output) The depth at the input pixel coordinates
  */
  void getPixelDepth(int x, int y, double & depth_mm);

private:
};  // end class BladeImage

class BladeCloud
{
public:
  BladeCloud();
  ~BladeCloud();

  bool point_clouds_iterated = false;
  // cloud pointers
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;  //(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;  //(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;  //(new pcl::PointCloud<pcl::PointXYZ>);

  // publishers
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_current_cloud_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_previous_cloud_;
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_aligned_cloud_;
  std::shared_ptr<rlc::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> pub_blade_pose_;

  /**
   * @brief generates a point cloud of the matched keypoints using => 
   * (x, y, z, 1) = D(u,v) * inv(K) * (u, v, 1)
   * @param blade_image
  */
  void updatePointClouds(BladeImage& blade_image);
  void getPose(geometry_msgs::msg::PoseStamped& blade_pose);
  void publishClouds();

};

class BladePoseNode : public rlc::LifecycleNode
{
public:
  explicit BladePoseNode(const rclcpp::NodeOptions & OPTIONS);
  ~BladePoseNode();

  // Lifecycle functions
  LNI::CallbackReturn on_configure(const rlc::State & state) override;  // on node configuration
  LNI::CallbackReturn on_activate(const rlc::State & state) override;  // on node activation
  LNI::CallbackReturn on_deactivate(const rlc::State & state) override;  // on node deactivation
  LNI::CallbackReturn on_cleanup(const rlc::State & state) override;  // on node cleanup
  LNI::CallbackReturn on_shutdown(const rlc::State & state) override;  // on node shutdown

  // publishers
  // publish joint state?
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::JointState>> pub_joint_;

  // subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_img_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_img_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;

  sensor_msgs::msg::Image::SharedPtr depth_image_msg_;
  sensor_msgs::msg::Image::SharedPtr rgb_image_msg_;
  bool depth_image_empty_ = true;
  bool rgb_image_empty_ = true;

  // params
  bool absolute_pose_tracking_;
  std::string frame_id_;      // such as: base, etc.
  std::string sensor_name_; 
  std::string sub_topic_depth_img_;
  std::string sub_topic_left_img_;
  std::string pub_topic_blade_pose_;

  geometry_msgs::msg::PoseStamped blade_delta_pose_;
  geometry_msgs::msg::PoseStamped blade_pose_absolute_;
  geometry_msgs::msg::Pose relative_pose_prev_;
  Eigen::Quaterniond quaternion_orientation_;

  // timers
  rclcpp::TimerBase::SharedPtr relative_pose_timer_;
  rclcpp::TimerBase::SharedPtr absolute_pose_timer_;

  // data classes
  BladeImage blade_image;
  BladeCloud blade_cloud;

  // functions
  void RelativePoseCallback(); 
  void AbsolutePoseCallback(); 

  void rxDepthImage(sensor_msgs::msg::Image::SharedPtr depth_image);
  void rxRGBLeftImage(sensor_msgs::msg::Image::SharedPtr left_image);
  
  /**
   * ¿SOME QUESTIONS?
   *    ¿Should we publish blade pose and joint states within the functions that find them?
   *    ¿Or should we pass in variables to be filled, and publish them in a separate function?
  */

  // /**
  //  * @brief Generates a pose estimate of the blade. 
  //  * Also publishes the target, source, and aligned point clouds
  //  * @param target_cloud (input) The point cloud generated in the previous frame
  //  * @param source_cloud (input) The point cloud generated in the current frame
  // */
  // void GetPose(
  //   const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, 
  //   const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, 
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud, 
  //   geometry_msgs::msg::PoseStamped& blade_pose);

  // void PublishRelativePose(
  //   const Eigen::Matrix4f& transform, 
  //   const geometry_msgs::msg::PoseStamped& blade_pose_in, 
  //   geometry_msgs::msg::PoseStamped& prev_blade_pose);


  // void PublishAbsolutePose(geometry_msgs::msg::PoseStamped& blade_pose_in);

  // void OptimizePose(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& result_image,
  //                   std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, geometry_msgs::msg::PoseStamped& blade_pose_out);

  // /**
  //  * @brief Finds the blade pose
  //  * @note INPUT A VECTOR OF IMAGES, SINCE WE MIGHT WANT A BUNCH FOR POSE RECONSTRUCTION?
  //  * @note Also, this is complex. Requires tons of individual functions (if doing custom)
  // */
  // bool ConstructBladePose(
  //   const cv::Mat& rgb_masked_1, const cv::Mat& rgb_masked_2, const cv::Mat& rgb_masked_1_unmasked, const cv::Mat& rgb_masked_2_unmasked, const cv::Mat& rgb_masked_1_depth, const cv::Mat& rgb_masked_2_depth, geometry_msgs::msg::PoseStamped& blade_pose_out);
  
  // /**
  //  * @brief Finds the arm joint states using inverse kinematics 
  //  * (known end effector pose, finds joint angles)
  // */
  // void FindArmJointStates(
  //   const geometry_msgs::msg::PoseStamped& blade_pose_in, sensor_msgs::msg::JointState& arm_joint_x_out);

};  // end class BladePoseNode

}  // end namespace blade_pose

#endif  // BLADE_POSE__BLADE_POSE_NODE_HPP_