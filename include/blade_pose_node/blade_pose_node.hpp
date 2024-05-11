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

using namespace std::chrono_literals;

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

namespace blade_pose
{

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
  // publish the masked depth image
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Image>> pub_masked_depth_image_;
  // publish the masked rgb image
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Image>> pub_masked_rgb_image_;
  // publish the keypoints image
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Image>> pub_keypoints_image_;
  // publish the blade pose
  std::shared_ptr<rlc::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> pub_blade_pose_relative_;
  std::shared_ptr<rlc::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> pub_blade_pose_absolute_;
  // publish point cloud 1
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_point_cloud_1_;
  // publish point cloud 2
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_point_cloud_2_;
  // publish point cloud 3
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_point_cloud_3_;

  // publish point cloud 1
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_point_cloud_1_absolute_;
  // publish point cloud 2
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_point_cloud_2_absolute_;
  // publish point cloud 3
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pub_point_cloud_3_absolute_;

  // subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_img_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_img_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;

  // params
  bool absolute_pose_tracking_;
  std::string frame_id_;      // such as: base, etc.
  std::string sensor_name_; 
  std::string sub_topic_depth_img_;
  std::string sub_topic_left_img_;
  std::string sub_topic_imu_;
  std::string pub_topic_masked_image_;
  std::string pub_topic_masked_depth_img_;
  std::string pub_topic_masked_rgb_img_;
  std::string pub_topic_keypoints_img_;
  std::string pub_topic_blade_pose_;
  std::string pub_topic_point_cloud_1_;
  std::string pub_topic_point_cloud_2_;
  std::string pub_topic_point_cloud_3_;

  // global vars
  cv::cuda::GpuMat gpu_depth_img_;
  cv::cuda::GpuMat gpu_rgb_img_;
  cv::Mat cpu_depth_img_;
  cv::Mat cpu_depth_img_32_;
  cv::Mat cpu_rgb_img_;
  // cv::Mat rgb_masked_stored_1;
  // cv::Mat rgb_masked_stored_2;
  // cv::Mat rgb_img_1_;
  // cv::Mat rgb_img_2_;

  cv::Mat absolute_rgb_img_1_;
  cv::Mat absolute_rgb_masked_img_1_;
  // cv::Mat depth_masked_stored_1_;
  // cv::Mat depth_masked_stored_2_;
  sensor_msgs::msg::Image::SharedPtr depth_image_msg_stored_;
  sensor_msgs::msg::Image::SharedPtr depth_image_msg_;
  
  geometry_msgs::msg::PoseStamped blade_pose_absolute_;
  geometry_msgs::msg::Pose relative_pose_prev_;
  Eigen::Quaterniond quaternion_orientation_;
  int depth_upper_bound_;

  std::vector<cv::Mat> image_buffer_; 
  // timers
  rclcpp::TimerBase::SharedPtr relative_pose_timer_;
  rclcpp::TimerBase::SharedPtr absolute_pose_timer_;

  // functions
  void RelativePoseCallback(); 
  void AbsolutePoseCallback(); 

  void rxDepthImage(const sensor_msgs::msg::Image::SharedPtr depth_image);
  void rxRGBLeftImage(const sensor_msgs::msg::Image left_image);
  
  /**
   * ¿SOME QUESTIONS?
   *    ¿Should we publish blade pose and joint states within the functions that find them?
   *    ¿Or should we pass in variables to be filled, and publish them in a separate function?
  */


  /**
   * @brief Removes far away background pixels from the depth image to produce a mask
  */
  void RemoveBackground(const cv::Mat& depth_in, cv::Mat& depth_mask_out);

  /**
   * @brief Masks RGB image to produce color iamge with background masked, leaving blade
  */
  void MaskRGBImage(const cv::Mat& depth_mask_in, const cv::Mat (&rgb_in), cv::Mat (&rgb_out));

  /**
   * @brief Gets the good keypoint matches and their pixel coordinates
   * @param image_1 (input) the previouse frame (to match to)
   * @param image_2 (input) the current frame
   * @param points_pixels_1 (output) the pixel coordinates of the matched keypoints in image 1
   * @param points_pixels_2 (output) the pixel coordinates of the matched keypoints in image 2
  */
  void GetKeypoints(
    const cv::Mat& image_1, const cv::Mat& image_2, 
    std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, 
    cv::Mat& descriptors_1, cv::Mat& descriptors_2);

  void GetKeypointCoords(
    const cv::Mat& descriptors_1, const cv::Mat& descriptors_2, 
    const std::vector<cv::KeyPoint>& keypoints_1, const std::vector<cv::KeyPoint>& keypoints_2, 
    std::vector<Eigen::Vector3i>& points_pixels_1, std::vector<Eigen::Vector3i>& points_pixels_2,
    std::vector<cv::DMatch>& matches);

  void GetMatchImage(
    const cv::Mat& rgb_image_1, const cv::Mat& rgb_image_2, 
    const std::vector<cv::KeyPoint>& keypoints_1, const std::vector<cv::KeyPoint>& keypoints_2, 
    const std::vector<cv::DMatch>& matches);


  /**
   * @brief generates a point cloud of the matched keypoints using => 
   * (x, y, z, 1) = D(u,v) * inv(K) * (u, v, 1)
   * @param points_pixels_1 (input) the pixel coordinates of the matched keypoints in image 1
   * @param points_pixels_2 (input) the pixel coordinates of the matched keypoints in image 2
   * @param target_cloud (output) The point cloud generated from points_pixels_1 (the prev frame)
   * @param source_cloud (output) The point cloud generated from points_pixels_2 (the current frame)
   * 
  */
  void GetPointCloud(
    const std::vector<Eigen::Vector3i>& points_pixels_1, 
    const std::vector<Eigen::Vector3i>& points_pixels_2, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud);

  void PublishClouds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud);

  void PublishAbsoluteClouds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud);


  /**
   * @brief Generates a pose estimate of the blade. 
   * Also publishes the target, source, and aligned point clouds
   * @param target_cloud (input) The point cloud generated in the previous frame
   * @param source_cloud (input) The point cloud generated in the current frame
  */
  void GetPose(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud, 
    geometry_msgs::msg::PoseStamped& blade_pose);

  void PublishRelativePose(
    const Eigen::Matrix4f& transform, 
    const geometry_msgs::msg::PoseStamped& blade_pose_in, 
    geometry_msgs::msg::PoseStamped& prev_blade_pose);


  void PublishAbsolutePose(geometry_msgs::msg::PoseStamped& blade_pose_in);

  /**
   * @brief extracts the depth at the input x and y pixel coordinates
   * @param x (input) The x coordinate in pixels (row)
   * @param y (input) The y coordinate in pixels (col)
   * @param depth (output) The depth at the input pixel coordinates
  */
  void GetPixelDepth(int x, int y, sensor_msgs::msg::Image::SharedPtr& depth_img_msg, double & depth_mm);



  // might be cool to optimize pose
  void OptimizePose(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& result_image,
                    std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, geometry_msgs::msg::PoseStamped& blade_pose_out);

  /**
   * @brief Finds the blade pose
   * @note INPUT A VECTOR OF IMAGES, SINCE WE MIGHT WANT A BUNCH FOR POSE RECONSTRUCTION?
   * @note Also, this is complex. Requires tons of individual functions (if doing custom)
  */
  bool ConstructBladePose(
    const cv::Mat& rgb_masked_1, const cv::Mat& rgb_masked_2, const cv::Mat& rgb_masked_1_unmasked, const cv::Mat& rgb_masked_2_unmasked, const cv::Mat& rgb_masked_1_depth, const cv::Mat& rgb_masked_2_depth, geometry_msgs::msg::PoseStamped& blade_pose_out);
  
  /**
   * @brief Finds the arm joint states using inverse kinematics 
   * (known end effector pose, finds joint angles)
  */
  void FindArmJointStates(
    const geometry_msgs::msg::PoseStamped& blade_pose_in, sensor_msgs::msg::JointState& arm_joint_x_out);

  void extractAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& img1_unmasked, const cv::Mat& img2_unmasked,  const cv::Mat& img1_depth, const cv::Mat& img2_depth, cv::Mat& result_image,
                                            std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, std::vector<cv::Point3f>& points3d1, std::vector<cv::Point3f>& points3d2);


  void Convert32F(cv::Mat& mat);


};  // end class BladePoseNode

}  // end namespace blade_pose

#endif  // BLADE_POSE__BLADE_POSE_NODE_HPP_