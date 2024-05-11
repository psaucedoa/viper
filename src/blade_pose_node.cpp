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

#include "blade_pose_node/blade_pose_node.hpp"
#include <memory>
#include <string>
#include <Eigen/Dense>

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

namespace blade_pose
{

BladePoseNode::BladePoseNode(const rclcpp::NodeOptions & OPTIONS)
: rclcpp_lifecycle::LifecycleNode("blade_pose_node", OPTIONS)
{
  // params
  absolute_pose_tracking_ = this->declare_parameter<bool>("absolute_pose_tracking", false);
  sensor_name_ = this->declare_parameter<std::string>("sensor_name", "");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "");
  sub_topic_depth_img_ = this->declare_parameter<std::string>("sub_topic_depth_img", "");
  sub_topic_left_img_ = this->declare_parameter<std::string>("sub_topic_left_img", "");
  sub_topic_imu_ = this->declare_parameter<std::string>("sub_topic_imu", "");
  pub_topic_masked_depth_img_ = this->declare_parameter<std::string>(
    "pub_topic_depth_masked_image",
    "");
  pub_topic_masked_rgb_img_ = this->declare_parameter<std::string>("pub_topic_rgb_image", "");
  pub_topic_keypoints_img_ = this->declare_parameter<std::string>("pub_topic_keypoints_image", "");
  pub_topic_blade_pose_ = this->declare_parameter<std::string>("pub_topic_blade_pose", "");
  pub_topic_point_cloud_1_ = this->declare_parameter<std::string>(
    "pub_topic_point_cloud_1", "cloud1");
  pub_topic_point_cloud_2_ = this->declare_parameter<std::string>(
    "pub_topic_point_cloud_2", "cloud2");
  pub_topic_point_cloud_3_ = this->declare_parameter<std::string>(
    "pub_topic_point_cloud_3","cloud3");

  depth_upper_bound_ = this->declare_parameter<int>("depth_upper_bound", 200);

  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "sensor_name: %s", sensor_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "sub_topic_depth_img: %s", sub_topic_depth_img_.c_str());
  RCLCPP_INFO(this->get_logger(), "sub_topic_left_img: %s", sub_topic_left_img_.c_str());
  RCLCPP_INFO(this->get_logger(), "sub_topic_imu: %s", sub_topic_imu_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "pub_topic_masked_depth_img: %s", pub_topic_masked_depth_img_.c_str());
  RCLCPP_INFO(
    this->get_logger(), "pub_topic_masked_rgb_img: %s",
    pub_topic_masked_rgb_img_.c_str());
  RCLCPP_INFO(this->get_logger(), "pub_topic_point_cloud_1: %s", pub_topic_point_cloud_1_.c_str());
  RCLCPP_INFO(this->get_logger(), "pub_topic_point_cloud_2: %s", pub_topic_point_cloud_2_.c_str());
  RCLCPP_INFO(this->get_logger(), "pub_topic_point_cloud_3: %s", pub_topic_point_cloud_3_.c_str());

}

BladePoseNode::~BladePoseNode() {}

////////////////////////////////////////////////
// END NODE
// BEGIN LIFECYCLES
////////////////////////////////////////////////

LNI::CallbackReturn BladePoseNode::on_configure(const rlc::State & state)
{
  LNI::on_configure(state);

  try {
    // subscribers
    sub_depth_img_ = this->create_subscription<sensor_msgs::msg::Image>(
      sub_topic_depth_img_, 500,
      std::bind(&BladePoseNode::rxDepthImage, this, std::placeholders::_1));

    sub_left_img_ = this->create_subscription<sensor_msgs::msg::Image>(
      sub_topic_left_img_, 500,
      std::bind(&BladePoseNode::rxRGBLeftImage, this, std::placeholders::_1));

    // publishers
    pub_masked_depth_image_ = this->create_publisher<sensor_msgs::msg::Image>(
      pub_topic_masked_depth_img_, 20);
    pub_masked_rgb_image_ = this->create_publisher<sensor_msgs::msg::Image>(
      pub_topic_masked_rgb_img_, 20);
    pub_keypoints_image_ = this->create_publisher<sensor_msgs::msg::Image>(
      pub_topic_keypoints_img_, 20);
    pub_blade_pose_relative_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/blade_pose/relative", 20);
    pub_blade_pose_absolute_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/blade_pose/absolute", 20);
    pub_point_cloud_1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/blade_pose/cloud_relative/previous", 20);
    pub_point_cloud_2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/blade_pose/cloud_relative/current", 20);
    pub_point_cloud_3_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/blade_pose/cloud_relative/aligned", 20);
    pub_point_cloud_1_absolute_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/blade_pose/cloud_absolute/previous", 20);
    pub_point_cloud_2_absolute_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/blade_pose/cloud_absolute/current", 20);
    pub_point_cloud_3_absolute_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/blade_pose/cloud_absolute/aligned", 20);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error w/ on_configure: %s", e.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Successfully Configured!");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn BladePoseNode::on_activate(const rlc::State & state)
{
  LNI::on_activate(state);
  // when driver activates, configrue the device
  pub_masked_depth_image_->on_activate();
  pub_masked_rgb_image_->on_activate();
  pub_keypoints_image_->on_activate();
  pub_blade_pose_relative_->on_activate();
  pub_blade_pose_absolute_->on_activate();
  pub_point_cloud_1_->on_activate();
  pub_point_cloud_2_->on_activate();
  pub_point_cloud_3_->on_activate();
  pub_point_cloud_1_absolute_->on_activate();
  pub_point_cloud_2_absolute_->on_activate();
  pub_point_cloud_3_absolute_->on_activate();

  // timers
  relative_pose_timer_ = this->create_wall_timer(
    100ms, std::bind(&BladePoseNode::RelativePoseCallback, this));
  
  if(absolute_pose_tracking_)
  {
    absolute_pose_timer_ = this->create_wall_timer(
    1000ms, std::bind(&BladePoseNode::AbsolutePoseCallback, this));
  }

  RCLCPP_DEBUG(this->get_logger(), "Microstrain activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn BladePoseNode::on_deactivate(const rlc::State & state)
{
  // (void)state;
  pub_masked_depth_image_->on_deactivate();
  pub_masked_rgb_image_->on_deactivate();
  pub_keypoints_image_->on_deactivate();
  pub_blade_pose_relative_->on_deactivate();
  pub_blade_pose_absolute_->on_deactivate();
  pub_point_cloud_1_->on_deactivate();
  pub_point_cloud_2_->on_deactivate();
  pub_point_cloud_3_->on_deactivate();
  pub_point_cloud_1_absolute_->on_deactivate();
  pub_point_cloud_2_absolute_->on_deactivate();
  pub_point_cloud_3_absolute_->on_deactivate();
  LNI::on_deactivate(state);

  RCLCPP_DEBUG(this->get_logger(), "BladePoseNode deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn BladePoseNode::on_cleanup(const rlc::State & state)
{
  // (void)state;

  LNI::on_cleanup(state);

  RCLCPP_DEBUG(this->get_logger(), "BladePoseNode cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn BladePoseNode::on_shutdown(const rlc::State & state)
{
  // (void)state;

  LNI::on_shutdown(state);

  RCLCPP_DEBUG(this->get_logger(), "BladePoseNode shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

////////////////////////////////////////////////
// END LIFECYCLES
// BEGIN PROGRAM FUNCTIONS
////////////////////////////////////////////////

void BladePoseNode::RelativePoseCallback()
{
  if (!cpu_rgb_img_.empty()) 
  {
    static bool initial_pose_ = true;
    static cv::Mat rgb_img_1_;
    static cv::Mat rgb_img_2_;
    static cv::Mat rgb_img_masked_1_;
    static cv::Mat rgb_img_masked_2_;
    static cv::Mat depth_img_1_;
    static cv::Mat depth_img_2_;
    static cv::Mat depth_img_masked_1_;
    static cv::Mat depth_img_masked_2_;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<Eigen::Vector3i> points_pixels_1;
    std::vector<Eigen::Vector3i> points_pixels_2;
    std::vector<cv::DMatch> good_matches;
    geometry_msgs::msg::PoseStamped blade_pose;
    static geometry_msgs::msg::PoseStamped blade_pose_absolute_;
    static geometry_msgs::msg::PoseStamped prev_blade_pose_relative;

    // local copy to prevent weird data race
    cv::Mat depth_img = cpu_depth_img_;
    cv::Mat depth_img_masked;
    // Convert32F(depth_img_32);
    RemoveBackground(depth_img, depth_img_masked);

    cv::Mat rgb_img = cpu_rgb_img_.clone();
    cv::Mat rgb_img_masked;
    MaskRGBImage(depth_img_masked, rgb_img, rgb_img_masked);

    if (initial_pose_)
    {
      rgb_img_masked_1_ = rgb_img_masked.clone();
      rgb_img_1_ = rgb_img.clone();
    }

    else
    {
      // update current frame image
      rgb_img_masked_1_ = rgb_img_masked.clone();
      rgb_img_1_ = rgb_img.clone();
      // instantiate pixel coordinate vectors
      std::vector<Eigen::Vector3i> points_pixels_1;
      std::vector<Eigen::Vector3i> points_pixels_2;
      Eigen::Matrix4f transform;
      
      // instantiate point clouds | Target - The cloud to match to | Source - The cloud being moved 
      pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      // Get the keypoints
      GetKeypoints(rgb_img_masked_1_, rgb_img_masked_2_, keypoints_1, keypoints_2, descriptors_1, descriptors_2);
      // RCLCPP_INFO(this->get_logger(), "Got relative keypoints");
      // Get their pixel coordinates
      GetKeypointCoords(descriptors_1, descriptors_2, keypoints_1, keypoints_2, points_pixels_1, points_pixels_2, good_matches);
      // RCLCPP_INFO(this->get_logger(), "Got relative keypoint coords");
      // Publish keypoint match image
      GetMatchImage(rgb_img_1_, rgb_img_2_, keypoints_1, keypoints_2, good_matches);
      // RCLCPP_INFO(this->get_logger(), "Got relative match images");
      // generate a point cloud of the matched keypoints (% accepted is in here)
      GetPointCloud(points_pixels_1, points_pixels_2, target_cloud, source_cloud);
      // RCLCPP_INFO(this->get_logger(), "got relative point clouds");
      // do icp, get pose, publish clouds and pose
      GetPose(target_cloud, source_cloud, aligned_cloud, blade_pose);
      // RCLCPP_INFO(this->get_logger(), "Got relative icp alignment");
      // publish clouds
      PublishClouds(target_cloud, source_cloud, aligned_cloud);
      // RCLCPP_INFO(this->get_logger(), "published relative clouds");
      // publish pose
      PublishRelativePose(transform, blade_pose, prev_blade_pose_relative);
      // RCLCPP_INFO(this->get_logger(), "published relative pose");

      sensor_msgs::msg::Image::SharedPtr depth_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", depth_img_masked).toImageMsg();
      sensor_msgs::msg::Image::SharedPtr rgb_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", rgb_img_masked_2_).toImageMsg();
      pub_masked_depth_image_->publish(*depth_msg);
      pub_masked_rgb_image_->publish(*rgb_msg);
    
    }
    initial_pose_ = false;
    depth_image_msg_stored_ = depth_image_msg_;

    // update previous frame image
    rgb_img_2_ = rgb_img.clone();
    rgb_img_masked_2_ = rgb_img_masked.clone();
  }
}

void BladePoseNode::AbsolutePoseCallback()
{
  if (!cpu_rgb_img_.empty()) 
  {
    static bool initial_pose_ = true;
    static cv::Mat rgb_img_1_;
    static cv::Mat rgb_img_2_;
    static cv::Mat rgb_img_masked_1_;
    static cv::Mat rgb_img_masked_2_;
    static cv::Mat depth_img_1_;
    static cv::Mat depth_img_2_;
    static cv::Mat depth_img_masked_1_;
    static cv::Mat depth_img_masked_2_;
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    std::vector<Eigen::Vector3i> points_pixels_1;
    std::vector<Eigen::Vector3i> points_pixels_2;
    std::vector<cv::DMatch> good_matches;
    geometry_msgs::msg::PoseStamped blade_pose;
    static geometry_msgs::msg::PoseStamped blade_pose_absolute_;
    static geometry_msgs::msg::PoseStamped prev_blade_pose_relative;

    // local copy to prevent weird data race
    cv::Mat depth_img = cpu_depth_img_;
    cv::Mat depth_img_masked;
    // Convert32F(depth_img_32);
    RemoveBackground(depth_img, depth_img_masked);
    sensor_msgs::msg::Image::SharedPtr depth_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", depth_img_masked).toImageMsg();

    cv::Mat rgb_img = cpu_rgb_img_.clone();
    cv::Mat rgb_img_masked;
    MaskRGBImage(depth_img_masked, rgb_img, rgb_img_masked);
    sensor_msgs::msg::Image::SharedPtr rgb_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", rgb_img_masked).toImageMsg();

    if (initial_pose_)
    {
      rgb_img_masked_1_ = rgb_img_masked.clone();
      rgb_img_1_ = rgb_img.clone();
    }

    else
    {
      // instantiate point clouds | Target - The cloud to match to | Source - The cloud being moved 
      pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      // Get the keypoints
      GetKeypoints(rgb_img_masked_1_, rgb_img_masked_2_, keypoints_1, keypoints_2, descriptors_1, descriptors_2);
      // Get their pixel coordinates
      GetKeypointCoords(descriptors_1, descriptors_2, keypoints_1, keypoints_2, points_pixels_1, points_pixels_2, good_matches);
      // Publish keypoint match image
      GetMatchImage(rgb_img_1_, rgb_img_2_, keypoints_1, keypoints_2, good_matches);
      // generate a point cloud of the matched keypoints (% accepted is in here)
      GetPointCloud(points_pixels_1, points_pixels_2, target_cloud, source_cloud);
      // do icp, get pose, publish clouds and pose
      GetPose(target_cloud, source_cloud, aligned_cloud, blade_pose);
      // publish clouds
      PublishAbsoluteClouds(target_cloud, source_cloud, aligned_cloud);
      // publish pose
      PublishAbsolutePose(blade_pose);
    }
    initial_pose_ = false;
    depth_image_msg_stored_ = depth_image_msg_;

    // update previous frame image
    rgb_img_2_ = rgb_img.clone();
    rgb_img_masked_2_ = rgb_img_masked.clone();
  }
}

void BladePoseNode::PublishAbsolutePose(geometry_msgs::msg::PoseStamped& blade_pose_in)
{
  blade_pose_in.header.stamp = now();  // Assuming you have a function to get current time
  blade_pose_in.header.frame_id = "map";  // Typically, this would be something like "world" or "map"
  pub_blade_pose_absolute_->publish(blade_pose_in);
}

void BladePoseNode::PublishRelativePose(const Eigen::Matrix4f& transform, const geometry_msgs::msg::PoseStamped& blade_pose_in, geometry_msgs::msg::PoseStamped& prev_blade_pose)
{
  geometry_msgs::msg::PoseStamped blade_pose_out;
  tf2::Transform tf_absolute_pose, tf_relative_pose;

  tf2::fromMsg(prev_blade_pose.pose, tf_absolute_pose);
  tf2::fromMsg(blade_pose_in.pose, tf_relative_pose);

  // Combine the absolute and relative poses
  tf2::Transform tf_new_pose = tf_absolute_pose * tf_relative_pose;
  // stuff the current blade pose out
  tf2::toMsg(tf_new_pose, blade_pose_out.pose);
  // save in prev_blade_pose for the next iteration
  tf2::toMsg(tf_new_pose, prev_blade_pose.pose);

  blade_pose_out.header.stamp = now();  // Assuming you have a function to get current time
  blade_pose_out.header.frame_id = "map";  // Typically, this would be something like "world" or "map"

  pub_blade_pose_relative_->publish(blade_pose_out);

  // // get the transform
  // Eigen::Affine3f affine_transform(transform);
  // // create rotation and translation matrices
  // Eigen::Matrix3f rotationMatrix = transform.block<3, 3>(0, 0);

  // RCLCPP_INFO(this->get_logger(), "Pose recovery successful");

  // Eigen::Quaternionf quaternion(rotationMatrix);

  // // Create a ROS pose
  // blade_pose_out.header.stamp = now();  // Assuming you have a function to get current time
  // blade_pose_out.header.frame_id = "map";  // Typically, this would be something like "world" or "map"

  // blade_pose_out.orientation.x = quaternion.x();
  // blade_pose_out.orientation.y = quaternion.y();
  // blade_pose_out.orientation.z = quaternion.z();
  // blade_pose_out.orientation.w = quaternion.w();
  // blade_pose_out.position.x = affine_transform.translation().x();
  // blade_pose_out.position.y = affine_transform.translation().y();
  // blade_pose_out.position.z = affine_transform.translation().z();

}

void BladePoseNode::PublishClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud)
{
  // create pcl2 ros message
  sensor_msgs::msg::PointCloud2 source_cloud_msg;
  sensor_msgs::msg::PointCloud2 target_cloud_msg;
  sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
  
  // stuff message with data
  pcl::toROSMsg(*target_cloud, target_cloud_msg);
  pcl::toROSMsg(*source_cloud, source_cloud_msg);
  pcl::toROSMsg(*aligned_cloud, aligned_cloud_msg);
  
  // give frame id
  source_cloud_msg.header.frame_id = "map";
  target_cloud_msg.header.frame_id = "map";
  aligned_cloud_msg.header.frame_id = "map";
  pub_point_cloud_1_->publish(target_cloud_msg);
  pub_point_cloud_2_->publish(source_cloud_msg);
  pub_point_cloud_3_->publish(aligned_cloud_msg);
}

void BladePoseNode::PublishAbsoluteClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud)
{
  // create pcl2 ros message
  sensor_msgs::msg::PointCloud2 source_cloud_msg;
  sensor_msgs::msg::PointCloud2 target_cloud_msg;
  sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
  
  // stuff message with data
  pcl::toROSMsg(*target_cloud, target_cloud_msg);
  pcl::toROSMsg(*source_cloud, source_cloud_msg);
  pcl::toROSMsg(*aligned_cloud, aligned_cloud_msg);
  
  // give frame id
  source_cloud_msg.header.frame_id = "map";
  target_cloud_msg.header.frame_id = "map";
  aligned_cloud_msg.header.frame_id = "map";
  pub_point_cloud_1_absolute_->publish(target_cloud_msg);
  pub_point_cloud_2_absolute_->publish(source_cloud_msg);
  pub_point_cloud_3_absolute_->publish(aligned_cloud_msg);
}

void BladePoseNode::GetPose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud, geometry_msgs::msg::PoseStamped& blade_pose)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  // RCLCPP_INFO(this->get_logger(), "GetPose Target 1: %f, %f, %f", target_cloud->points[0].x, target_cloud->points[0].y, target_cloud->points[0].z);
  // RCLCPP_INFO(this->get_logger(), "GetPose Source 1: %f, %f, %f", source_cloud->points[0].x, source_cloud->points[0].y, source_cloud->points[0].z);

  icp.setInputTarget(target_cloud);
  icp.setInputSource(source_cloud);
  // icp.setMaxCorrespondenceDistance(0.05);  // in meters
  icp.setMaximumIterations(12);
  icp.setTransformationEpsilon(1e-8);
  // icp.setEuclideanFitnessEpsilon(1);
  icp.align(*aligned_cloud);

  // get the transform
  Eigen::Matrix4f transform = icp.getFinalTransformation();
  Eigen::Affine3f affine_transform(transform);
  // create rotation and translation matrices
  Eigen::Matrix3f rotationMatrix = transform.block<3, 3>(0, 0);
  // Eigen::Vector3f translationVector = transform.block<3, 1>(0, 3);

  // get the inverse of these things
  // Eigen::Matrix3f rotation_matrix_inv = rotationMatrix;
  // Eigen::Vector3f translation_vector_inv = translationVector;
  // RCLCPP_INFO(this->get_logger(), "ICP Position translationVector: %f, %f, %f", translationVector.x(), translationVector.y(), translationVector.z());

  // Convert rotation matrix to quaternion
  Eigen::Quaternionf quaternion(rotationMatrix);
  quaternion.normalize();

  // Populate geometry_msgs::msg::PoseStamped message
  // geometry_msgs::msg::PoseStamped blade_pose;
  blade_pose.pose.orientation.x = quaternion.x();
  blade_pose.pose.orientation.y = quaternion.y();
  blade_pose.pose.orientation.z = quaternion.z();
  blade_pose.pose.orientation.w = quaternion.w();
  blade_pose.pose.position.x = affine_transform.translation().x();
  blade_pose.pose.position.y = affine_transform.translation().y();
  blade_pose.pose.position.z = affine_transform.translation().z();

  // RCLCPP_INFO(this->get_logger(), "ICP Position: %f, %f, %f", relative_pose.position.x, relative_pose.position.y, relative_pose.position.z);

  //check that relative pose is the same from the previous one
  // if (relative_pose_prev_.position.x == relative_pose.position.x ||
  //   relative_pose_prev_.position.y == relative_pose.position.y ||
  //   relative_pose_prev_.position.z == relative_pose.position.z ||
  //   (abs(relative_pose.position.x) < .04 && abs(relative_pose.position.y) < .04 &&
  //   abs(relative_pose.position.z) < .04))
  // {
  //   // do nothing!
  // }

  // else
  {

    // tf2::Transform tf_abs_pose, tf_rel_pose;
    // tf2::fromMsg(blade_pose_absolute_.pose, tf_abs_pose);

    // // Combine the absolute and relative poses
    // // tf2::Transform tf_new_pose = tf_rel_pose;
    // tf2::Transform tf_new_pose = tf_abs_pose * tf_rel_pose;
    // tf2::toMsg(tf_new_pose, blade_pose.pose);
    // tf2::toMsg(tf_new_pose, blade_pose_absolute_.pose);
    blade_pose.header.stamp = now();  // Assuming you have a function to get current time
    blade_pose.header.frame_id = "map";  // Typically, this would be something like "world" or "map"

    RCLCPP_INFO(this->get_logger(), "Pose recovery successful");
  }
}

void BladePoseNode::GetPointCloud(const std::vector<Eigen::Vector3i>& points_pixels_1, const std::vector<Eigen::Vector3i>& points_pixels_2, pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud)
{
  // insantiate the kamera intrinsic matrix (maybe make global?) 
  Eigen::Matrix3f K = (Eigen::Matrix3f() << 534.451, 0.0, 475.710, 
                                            0.0, 534.451, 263.394, 
                                            0.0, 0.0, 1.0).finished();
  // NOTE: there might be a problem here!
  // RCLCPP_INFO(this->get_logger(), "points_pixels_1 size: %d", points_pixels_1.size());


  // stuff the point cloud we will be matching to
  for (size_t i = 0; i < points_pixels_1.size(); i++)
  {
    pcl::PointXYZ pcl_point;
    Eigen::Vector3i pixel = points_pixels_1[i];
    double depth;

    int x = static_cast<int>(round(pixel[0]));
    int y = static_cast<int>(round(pixel[1]));
    GetPixelDepth( x, y, depth_image_msg_stored_, depth);
    if (depth < 0.4 || depth > 3)
    {
      // pass
    }

    else
    {
      // RCLCPP_INFO(this->get_logger(), "Pixels: (%d, %d)", pixel[0], pixel[1]);

      Eigen::Vector3f point_3d = K.inverse() * Eigen::Vector3f(pixel.cast<float>()) * static_cast<float>(depth);
      pcl_point.x = point_3d[0];
      pcl_point.y = point_3d[1];
      pcl_point.z = point_3d[2];
      target_cloud->push_back(pcl_point);
    }
  }
  // RCLCPP_INFO(this->get_logger(), "Point 1: %f, %f, %f", target_cloud->points[0].x, target_cloud->points[0].y, target_cloud->points[0].z);

  // stuff the point cloud we are trying to match
  for (size_t i = 0; i < points_pixels_2.size(); i++)
  {
    pcl::PointXYZ pcl_point;
    Eigen::Vector3i pixel = points_pixels_2[i];
    double depth;

    int x = static_cast<int>(round(pixel[0]));
    int y = static_cast<int>(round(pixel[1]));
    GetPixelDepth( x, y, depth_image_msg_, depth);

    if (depth < 0.4 || depth > 3)
    {
      // pass
    }
    
    else
    {
      Eigen::Vector3f point_3d = K.inverse() * Eigen::Vector3f(pixel.cast<float>()) * static_cast<float>(depth);
      pcl_point.x = point_3d[0];
      pcl_point.y = point_3d[1];
      pcl_point.z = point_3d[2];
      source_cloud->push_back(pcl_point);
    }
  }
  // RCLCPP_INFO(this->get_logger(), "Point 1: %f, %f, %f", source_cloud->points[0].x, source_cloud->points[0].y, source_cloud->points[0].z);

  // RCLCPP_INFO(this->get_logger(), "Target Cloud Size: %d", target_cloud->size());
  // RCLCPP_INFO(this->get_logger(), "Source Cloud Size: %d", source_cloud->size());

}

// std::vector<Eigen::Vector3i>& points_pixels_1, std::vector<Eigen::Vector3i>& points_pixels_2

void BladePoseNode::GetKeypoints(const cv::Mat& image_1, const cv::Mat& image_2, std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2, cv::Mat& descriptors_1, cv::Mat& descriptors_2)
{
  // Initialize ORB detector
  float scale_factor = 1.2f;
  int n_features = 3000;
  int n_levels = 8;
  int edge_threshold = 10;
  cv::Ptr<cv::ORB> detector = cv::ORB::create(n_features, scale_factor, n_levels, edge_threshold);

  // Detect keypoints

  detector->detectAndCompute(image_1, cv::Mat(), keypoints_1, descriptors_1);
  detector->detectAndCompute(image_2, cv::Mat(), keypoints_2, descriptors_2);
}

void BladePoseNode::GetKeypointCoords(const cv::Mat& descriptors_1, const cv::Mat& descriptors_2, const std::vector<cv::KeyPoint>& keypoints_1, const std::vector<cv::KeyPoint>& keypoints_2, std::vector<Eigen::Vector3i>& points_pixels_1, std::vector<Eigen::Vector3i>& points_pixels_2, std::vector<cv::DMatch>& matches)
{
  // Match features between the two images
  cv::BFMatcher matcher(cv::NORM_HAMMING, true);
  matcher.match(descriptors_1, descriptors_2, matches);

  // Filter matches using the Lowe's ratio test
  std::vector<cv::DMatch> good_matches;

  //sort matches by distance
  std::sort(matches.begin(), matches.end());

  // Get the top 10% of matches
  int num_good_matches = matches.size() * 0.4;
  for (int i = 0; i < num_good_matches; i++)
  {
    good_matches.push_back(matches[i]);
  }

  if (static_cast<int>(good_matches.size()) < 8)
  {
    RCLCPP_WARN(this->get_logger(), "Not enough matches found");
    // return;
  }

  for (auto & match : good_matches)
  {
    
    int x_1 = static_cast<int>(round(keypoints_1[match.queryIdx].pt.x));
    int y_1 = static_cast<int>(round(keypoints_1[match.queryIdx].pt.y));
    int x_2 = static_cast<int>(round(keypoints_1[match.queryIdx].pt.x));
    int y_2 = static_cast<int>(round(keypoints_1[match.queryIdx].pt.y));
    Eigen::Vector3i points_1(x_1, y_1, 1);
    Eigen::Vector3i points_2(x_2, y_2, 1);

    points_pixels_1.push_back(points_1);
    points_pixels_2.push_back(points_2);
  }
}

void BladePoseNode::GetMatchImage(const cv::Mat& rgb_image_1, const cv::Mat& rgb_image_2, const std::vector<cv::KeyPoint>& keypoints_1, const std::vector<cv::KeyPoint>& keypoints_2, const std::vector<cv::DMatch>& matches)
{    // draw matches
  cv::Mat keypoints_image;
  cv::drawMatches(rgb_image_1, keypoints_1, rgb_image_2, keypoints_2, matches, keypoints_image);
  pub_keypoints_image_->publish(
    *cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", keypoints_image).toImageMsg());

}

void BladePoseNode::rxDepthImage(const sensor_msgs::msg::Image::SharedPtr depth_image)
{

  cv_bridge::CvImagePtr cv_ptr_depth_image;
  depth_image_msg_ = depth_image;

  try {
    // copy ros image to cv image
    cv_ptr_depth_image = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
    //
    cv::Mat mono8_img = cv::Mat(cv_ptr_depth_image->image.size(), CV_8UC1);
    cv::Mat mono32_img = cv::Mat(cv_ptr_depth_image->image.size(), CV_32F);
    cv::convertScaleAbs(cv_ptr_depth_image->image, mono8_img, 100, 0.0);

    // store globally
    cpu_depth_img_ = mono8_img.clone();
    cpu_depth_img_32_ = mono32_img.clone();
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in Depth Image Callback: %s", e.what());
  }
}

void BladePoseNode::rxRGBLeftImage(const sensor_msgs::msg::Image left_image)
{

  cv_bridge::CvImagePtr cv_ptr_rgb_image;

  try {
    // copy ros image to cv image
    cv_ptr_rgb_image = cv_bridge::toCvCopy(left_image);
    cv::Mat rgb_img = cv_ptr_rgb_image->image;
    // store globally
    cpu_rgb_img_ = rgb_img.clone();
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception in RGB Image Callback: %s", e.what());
  }
}

void BladePoseNode::RemoveBackground(const cv::Mat & depth_in, cv::Mat & depth_mask_out)
{
  // upper and lower bound
  const cv::Scalar depth_pixel_upper_bound(depth_upper_bound_);  // some fairly high value
  const cv::Scalar depth_pixel_lower_bound(1);  // some fairly low

  // filters pixels, creating mask of pixels in range = 255, out of range = 0
  cv::inRange(depth_in, depth_pixel_lower_bound, depth_pixel_upper_bound, depth_mask_out);
}

void BladePoseNode::MaskRGBImage(
  const cv::Mat & depth_mask_in, const cv::Mat(&rgb_in), cv::Mat(&rgb_out))
{
  // turn the grayscale image into a three-channel image to match RGB image dimensions
  cv::Mat rgb_out_local;

  // Perform bitwise AND to mask the rgb image (since 'background' will be = 0)
  rgb_in.copyTo(rgb_out_local, depth_mask_in);
  cv::cvtColor(rgb_out_local, rgb_out_local, cv::COLOR_BGRA2BGR);
  // cv::imshow("rgb_out_local", rgb_out_local);

  // filter gray color
  cv::Scalar lower_colors_gray(50, 30, 33);
  cv::Scalar upper_colors_gray(170, 120, 100);
  cv::Mat gray_mask;
  cv::inRange(rgb_out_local, lower_colors_gray, upper_colors_gray, gray_mask);

  cv::Scalar lower_colors_metal(170, 120, 100);
  cv::Scalar upper_colors_metal(255, 255, 255);
  cv::Mat metal_mask;
  cv::inRange(rgb_out_local, lower_colors_metal, upper_colors_metal, metal_mask);

  // cv::imshow("mask", mask);
  cv::Mat gray_filtered;
  cv::Mat inverted_gray_mask;
  cv::bitwise_not(gray_mask, inverted_gray_mask);
  cv::bitwise_and(rgb_out_local, rgb_out_local, gray_filtered, inverted_gray_mask);

  cv::Mat metal_filtered;
  cv::Mat inverted_metal_mask;
  cv::bitwise_not(metal_mask, inverted_metal_mask);
  cv::bitwise_and(gray_filtered, gray_filtered, metal_filtered, inverted_metal_mask);

  // cv::imshow("rgb_filtered", rgb_filtered);
  // cv::waitKey(0);

  cv::cvtColor(metal_filtered, metal_filtered, cv::COLOR_BGR2BGRA);
  rgb_out = metal_filtered.clone();
  // cv::bitwise_and(depth_mask_in, rgb_in, rgb_out);
}

void BladePoseNode::extractAndMatchFeatures(
  const cv::Mat & img1, const cv::Mat & img2, const cv::Mat & img1_unmasked,
  const cv::Mat & img2_unmasked, const cv::Mat & img1_depth, const cv::Mat & img2_depth,
  cv::Mat & result_image,
  std::vector<cv::Point2f> & points1, std::vector<cv::Point2f> & points2,
  std::vector<cv::Point3f> & points3d1, std::vector<cv::Point3f> & points3d2)
{

  // Initialize ORB detector
  float scale_factor = 1.2f;
  int n_features = 3000;
  int n_levels = 8;
  int edge_threshold = 62;
  cv::Ptr<cv::ORB> detector = cv::ORB::create(n_features, scale_factor, n_levels, edge_threshold);

  // Detect keypoints
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  cv::Mat descriptors1, descriptors2;
  detector->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);
  detector->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);

  // Match features between the two images
  cv::BFMatcher matcher(cv::NORM_HAMMING, true);
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);
  // Filter matches using the Lowe's ratio test
  std::vector<cv::DMatch> good_matches;
  std::vector<cv::DMatch> great_matches;
  //sort matches by distance

  std::sort(matches.begin(), matches.end());

  // Get the top 10% of matches
  int num_good_matches = matches.size() * 0.4;
  for (int i = 0; i < num_good_matches; i++) {
    good_matches.push_back(matches[i]);
  }

  if (static_cast<int>(good_matches.size()) < 8) {
    RCLCPP_WARN(this->get_logger(), "Not enough matches found");
    // return;
  }


  // Extract the matched points
  points1.clear();
  points2.clear();
  for (auto & match : good_matches) {
    points1.push_back(keypoints1[match.queryIdx].pt);
    points2.push_back(keypoints2[match.trainIdx].pt);
  }

  for (auto & match : good_matches) {
    const cv::KeyPoint & kp1 = keypoints1[match.queryIdx];
    const cv::KeyPoint & kp2 = keypoints2[match.trainIdx];

    double d1 =
      static_cast<double>(img1_depth.at<float>(
        static_cast<int>(kp1.pt.x),
        static_cast<int>(kp1.pt.y)));
    double d2 =
      static_cast<double>(img2_depth.at<float>(
        static_cast<int>(kp2.pt.x),
        static_cast<int>(kp2.pt.y)));

    // RCLCPP_INFO(this->get_logger(), "Depth D1, D2: %fm %f", d1, d2);

    // RCLCPP_WARN(this->get_logger(), "Depth in image1 %f inliers", d1);
    // RCLCPP_WARN(this->get_logger(), "Depth in image2 %f inliers", d2);
    if (d1 > 0.3 && d2 > 0.3 && d1 < 3.0 && d2 < 3.0) {      // Check for valid depth
      // Convert 2D keypoints to 3D points based on the depth
      // static_cast<float>
      // RCLCPP_WARN(this->get_logger(), "Depth in image1 %f inliers", d1);
      // RCLCPP_WARN(this->get_logger(), "Depth in image2 %f inliers", d2);
      great_matches.push_back(match);
      points3d1.push_back(
        cv::Point3f(
          static_cast<float>(kp1.pt.x * d1),
          static_cast<float>(kp1.pt.y * d1), static_cast<float>(d1)));
      points3d2.push_back(
        cv::Point3f(
          static_cast<float>(kp2.pt.x * d2),
          static_cast<float>(kp2.pt.y * d2), static_cast<float>(d2)));
      RCLCPP_INFO(this->get_logger(), "points 3d 2 pushback: %f, %f, %f", static_cast<float>(kp2.pt.x), static_cast<float>(kp2.pt.y), static_cast<float>(d2));
    }
  }


  // Draw matches
  cv::drawMatches(
    img1_unmasked, keypoints1, img2_unmasked, keypoints2, great_matches,
    result_image);

  // Simple scale computation example (assuming isotropic scaling)
  // Q
  double scaleSum = 0;
  int count = 0;
  for (size_t i = 0; i < points3d1.size(); i++) {
    double norm1 = cv::norm(points3d1[i]);
    double norm2 = cv::norm(points3d2[i]);
    if (norm1 > 0 && norm2 > 0) {
      scaleSum += norm2 / norm1;
      count++;
    }
  }

  // NOTE: SO the problem is that the x and y coordinates are in pixels??

  double scale = scaleSum / count;
  RCLCPP_INFO(this->get_logger(), "Point Scale: %f", scale);  // is this supposed ot just be equal to unit?
  // Scale one of the point sets
  for (auto & point : points3d1) {
    point.x *= scale;
    point.y *= scale;
    point.z *= scale;
  }
  points1.clear();
  points2.clear();
  //add the x ,y of points3d1 to points1 and points3d2 to points2
  for (auto & point : points3d1) {
    points1.push_back(cv::Point2f(point.x, point.y));
  }
  for (auto & point : points3d2) {
    points2.push_back(cv::Point2f(point.x, point.y));
  }

}


void BladePoseNode::GetPixelDepth(int x, int y, sensor_msgs::msg::Image::SharedPtr& depth_img_msg, double & depth_mm)
{
  // Get a pointer to the depth values casting the data
  // pointer to floating point
  float * depths = reinterpret_cast<float *>(&depth_img_msg->data[0]);

  // Image coordinates of the center pixel

  // Linear index of the center pixel
  int centerIdx = x + depth_img_msg->width * y;

  // Output the measure
  depth_mm = depths[centerIdx];

  if (depth_mm < 0 || std::isnan(depth_mm) || depth_mm != depth_mm) {
    depth_mm = 0.0;
  }
  // RCLCPP_INFO(get_logger(), "Center distance : %g m", depth_mm);
}

}  // end namespace blade_pose

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blade_pose::BladePoseNode)
