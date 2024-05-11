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

BladeImage::BladeImage()
{
 this->K = (Eigen::Matrix3f() << 534.451, 0.0, 475.710, 
                                 0.0, 534.451, 263.394, 
                                 0.0, 0.0, 1.0).finished();
}

BladeImage::~BladeImage()
{}

BladeCloud::BladeCloud()
{
  // // instantiate clouds
  // this->previous_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
  // this->current_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
  // this->aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
}

BladeCloud::~BladeCloud()
{}

BladePoseNode::BladePoseNode(const rclcpp::NodeOptions & OPTIONS)
: rclcpp_lifecycle::LifecycleNode("blade_pose_node", OPTIONS)
{
  // params
  absolute_pose_tracking_ = this->declare_parameter<bool>("absolute_pose_tracking", false);
  sensor_name_ = this->declare_parameter<std::string>("sensor_name", "");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "");
  sub_topic_depth_img_ = this->declare_parameter<std::string>("sub_topic_depth_img", "");
  sub_topic_left_img_ = this->declare_parameter<std::string>("sub_topic_left_img", "");
  pub_topic_blade_pose_ = this->declare_parameter<std::string>("pub_topic_blade_pose", "");

  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "sensor_name: %s", sensor_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "sub_topic_depth_img: %s", sub_topic_depth_img_.c_str());
  RCLCPP_INFO(this->get_logger(), "sub_topic_left_img: %s", sub_topic_left_img_.c_str());


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

    // configure publishers
    this->blade_cloud.pub_current_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/viper/clouds/current_cloud", 20);
    this->blade_cloud.pub_previous_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/viper/clouds/previous_cloud", 20);
    this->blade_cloud.pub_aligned_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/viper/clouds/aligned_cloud", 20);
    this->blade_cloud.pub_blade_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/viper/pose/blade", 20);
    this->blade_image.pub_masked_depth_image_ = this->create_publisher<sensor_msgs::msg::Image>("/viper/images/masked_depth_image", 20);
    this->blade_image.pub_masked_rgb_image_ = this->create_publisher<sensor_msgs::msg::Image>("/viper/images/masked_rgb_image", 20);
    this->blade_image.pub_keypoints_image_ = this->create_publisher<sensor_msgs::msg::Image>("/viper/images/keypoints_image", 20); 

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

  blade_cloud.pub_current_cloud_->on_activate();
  blade_cloud.pub_previous_cloud_->on_activate();
  blade_cloud.pub_aligned_cloud_->on_activate();
  blade_cloud.pub_blade_pose_->on_activate();
  blade_image.pub_masked_depth_image_->on_activate();
  blade_image.pub_masked_rgb_image_->on_activate();
  blade_image.pub_keypoints_image_->on_activate();  
  
  // timers
  relative_pose_timer_ = this->create_wall_timer(
    100ms, std::bind(&BladePoseNode::RelativePoseCallback, this));
  
  // if(absolute_pose_tracking_)
  // {
  //   absolute_pose_timer_ = this->create_wall_timer(
  //   1000ms, std::bind(&BladePoseNode::AbsolutePoseCallback, this));
  // }

  RCLCPP_DEBUG(this->get_logger(), "Microstrain activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn BladePoseNode::on_deactivate(const rlc::State & state)
{
  // (void)state;
  LNI::on_deactivate(state);

  blade_cloud.pub_current_cloud_->on_deactivate();
  blade_cloud.pub_previous_cloud_->on_deactivate();
  blade_cloud.pub_aligned_cloud_->on_deactivate();
  blade_cloud.pub_blade_pose_->on_deactivate();
  blade_image.pub_masked_depth_image_->on_deactivate();
  blade_image.pub_masked_rgb_image_->on_deactivate();
  blade_image.pub_keypoints_image_->on_deactivate();

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
  if (!this->depth_image_empty_ && !this->rgb_image_empty_) 
  {

    this->blade_image.pushbackDepthImage(this->depth_image_msg_);
    this->blade_image.pushbackRGBImage(this->rgb_image_msg_);
    this->blade_image.createMask();
    this->blade_image.maskRGBImage();
    this->blade_image.getKeypoints();
    this->blade_image.getKeypointCoords();
    this->blade_image.publishMatchImage();
    this->blade_image.publishMaskedDepthImage();
    this->blade_image.publishMaskedRGBImage();

    this->blade_cloud.updatePointClouds(blade_image);
    this->blade_cloud.getPose(this->blade_delta_pose_);
    this->blade_cloud.publishClouds();

  }
}

void BladePoseNode::rxDepthImage(sensor_msgs::msg::Image::SharedPtr depth_image)
{
  this->depth_image_msg_ = depth_image;
  this->depth_image_empty_ = false;
}

void BladePoseNode::rxRGBLeftImage(sensor_msgs::msg::Image::SharedPtr left_image)
{
  this->rgb_image_msg_ = left_image;
  this->rgb_image_empty_ = false;
}

void BladeImage::pushbackDepthImage(const sensor_msgs::msg::Image::SharedPtr depth_image)
{
  // store the image message data for later
  this->depth_image_msg = depth_image;

  // bridge to translate data
  cv_bridge::CvImagePtr cv_ptr_depth_image;

  // copy ros image to cv image
  cv_ptr_depth_image = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat mono8_img = cv::Mat(cv_ptr_depth_image->image.size(), CV_8UC1);
  cv::convertScaleAbs(cv_ptr_depth_image->image, mono8_img, 100, 0.0);

  if(!this->depth_image_current.empty())
  {
    // update previous depth image if we have already completed one iteration
    this->depth_image_previous = this->depth_image_current.clone();
  }

  // update current depth image with incoming image
  this->depth_image_current = mono8_img.clone();
}

void BladeImage::pushbackRGBImage(const sensor_msgs::msg::Image::SharedPtr rgb_image)
{
  // bridge to translate data
  cv_bridge::CvImagePtr cv_ptr_rgb_image;

  // copy ros image to cv image
  cv_ptr_rgb_image = cv_bridge::toCvCopy(rgb_image);
  cv::Mat rgb_img = cv_ptr_rgb_image->image;

  if(!this->rgb_image_current.empty())
  {
    // update previous rgb image if we have already completed one iteration
    this->rgb_image_previous = this->rgb_image_current.clone();
  }

  // update current rgb image with incoming image
  this->rgb_image_current = rgb_img.clone();
}

void BladeImage::createMask()
{
  // upper and lower bound
  const cv::Scalar upper_bound(180);  // some fairly high value
  const cv::Scalar lower_bound(1);  // some fairly low

  // filters pixels, creating mask of pixels in range = 255, out of range = 0
  cv::inRange(this->depth_image_current, lower_bound, upper_bound, this->depth_image_mask);
}

void BladeImage::maskRGBImage()
{
  // masked rgb image
  cv::Mat rgb_masked;
  // generated depth mask to remove background
  cv::Mat depth_mask = this->depth_image_mask;

  // copy in the current time rgb image using depth mask to remove background
  this->rgb_image_current.copyTo(rgb_masked, depth_mask);
  // mask the rgb image & convert to 3 channel
  cv::cvtColor(rgb_masked, rgb_masked, cv::COLOR_BGRA2BGR);

  // filter out the gray color
  cv::Scalar lower_bound(50, 30, 33);
  cv::Scalar upper_bound(170, 120, 100);
  cv::Mat gray_mask;
  cv::inRange(rgb_masked, lower_bound, upper_bound, gray_mask);

  cv::Scalar lower_bound_metal(170, 120, 100);
  cv::Scalar upper_bound_metal(255, 255, 255);
  cv::Mat metal_mask;
  cv::inRange(rgb_masked, lower_bound_metal, upper_bound_metal, metal_mask);

  cv::Mat gray_filtered;
  cv::Mat inverted_gray_mask;
  cv::bitwise_not(gray_mask, inverted_gray_mask);
  cv::bitwise_and(rgb_masked, rgb_masked, gray_filtered, inverted_gray_mask);

  cv::Mat metal_filtered;
  cv::Mat inverted_metal_mask;
  cv::bitwise_not(metal_mask, inverted_metal_mask);
  cv::bitwise_and(gray_filtered, gray_filtered, metal_filtered, inverted_metal_mask);

  cv::cvtColor(metal_filtered, metal_filtered, cv::COLOR_BGR2BGRA);

  // if we already have a masked rgb image (we already did an iteration)
  if(!this->rgb_image_current_masked.empty())
  {
    // then we copy it as the previous masked image before updating it
    this->rgb_image_previous_masked = this->rgb_image_current_masked.clone();
  }

  this->rgb_image_current_masked = metal_filtered.clone();
}

void BladeImage::getKeypoints()
{
  // Initialize ORB detector
  float scale_factor = 1.2f;
  int n_features = 3000;
  int n_levels = 8;
  int edge_threshold = 10;
  cv::Ptr<cv::ORB> detector = cv::ORB::create(n_features, scale_factor, n_levels, edge_threshold);
  
  // if we already have a descriptors image (we already did an iteration)
  if(!this->descriptors_image_current.empty())
  {
    // update previous image keypoints
    this->keypoints_vector_previous = this->keypoints_vector_current;
    this->descriptors_image_previous = this->descriptors_image_current.clone();
  }

  // Detect keypoints on new image
  detector->detectAndCompute(
    this->rgb_image_current_masked, 
    cv::Mat(), 
    this->keypoints_vector_current, 
    this->descriptors_image_current
    );
 
}

void BladeImage::getKeypointCoords()
{
  // if we already have a descriptors image (we already did an iteration)
  if(!this->descriptors_image_current.empty())
  {
    // var to store matches
    std::vector<cv::DMatch> matches;

    // Match features between the two images
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    matcher.match(this->descriptors_image_current, this->descriptors_image_previous, matches);

    // update class member matches
    this->matches = matches;

    //sort matches by distance
    std::sort(matches.begin(), matches.end());

    // Filter matches
    std::vector<cv::DMatch> good_matches;

    // Get the top 10% of matches
    int num_good_matches = matches.size() * 0.4;
    for (int i = 0; i < num_good_matches; i++)
    {
      good_matches.push_back(matches[i]);
    }

    if (static_cast<int>(good_matches.size()) < 8)
    {
      // TODO:Arturo -> might just crash if we don't have 8 keypoints. Follow this through
      // return;
    }

    for (auto & match : good_matches)
    {

      int x_1 = static_cast<int>(round(this->keypoints_vector_current[match.queryIdx].pt.x));
      int y_1 = static_cast<int>(round(this->keypoints_vector_current[match.queryIdx].pt.y));
      int x_2 = static_cast<int>(round(this->keypoints_vector_previous[match.queryIdx].pt.x));
      int y_2 = static_cast<int>(round(this->keypoints_vector_previous[match.queryIdx].pt.y));
      Eigen::Vector3i points_current(x_1, y_1, 1);
      Eigen::Vector3i points_previous(x_2, y_2, 1);

      this->points_pixels_current.push_back(points_current);
      this->points_pixels_previous.push_back(points_previous);
    }
  }

}

void BladeImage::publishMatchImage()
{
  // if we already have an rgb previous image (we already did an iteration)
  if(!this->rgb_image_previous.empty())
  {
    // draw matches
    cv::Mat keypoints_image;
    cv::drawMatches(
      this->rgb_image_current, 
      this->keypoints_vector_current, 
      this->rgb_image_previous, 
      this->keypoints_vector_previous, 
      this->matches, 
      keypoints_image
      );

    // publish image
    this->pub_keypoints_image_->publish(
      *cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", keypoints_image).toImageMsg());
  }
}

void BladeImage::publishMaskedDepthImage()
{
  this->pub_masked_depth_image_->publish(
    *cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", this->depth_image_mask).toImageMsg());

}

void BladeImage::publishMaskedRGBImage()
{
  this->pub_masked_rgb_image_->publish(
    *cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", this->rgb_image_current_masked).toImageMsg());
}

void BladeImage::getPixelDepth(int x, int y, double & depth_mm)
{
  // Get a pointer to the depth values casting the data
  // pointer to floating point
  float * depths = reinterpret_cast<float *>(&this->depth_image_msg->data[0]);

  // Image coordinates of the center pixel

  // Linear index of the center pixel
  int centerIdx = x + this->depth_image_msg->width * y;

  // Output the measure
  depth_mm = depths[centerIdx];

  if (depth_mm < 0 || std::isnan(depth_mm) || depth_mm != depth_mm) {
    depth_mm = 0.0;
  }
}

void BladeCloud::updatePointClouds(BladeImage& blade_image)
{
  if(!blade_image.points_pixels_previous.empty())
  {
    // update previous point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_new( *(this->current_cloud_) );
    this->previous_cloud_ = previous_cloud_new;

    // reset current point cloud (start from clean slate)
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
    this->current_cloud_ = current_cloud_new;
    
    // reset the target point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
    this->aligned_cloud_ = aligned_cloud_new;

    std::vector<Eigen::Vector3i> points_pixels_current = blade_image.points_pixels_current;
    std::vector<Eigen::Vector3i> points_pixels_previous = blade_image.points_pixels_previous;
  
    // stuff the current point cloud we are trying to match
    for (size_t i = 0; i < blade_image.points_pixels_current.size(); i++)
    {
      pcl::PointXYZ pcl_point;
      Eigen::Vector3i pixel = blade_image.points_pixels_current[i];
      double depth;

      int x = static_cast<int>(round(pixel[0]));
      int y = static_cast<int>(round(pixel[1]));
      blade_image.getPixelDepth( x, y, depth);

      if (depth < 0.4 || depth > 3)
      {
        // pass
      }

      else
      {
        Eigen::Vector3f point_3d = blade_image.K.inverse() * Eigen::Vector3f(pixel.cast<float>()) * static_cast<float>(depth);
        pcl_point.x = point_3d[0];
        pcl_point.y = point_3d[1];
        pcl_point.z = point_3d[2];
        this->current_cloud_->push_back(pcl_point);
      }
    }

    this->point_clouds_iterated = true;

  }
}

void BladeCloud::getPose(geometry_msgs::msg::PoseStamped& blade_pose)
{
  if(point_clouds_iterated)
  {
    // instantiate ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setInputTarget(this->previous_cloud_);
    icp.setInputSource(this->current_cloud_);
    // icp.setMaxCorrespondenceDistance(0.05);  // in meters
    icp.setMaximumIterations(12);
    icp.setTransformationEpsilon(1e-8);
    // icp.setEuclideanFitnessEpsilon(1);
    icp.align(*this->aligned_cloud_);

    // get the transform
    Eigen::Matrix4f transform = icp.getFinalTransformation();
    Eigen::Affine3f affine_transform(transform);
    // create rotation and translation matrices
    Eigen::Matrix3f rotationMatrix = transform.block<3, 3>(0, 0);

    // Convert rotation matrix to quaternion
    Eigen::Quaternionf quaternion(rotationMatrix);
    quaternion.normalize();

    blade_pose.pose.orientation.x = quaternion.x();
    blade_pose.pose.orientation.y = quaternion.y();
    blade_pose.pose.orientation.z = quaternion.z();
    blade_pose.pose.orientation.w = quaternion.w();
    blade_pose.pose.position.x = affine_transform.translation().x();
    blade_pose.pose.position.y = affine_transform.translation().y();
    blade_pose.pose.position.z = affine_transform.translation().z();
    // blade_pose.header.stamp = now();  // Assuming you have a function to get current time
    blade_pose.header.frame_id = "map";  // Typically, this would be something like "world" or "map"
  }
}

void BladeCloud::publishClouds()
{
  // create pcl2 ros message
  sensor_msgs::msg::PointCloud2 previous_cloud_msg;
  sensor_msgs::msg::PointCloud2 current_cloud_msg;
  sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
  
  // stuff message with data
  pcl::toROSMsg(*this->previous_cloud_, previous_cloud_msg);
  pcl::toROSMsg(*this->current_cloud_, current_cloud_msg);
  pcl::toROSMsg(*this->aligned_cloud_, aligned_cloud_msg);
  
  // give frame id
  previous_cloud_msg.header.frame_id = "map";
  current_cloud_msg.header.frame_id = "map";
  aligned_cloud_msg.header.frame_id = "map";
  this->pub_previous_cloud_->publish(previous_cloud_msg);
  this->pub_current_cloud_->publish(current_cloud_msg);
  this->pub_aligned_cloud_->publish(aligned_cloud_msg);
}

}  // end namespace blade_pose

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blade_pose::BladePoseNode)
