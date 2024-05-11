cmake_minimum_required(VERSION 3.5)

project(blade_pose)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED core features2d imgproc)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/blade_pose_node.cpp
)



target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-function)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY}
${PCL_LIBRARIES})

ament_target_dependencies(blade_pose
  rclcpp
  std_msgs 
  sensor_msgs 
  image_transport 
  cv_bridge
  pcl_conversions
  tf2_geometry_msgs
  tf2_ros
  )

rclcpp_components_register_node(blade_pose
  PLUGIN "blade_pose::BladePoseNode"
  EXECUTABLE blade_pose_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)