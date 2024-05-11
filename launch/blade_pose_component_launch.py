import os
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml

from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.conditions import LaunchConfigurationEquals

_BLADE_POSE_PARAMS_FILE = os.path.join(
  get_package_share_directory('blade_pose'),
  'launch',
  'params.yaml'
)

def generate_launch_description():
  launch_description = []

  with open(_BLADE_POSE_PARAMS_FILE, 'r') as file:
    params = yaml.safe_load(file)

  container = ComposableNodeContainer(
    name='can_driver_container',
    namespace='/drivers/can',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='blade_pose',
        plugin='blade_pose::BladePoseNode',
        name='bladepose',
        parameters=[
          params
          ],
        extra_arguments=[{'use_intra_process_comms': True}],
        ),      
    ],
    output='both'
  )

  launch_description.append(container)

  return LaunchDescription(launch_description)