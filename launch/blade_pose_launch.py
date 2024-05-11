import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
import yaml

_BLADE_POSE_PARAMS_FILE = os.path.join(
  get_package_share_directory('blade_pose'),
  'launch',
  'params.yaml'
)

def generate_launch_description():
    #### blade_pose Config ####
    with open(_BLADE_POSE_PARAMS_FILE, 'r') as file:
      blade_pose_params = yaml.safe_load(file)

    blade_pose_node = LifecycleNode(
        package='blade_pose',
        executable='blade_pose_exe',
        name='blade_pose',
        namespace=TextSubstitution(text=''),
        parameters=[
           blade_pose_params
           ],
        output='screen')

    blade_pose_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=blade_pose_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(blade_pose_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    blade_pose_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=blade_pose_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(blade_pose_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )  

    return LaunchDescription([
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        blade_pose_node,
        blade_pose_configure_event_handler,
        blade_pose_activate_event_handler,
    ])