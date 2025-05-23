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
from launch.actions import ExecuteProcess
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

import os

def generate_launch_description():

    imu_to_rpy = LifecycleNode(
        package='autonomy_utils',
        executable='imu_to_rpy_node',
        name='imu_to_rpy_node',
        namespace=TextSubstitution(text=''),
        parameters=[LaunchConfiguration('param_file')],
        output='screen',
    )

    imu_to_rpy_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=imu_to_rpy,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_to_rpy),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    imu_to_rpy_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=imu_to_rpy,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_to_rpy),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('param_file', default_value=os.path.join(get_package_share_directory('autonomy_perception'), 'config', 
                                                                   'perception_logger.yaml')),
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        imu_to_rpy,
        imu_to_rpy_configure_event_handler,
        imu_to_rpy_activate_event_handler,
    ])
