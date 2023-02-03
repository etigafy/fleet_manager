from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():

    namespaceArray = ["", "/tt1"]

    # arguments
    ARGUMENTS = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'model',
            default_value='standard',
            choices=['standard', 'lite'],
            description='Turtlebot4 Model'
        ),
        # DeclareLaunchArgument(
        #     'description',
        #     default_value='false',
        #     description='Launch turtlebot4 description'
        # ),
    ]

# ---------------------------------------------------------------------------------------------------------
    # substitutions
    pkg_tt_cpp = get_package_share_directory('tt_cpp')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')

    rviz2_config = PathJoinSubstitution(
        [pkg_tt_cpp, 'rviz', 'robot.rviz'])
    description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py']
    )

# ---------------------------------------------------------------------------------------------------------
    # init nodes
    logger = Node(
        package='tt_cpp',
        executable='orientation_logger',
        # name='orientation_logger_node',
        output='screen',
        namespace=namespaceArray[0]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        # name='rviz2',
        arguments=['-d', rviz2_config],
        output='screen',
        namespace=namespaceArray[0],
        # remappings= [
        #     ("poselogger", "poselogger123")
        # ]
        )

    # # Delay launch of robot description to allow Rviz2 to load first.
    # # Prevents visual bugs in the model.
    # robot_description = TimerAction(
    #     period=3.0,
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([description_launch]),
    #             launch_arguments=[('model', LaunchConfiguration('model'))],
    #             condition=IfCondition(LaunchConfiguration('description'))
    #         )]
    # )

    # create launch description object
    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(robot_description)
    ld.add_action(rviz2)

    ld.add_action(logger)

    return ld