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

    clientID = LaunchConfiguration("clientID")

    # clientID_launch_arg = DeclareLaunchArgument(
    #     "clientID",
    #     default_value="1"
    # )

    namespaceArray = ["", "/tt1"]

    turtleClient = Node(
        package='mci_rmf',
        executable='turtle_client',
        # name='rviz2',
        parameters=[
            {"clientID": clientID}
        ],
        output='screen',
        namespace=namespaceArray[0],
        remappings= [
            (namespaceArray[0] + "/clientState", "/clientState"),
            (namespaceArray[0] + "/clientTask", "/clientTask")
        ]
        )

    # create launch description object
    ld = LaunchDescription()
    ld.add_action(turtleClient)

    # ld.add_action(logger)

    return ld