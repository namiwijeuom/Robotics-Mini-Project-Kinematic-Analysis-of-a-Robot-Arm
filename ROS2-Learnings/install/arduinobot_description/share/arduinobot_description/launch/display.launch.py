from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=os.path.join(get_package_share_directory("arduinobot_description"), "urdf", "arduinobot.urdf.xacro"),
        description='Absolute path to robot URDF file'
        )
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("arduinobot_description"), "rviz", "display.rviz")]
    
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui,
        rviz_node
    ])

        # # Include the launch file that sets the parameters
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('arduinobot_description'), 'launch', 'parameters.launch.py')]
        #     )
        # ),
        # # Include the launch file that spawns the robot
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('arduinobot_description'), 'launch', 'spawn.launch.py')]
        #     )
        # ),
        # # Include the launch file that starts the robot state publisher
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('arduinobot_description'), 'launch', 'state_publisher.launch.py')]
        #     )
        # ),
        # # Include the launch file that starts the joint state publisher
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('arduinobot_description'), 'launch', 'joint_state_publisher.launch.py')]
        #     )
        # ),
        # # Include the launch file that starts rviz
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('arduinobot_description'), 'launch', 'rviz.launch.py')]
        #     )
        # ),
        # # Include the launch file that starts gazebo
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('arduinobot_description'), 'launch', 'gazebo.launch.py')]
        #
   
        # Include the launch file that actually does the work
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory('arduinobot_description'), 'launch', 'display.launch.py')]
        #     )
        # )],
        # # Set the parameters
        # DeclareLaunchArgument('model', default_value='arduinobot'),
        # DeclareLaunchArgument('x', default_value='0'),
        # DeclareLaunchArgument('y', default_value='0'),
        # DeclareLaunchArgument('z', default_value='0'),
        # DeclareLaunchArgument('roll', default_value='0'),
        # DeclareLaunchArgument('pitch', default_value='0'),
        # DeclareLaunchArgument('yaw', default_value='0'),
        # DeclareLaunchArgument('rviz', default_value='true'),
        # DeclareLaunchArgument('rviz_config', default_value='arduinobot.rviz'),
        # DeclareLaunchArgument('rviz_args', default_value='-d'),
        # DeclareLaunchArgument('gazebo', default_value='true'),
        # DeclareLaunchArgument('gazebo_world', default_value='arduinobot.world'),
        # DeclareLaunchArgument('gazebo_args', default_value='-s libgazebo_ros_init.so'),
        # DeclareLaunchArgument('gazebo_gui', default_value='true'),
        # DeclareLaunchArgument('gazebo_gui_args', default_value=''),
        # DeclareLaunchArgument('gazebo_paused', default_value='false'),
        # DeclareLaunchArgument('gazebo_paused_args', default_value=''),
        # DeclareLaunchArgument('gazebo_headless', default_value='false'),
        # DeclareLaunchArgument('gazebo_headless_args', default_value=''),
        # DeclareLaunchArgument('gazebo_debug', default_value='false'),
        # DeclareLaunchArgument('gazebo_debug_args', default_value=''),
        # DeclareLaunchArgument('gazebo_verbose', default_value='false'),
        # DeclareLaunchArgument('gazebo_verbose_args', default_value=''),
        # DeclareLaunchArgument('gazebo_ros', default_value='true'),
        # DeclareLaunchArgument('gazebo_ros_args', default_value=''),
        # DeclareLaunchArgument('gazebo_ros_verbose', default_value='false'),
        # DeclareLaunchArgument('gazebo_ros_verbose_args', default_value=''),
        # DeclareLaunchArgument('gazebo_ros_debug', default_value=')]