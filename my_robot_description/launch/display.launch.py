from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_path = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "urdf",
        "my_robot.urdf.xacro",
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "rviz",
        "urdf_config.rviz",
    ])

    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_path]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_joint_state_gui", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time
            }],
            output="screen",
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_joint_state_gui),
            output="screen",
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(use_joint_state_gui),
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config_path],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
