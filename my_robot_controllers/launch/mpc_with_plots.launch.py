from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    waypoints_flat = LaunchConfiguration("waypoints_flat")
    goal_tolerance = LaunchConfiguration("goal_tolerance")
    waypoint_radius = LaunchConfiguration("waypoint_radius")

    bringup_share = get_package_share_directory("my_robot_bringup")
    gazebo_launch = os.path.join(bringup_share, "launch", "my_robot_gazebo.launch.xml")

    controller = Node(
        package="my_robot_controllers",
        executable="mpc_controller",
        name="mpc_controller",
        output="screen",
        parameters=[{
            "waypoints_flat": waypoints_flat,
            "goal_tolerance": goal_tolerance,
        }],
    )

    plotter = Node(
        package="my_robot_controllers",
        executable="odom_plotter",
        name="odom_plotter",
        output="screen",
        parameters=[{
            "plot_name": "mpc_controller",
            "waypoints_flat": waypoints_flat,
            "goal_tolerance": goal_tolerance,
            "waypoint_radius": waypoint_radius,
            "plots_dir": "./plots",
        }],
    )

    shutdown_on_plot_done = RegisterEventHandler(
        OnProcessExit(
            target_action=plotter,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "waypoints_flat",
            default_value="[3.0,0.0, 6.0,4.0, 3.0,4.0, 3.0,1.0, 0.0,3.0]",
        ),
        DeclareLaunchArgument("goal_tolerance", default_value="0.2"),
        DeclareLaunchArgument("waypoint_radius", default_value="0.2"),

        IncludeLaunchDescription(AnyLaunchDescriptionSource(gazebo_launch)),
        controller,
        plotter,
        shutdown_on_plot_done,
    ])
