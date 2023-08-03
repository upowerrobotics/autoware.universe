# Copyright 2023 U Power Robotics, USA

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # arguments
    input_lane_driving_trajectory_arg = DeclareLaunchArgument(
        name="input_lane_driving_trajectory",
    )

    input_parking_trajectory_arg = DeclareLaunchArgument(
        name="input_parking_trajectory",
    )

    input_lanelet_map_arg = DeclareLaunchArgument(
        name="input_lanelet_map",
    )

    input_route_arg = DeclareLaunchArgument(
        name="input_route",
    )

    input_odometry_arg = DeclareLaunchArgument(
        name="input_odometry",
    )

    is_parking_completed_arg = DeclareLaunchArgument(
        name="is_parking_completed",
    )

    output_scenario_arg = DeclareLaunchArgument(
        name="output_scenario",
    )

    output_trajectory_arg = DeclareLaunchArgument(
        name="output_trajectory",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulated time",
    )

    scenario_selector_node = Node(
        package="scenario_selector",
        executable="scenario_selector",
        name="scenario_selector",
        output="screen",
        remappings=[
            ("input/lane_driving/trajectory", LaunchConfiguration("input_lane_driving_trajectory")),
            ("input/parking/trajectory", LaunchConfiguration("input_parking_trajectory")),
            ("input/lanelet_map", LaunchConfiguration("input_lanelet_map")),
            ("input/route", LaunchConfiguration("input_route")),
            ("input/odometry", LaunchConfiguration("input_odometry")),
            ("is_parking_completed", LaunchConfiguration("is_parking_completed")),

            ("output/scenario", LaunchConfiguration("output_scenario")),
            ("output/trajectory", LaunchConfiguration("output_trajectory")),
        ],
        parameters=[
            {"update_rate": 10.0},
            {"th_max_message_delay_sec": 1.0},
            {"th_arrived_distance_m": 1.0},
            {"th_stopped_time_sec": 1.0},
            {"th_stopped_velocity_mps": 0.01},
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
    )

    return LaunchDescription([
        input_lane_driving_trajectory_arg,
        input_parking_trajectory_arg,
        input_lanelet_map_arg,
        input_route_arg,
        input_odometry_arg,
        is_parking_completed_arg,
        output_scenario_arg,
        output_trajectory_arg,
        use_sim_time_arg,
        scenario_selector_node
    ])
