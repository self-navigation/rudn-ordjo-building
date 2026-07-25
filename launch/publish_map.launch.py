"""Serve a baked floor map on /map, as a stand-in for live SLAM.

Pairs with spawn_floor.launch.py: that one puts the floor's meshes in Gazebo,
this one publishes the occupancy grid baked from those same meshes. Pass the
SAME floor_number to both, or the map will not match the walls.

This publishes the map only. It deliberately does NOT publish the map->odom
transform that SLAM would also provide -- that is the consuming workspace's
call, since it depends on how odometry is set up there.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def make_node(context):
    floor_number = LaunchConfiguration("floor_number").perform(context)
    map_yaml = LaunchConfiguration("map_yaml").perform(context)

    if not map_yaml:
        map_yaml = os.path.join(
            get_package_share_directory("rudn_ordjo_building"),
            "maps",
            f"floor_{floor_number}.yaml",
        )

    return [
        Node(
            package="rudn_ordjo_building",
            executable="map_publisher",
            name="map_publisher",
            output="screen",
            parameters=[
                {
                    "yaml_filename": map_yaml,
                    "frame_id": LaunchConfiguration("frame_id"),
                    "topic": LaunchConfiguration("topic"),
                    "publish_period": ParameterValue(
                        LaunchConfiguration("publish_period"), value_type=float
                    ),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "floor_number",
                default_value="3",
                description="Which floor's baked map to serve. Must match the "
                            "floor_number given to spawn_floor.launch.py.",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value="",
                description="Explicit path to a map YAML, overriding floor_number. "
                            "Use this to serve a hand-edited copy.",
            ),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("topic", default_value="/map"),
            # The grid is latched, so subscribers that respect transient-local
            # durability get it whenever they start. RViz's Map display can be
            # configured volatile, though, and then it silently never receives a
            # latched-only message -- so republish slowly as well. 1.4 MB every
            # 2 s is nothing next to the sim, and it makes "open RViz whenever"
            # just work. Set to 0 to publish once and rely on latching alone.
            DeclareLaunchArgument("publish_period", default_value="2.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=make_node),
        ]
    )
