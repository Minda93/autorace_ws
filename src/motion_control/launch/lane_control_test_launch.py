"""
  Launch
    rclcpp_components package
      component_container
        detect_object package
          detect_object::DetectLane
        motion_control package
          motion_control::LaneControl
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

  pkgsPath = FindPackageShare(["detect_object", "motion_control"])

  container = ComposableNodeContainer(
    name='ComponentManager',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='detect_object',
        plugin='detect_object::DetectLane',
        parameters=[
          os.path.join(
            pkgsPath.find("detect_object"),
            "config", "component", "detect_lane.yaml"),
          {"use_sim_time": True}],
        name='detect_lane'),
      ComposableNode(
        package='motion_control',
        plugin='motion_control::LaneControl',
        name='lane_control')
    ],
    output='screen',
  )

  return LaunchDescription([container])