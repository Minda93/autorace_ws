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
  use_sim_time = True

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
          {"use_sim_time": use_sim_time}],
        name='detect_lane'),
      ComposableNode(
        package='motion_control',
        plugin='motion_control::LaneControl',
        parameters=[
          os.path.join(
            pkgsPath.find("motion_control"),
            "config", "component", "lane_control.yaml"),
          {"use_sim_time": use_sim_time}],
        name='lane_control')
    ],
    output='screen',
  )

  return LaunchDescription([container])