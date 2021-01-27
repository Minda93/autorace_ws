"""
  Launch
    rclcpp_components package
      component_container
        detect_object package
          detect_object::DetectLane
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

  pkgsPath = FindPackageShare(["detect_object"])

  container = ComposableNodeContainer(
    name='ComponentManager',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='detect_object',
        plugin='detect_object::DetectLane',
        name='detect_lane')
    ],
    output='screen',
  )

  return LaunchDescription([container])