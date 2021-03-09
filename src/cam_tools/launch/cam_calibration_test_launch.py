"""
  Launch
    rclcpp_components package
      component_container
        cam_tools package
          cam_tools::CamCalibration plugin
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():

  pkgsPath = FindPackageShare(["cam_tools"])

  container = ComposableNodeContainer(
    name='ComponentManager',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='cam_tools',
        plugin='cam_tools::CamCalibration',
        parameters=[
          os.path.join(
            pkgsPath.find("cam_tools"),"config","cam_calibration_component.yaml"),
        ],
        name='cam_calibration')
    ],
    output='screen',
  )

  return LaunchDescription([container])