from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription
import yaml
import os
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def generate_launch_description():
	"""Simple launch to visualize the robot URDF in RViz.

	Provides launch arguments:
	- urdf_path: path to the URDF file to load (defaults to package URDF)
	- use_gui: whether to run joint_state_publisher_gui (true/false)
	- rviz_config: optional rviz config file path
	"""
	from launch.substitutions import LaunchConfiguration
	from launch.actions import OpaqueFunction

	declare_urdf = DeclareLaunchArgument(
		'urdf_path',
		default_value=os.path.join(get_package_share_directory('rm26_version1_engineering_model'), 'urdf', 'rm26_version1_engineering_model.urdf'),
		description='Absolute path to robot URDF file')

	declare_use_gui = DeclareLaunchArgument('use_gui', default_value='true', description='Run joint_state_publisher_gui')
	declare_rviz = DeclareLaunchArgument('rviz_config', default_value='', description='Optional RViz config file')

	def _create_nodes(context, *args, **kwargs):
		urdf_path = LaunchConfiguration('urdf_path').perform(context)
		use_gui = LaunchConfiguration('use_gui').perform(context).lower() in ('1', 'true', 'yes')
		rviz_config = os.path.join(get_package_share_directory('rm26_version1_engineering_model'), 'config', 'rviz.rviz'),

		# load urdf
		robot_description = ''
		try:
			with open(urdf_path, 'r') as f:
				robot_description = f.read()
		except Exception as e:
			raise RuntimeError(f"Failed to read URDF file '{urdf_path}': {e}")

		nodes = []

		# robot_state_publisher
		nodes.append(Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name='robot_state_publisher',
			output='screen',
			parameters=[{'robot_description': robot_description}]
		))

		# joint_state_publisher (GUI optional)
		if use_gui:
			nodes.append(Node(
				package='joint_state_publisher_gui',
				executable='joint_state_publisher_gui',
				name='joint_state_publisher_gui',
				output='screen'
			))
		else:
			nodes.append(Node(
				package='joint_state_publisher',
				executable='joint_state_publisher',
				name='joint_state_publisher',
				output='screen'
			))

		# rviz
		rviz_args = []
		if rviz_config:
			rviz_args = ['-d', rviz_config]
		nodes.append(Node(
			package='rviz2',
			executable='rviz2',
			name='rviz2',
			output='screen',
			arguments=rviz_args
		))

		return nodes

	return LaunchDescription([
		declare_urdf,
		declare_use_gui,
		declare_rviz,
		OpaqueFunction(function=_create_nodes)
	])
