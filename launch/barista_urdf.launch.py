import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    package_description = "barista_robot_description"
    install_dir = get_package_prefix(package_description)

    # --- Gazebo paths setup ---
    os.environ['GAZEBO_MODEL_PATH'] = os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + install_dir + '/share'
    os.environ['GAZEBO_PLUGIN_PATH'] = os.environ.get('GAZEBO_PLUGIN_PATH', '') + ':' + install_dir + '/lib'

    print("GAZEBO MODELS PATH==", os.environ["GAZEBO_MODEL_PATH"])
    print("GAZEBO PLUGINS PATH==", os.environ["GAZEBO_PLUGIN_PATH"])

    # --- Launch Gazebo ---
    gazebo_launch_args = {'verbose': 'false', 'pause': 'false'}
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments=gazebo_launch_args.items(),
    )

    # --- Robot description (URDF) ---
    urdf_file = 'barista_robot_model.urdf'
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': open(robot_desc_path).read()}],
        output='screen'
    )

    # --- Spawn robot (delayed) ---
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'barista_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.2'],
        output='screen'
    )

    # Delay the spawn for a few seconds (to ensure Gazebo fully loads)
    delayed_spawn = TimerAction(period=7.0, actions=[spawn_robot])

    # --- RViz ---
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    delayed_rviz = TimerAction(period=10.0, actions=[rviz_node])

    # --- Build final launch description ---
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn,
        delayed_rviz
    ])
