import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_name = "barista_robot_description"

    pkg_share = get_package_share_directory(package_name)
    install_dir = get_package_prefix(package_name)

    # Configura variables de entorno de Gazebo
    os.environ["GAZEBO_MODEL_PATH"] = os.environ.get("GAZEBO_MODEL_PATH", "") + ":" + os.path.join(install_dir, "share")
    os.environ["GAZEBO_PLUGIN_PATH"] = os.environ.get("GAZEBO_PLUGIN_PATH", "") + ":" + os.path.join(install_dir, "lib")

    # Archivo Xacro principal
    xacro_file = os.path.join(pkg_share, "xacro", "barista_robot_model.urdf.xacro")
    robot_description = Command(["xacro ", xacro_file])

    # Nodo Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    # Nodo RViz
    rviz_config = os.path.join(pkg_share, "rviz", "urdf_vis.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config],
    )

    # Lanza Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )


    # Spawnea el robot en Gazebo
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "barista_robot",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1"  # Ajusta la altura si quieres que no quede “caído”
        ],
        output="screen",
    )

    # Retraso: lanzar RViz después de spawnear el robot
    delayed_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        delayed_rviz,
    ])
