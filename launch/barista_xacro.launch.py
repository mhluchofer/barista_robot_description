import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    package_name = "barista_robot_description"

    pkg_share = get_package_share_directory(package_name)
    install_dir = get_package_prefix(package_name)

    # =========================
    # Configura variables de entorno de Gazebo
    # =========================
    os.environ["GAZEBO_MODEL_PATH"] = os.environ.get("GAZEBO_MODEL_PATH", "") + ":" + os.path.join(install_dir, "share")
    os.environ["GAZEBO_PLUGIN_PATH"] = os.environ.get("GAZEBO_PLUGIN_PATH", "") + ":" + os.path.join(install_dir, "lib")

    # =========================
    # Archivo Xacro principal
    # =========================
    xacro_file = os.path.join(pkg_share, "xacro", "barista_robot_model.urdf.xacro")

    # =========================
    # Gazebo launch
    # =========================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    # =========================
    # Robot definitions
    # =========================
    robots = [
        {"name": "rick", "color": "red", "x": 0.0, "y": 0.0},
        {"name": "morty", "color": "blue", "x": 1.0, "y": 0.0},
    ]

    launch_nodes = [gazebo_launch]

    for robot in robots:
        robot_desc = Command([
            "xacro ", xacro_file,
            " robot_name:=", robot["name"],
            " robot_color:=", robot["color"]
        ])

        # Robot State Publisher
        rsp_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": True, "robot_description": robot_desc}],
        )

        # Spawn Entity
        spawn_node = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", robot["name"],
                "-topic", "robot_description",
                "-x", str(robot["x"]),
                "-y", str(robot["y"]),
                "-z", "0.1"
            ],
            output="screen",
        )

        # Retardo para RViz (opcional, solo para el primer robot)
        if robot["name"] == "rick":
            rviz_config = os.path.join(pkg_share, "rviz", "urdf_vis.rviz")
            rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=["-d", rviz_config],
            )

            delayed_rviz = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_node,
                    on_exit=[rviz_node],
                )
            )
            launch_nodes.extend([rsp_node, spawn_node, delayed_rviz])
        else:
            launch_nodes.extend([rsp_node, spawn_node])

    return LaunchDescription(launch_nodes)
