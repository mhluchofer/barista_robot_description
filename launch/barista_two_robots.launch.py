import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix


# this is the function launch  system will look for
def generate_launch_description():

    package_description = "barista_robot_description"
    install_dir = get_package_prefix(package_description)

    # This is to find the models
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share"

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define the launch arguments for the Gazebo launch file
    gazebo_launch_args = {
        'verbose': 'false',
        'pause': 'false'
    }

    # Include the Gazebo launch file with the modified launch arguments
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments=gazebo_launch_args.items(),
    )

    # Define the robot model files to be used
    print("Fetching URDF ==>")
    robot_desc_file = 'barista_robot_model.urdf.xacro'
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", robot_desc_file)

    robot_name_1 = "rick"
    robot_name_2 = "morty"
    robot_pos_1 = ['0.0','0.0','0.'] # x,y,z
    robot_pos_2 = ['1.0', '1.0', '0.'] # x,y,z

    rsp_robot1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_" + robot_name_1,
        namespace=robot_name_1,
        parameters=[{
            'frame_prefix': robot_name_1 + '/',
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ', robot_desc_path,
                ' robot_name:=', robot_name_1,
                ' include_laser:=true',
                ' body_color:=', 'red',
                ' body_gz_material:=', 'Gazebo/Red'
            ])
        }],
        output="screen"
    )

    rsp_robot2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_" + robot_name_2,
        namespace=robot_name_2,
        parameters=[{
            'frame_prefix': robot_name_2 + '/',
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ', robot_desc_path,
                ' robot_name:=', robot_name_2,
                ' include_laser:=true',
                ' body_color:=', 'blue',
                ' body_gz_material:=', 'Gazebo/Blue'
            ])
        }],
        output="screen"
    )
    
    # Spawn Rick robot 
    spawn_robot1 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', robot_name_1, '-x', robot_pos_1[0], '-y', robot_pos_1[1], '-z', robot_pos_1[2],
                                   '-topic', robot_name_1 + '/robot_description'],
                        output='screen')

    # Spawn Morty robot
    spawn_robot2 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', robot_name_2, '-x', robot_pos_2[0], '-y', robot_pos_2[1], '-z', robot_pos_2[2],
                                   '-topic', robot_name_2 + '/robot_description'],
                        output='screen')

    # Static world -> rick/odom (initial pose)
    static_tf_pub_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_' + robot_name_1,
        output="screen",
        arguments=['0', '0', '0','0','0','0', 'world', robot_name_1+'/odom']
    )

    # Static world -> morty/odom (initial pose)
    static_tf_pub_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_'+robot_name_2,
        output="screen",
        arguments=['0', '0', '0','0','0','0', 'world', robot_name_2+'/odom']
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'two_robots_vis.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir]
    )
    

    delayed_rviz = TimerAction(
        actions=[rviz_node],
        period=5.0
    )

    # create and return launch description object
    return LaunchDescription(
        [            
            gazebo,
            rsp_robot1,
            rsp_robot2,
            spawn_robot1,
            spawn_robot2,
            static_tf_pub_1,
            static_tf_pub_2,
            delayed_rviz
        ]
    )