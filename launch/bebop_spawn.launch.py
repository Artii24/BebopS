import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('bebop_simulator_r2'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    user_account = LaunchConfiguration('user_account', default='artem')
    waypoint_filter = LaunchConfiguration('waypoint_filter', default='true')
    EKFActive = LaunchConfiguration('EKFActive', default='false')
    csvFilesStoring = LaunchConfiguration('csvFilesStoring', default='false')
    csvFilesStoringTime = LaunchConfiguration('csvFilesStoringTime', default='60.0')

    world = os.path.join(
        get_package_share_directory('bebop_simulator_r2'),
        'worlds',
        'basic.world'
    )

    pkg_box_car_description = get_package_share_directory('bebop_simulator_r2')
    xacro_file = os.path.join(get_package_share_directory('bebop_simulator_r2'), 'urdf/', 'bebop.urdf.xacro')    
    assert os.path.exists(xacro_file), "The box_bot.xacro doesnt exist in "+str(xacro_file)

    install_dir = get_package_prefix('bebop_simulator_r2')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Get the urdf file
    params_model={'radius': '0.9',
            'enable_ground_truth': 'true',
            'enable_odometry_sensor_with_noise': 'false',
            'disable_odometry_sensor_with_noise': 'true',
            'enable_ground_truth_sensor': 'false',
            'enable_wind_plugin': 'false',
            'enable_laser1D': 'false',
            'enable_imu': 'false',
            'namespace': 'bebop',
            'wind_force': '0.0',
            'wind_start': '0.0',
            'wind_duration': '0.0',
            'wind_direction_x': '0.0',
            'wind_direction_y': '0.0',
            'wind_direction_z': '0.0',
            'x': '0.0',
            'y': '0.0',
            'z': '0.31',
            'model': 'bebop'
            }

    robot_description_config = xacro.process_file(xacro_file, mappings=params_model)
    robot_desc = robot_description_config.toxml()

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    declare_time_cmd =DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_user_account_cmd =DeclareLaunchArgument(
        'user_account',
        default_value='artem',
        description='Use simulation (Gazebo) clock if true')
        
    declare_waypoint_filter_cmd =DeclareLaunchArgument(
        'waypoint_filter',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_EKFActive_cmd =DeclareLaunchArgument(
        'EKFActive',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_csvFilesStoring_cmd =DeclareLaunchArgument(
        'csvFilesStoring',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_csvFilesStoringTime_cmd =DeclareLaunchArgument(
        'csvFilesStoringTime',
        default_value='60.0',
        description='Use simulation (Gazebo) clock if true')
        
    robot_state_publisher_cmd = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{'robot_description': robot_desc}
            ],
            output='both')
    param_launch = {'user_account' :"artem",
        'waypoint_filter' :"true",
        'EKFActive' :"false",
        'csvFilesStoring' :"false",
        'csvFilesStoringTime':"60.0"}
    postion_contrl_cmd =Node(package='bebop_simulator_r2', executable='position_controller_node', arguments=[param_launch], output='screen',
        parameters=[
        ])
    
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    spawn_bebop_cmd = Node(package='bebop_simulator_r2', executable='spawn_bebop.py', arguments=[robot_desc], output='screen')
    # start_gazebo_ros_spawner_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', "robot",
    #         '-file', robot_description_config,
    #         '-x', x_pose,
    #         '-y', y_pose,
    #         '-z', z_pose
    #     ],
    #     output='screen',
    # )
    ld =LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_time_cmd)
    ld.add_action(declare_user_account_cmd)   
    ld.add_action(declare_waypoint_filter_cmd)
    ld.add_action(declare_EKFActive_cmd)   
    ld.add_action(declare_csvFilesStoring_cmd)
    ld.add_action(declare_csvFilesStoringTime_cmd)   
    
    # Add the commands to the launch description
    # ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_bebop_cmd)
    # ld.add_action(postion_contrl_cmd)
    return ld
