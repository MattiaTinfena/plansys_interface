import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_erl2 = get_package_share_directory('erl2')
    # pkg_exp_rob_ass2 = get_package_share_directory('exp_rob_ass2')

    namespace = LaunchConfiguration('namespace')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    erl2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_erl2, 'launch', 'erl2_simulation.launch.py'])
        ),
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )


    navigation_params_path = os.path.join(
        get_package_share_directory('exp_rob_ass2'),
        'config',
        'navigation.yaml'
    )

    # Path to the Slam Toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        get_package_share_directory('exp_rob_ass2'),
        'config',
        'slam_toolbox_mapping.yaml'
    ) 

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
        }.items()
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    go_to_point_cmd = Node(
        package='exp_rob_ass2',
        executable='go_to_point_action',
        name='go_to_point',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    launchDescriptionObject = LaunchDescription()
    
    launchDescriptionObject.add_action(erl2_launch)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(slam_toolbox_launch)
    launchDescriptionObject.add_action(navigation_launch)
    launchDescriptionObject.add_action(declare_namespace_cmd)
    launchDescriptionObject.add_action(go_to_point_cmd)

    return launchDescriptionObject