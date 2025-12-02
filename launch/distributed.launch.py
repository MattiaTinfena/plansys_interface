import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('plansys2_bringup')
    print ("bringup_dir:", bringup_dir)
    interface_dir = get_package_share_directory('plansys_interface')
    model_file = LaunchConfiguration('model_file')
    problem_file = LaunchConfiguration('problem_file')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    
    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(interface_dir, "domain", "example.pddl"),
        description='PDDL Model file'
    )

    declare_problem_file_cmd = DeclareLaunchArgument(
        'problem_file', 
        default_value=os.path.join(interface_dir, "domain", "problem.pddl"),
        description='PDDL Problem file')
        
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'plansys2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
        
    
    domain_expert_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_domain_expert'),
            'launch',
            'domain_expert_launch.py')),
        launch_arguments={
            'model_file': model_file,
            'namespace': namespace,
            'params_file': params_file
        }.items())
        
        
    problem_expert_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_problem_expert'),
            'launch',
            'problem_expert_launch.py')),
        launch_arguments={
            'model_file': model_file,
            'problem_file': problem_file,
            'namespace': namespace,
            'params_file': params_file
        }.items())

    planner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_planner'),
            'launch',
            'planner_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'params_file': params_file
        }.items())

   
    lifecycle_manager_cmd = Node(
        package='plansys2_lifecycle_manager',
        executable='lifecycle_manager_node',
        name='lifecycle_manager_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
        
    ld = LaunchDescription()

    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_problem_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    
    ld.add_action(domain_expert_cmd)
    ld.add_action(problem_expert_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(lifecycle_manager_cmd)
    
    return ld
