import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- PATHS ---
    pkg_grazen = get_package_share_directory('grazen_system')
    pkg_tb3_nav = get_package_share_directory('turtlebot3_navigation2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    
    # --- ARGUMENTS ---
    mode_arg = DeclareLaunchArgument('mode', default_value='sim', description='sim or real')
    task_arg = DeclareLaunchArgument('task', default_value='nav', description='nav or map')
    auto_map_arg = DeclareLaunchArgument('auto_map', default_value='true', description='true=Explore Lite, false=Manual Mapper')
    
    mode = LaunchConfiguration('mode')
    task = LaunchConfiguration('task')
    auto_map = LaunchConfiguration('auto_map')
    
    # --- CONFIG FILES ---
    nav_params = PathJoinSubstitution([pkg_grazen, 'config', 'grazen_params.yaml'])
    explore_params = PathJoinSubstitution([pkg_grazen, 'config', 'explore_params.yaml'])
    slam_params = PathJoinSubstitution([pkg_grazen, 'config', 'slam_params.yaml'])
    map_file = PathJoinSubstitution([pkg_grazen, 'maps', 'warehouse_map.yaml'])
    rviz_config = os.path.join(pkg_tb3_nav, 'rviz', 'tb3_navigation2.rviz')

    # ========================================================================
    # 1. SIMULATION LAYER (Gazebo + Robot)
    # ========================================================================
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_aws, 'worlds', 'small_warehouse', 'small_warehouse.world'),
            'extra_gazebo_args': '-s libgazebo_ros_state.so'
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )

    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': '-2.0', 'y_pose': '-0.5'}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )

    # ========================================================================
    # 2. VISUALIZATION LAYER (RViz)
    # ========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )

    # ========================================================================
    # 3. MAPPING MODE (Task = 'map')
    # ========================================================================
    
    # A. SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params, 
            {'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"])}
        ],
        condition=IfCondition(PythonExpression(["'", task, "' == 'map'"]))
    )

    # B. Nav2 (Mapping Config - NO AMCL)
    nav2_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
            'params_file': nav_params,
            'map_subscribe_transient_local': 'true'
        }.items(),
        condition=IfCondition(PythonExpression(["'", task, "' == 'map'"]))
    )

    # C1. Auto Mapper (Explore Lite) - Runs if auto_map='true'
    explore_lite = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_params, 
            {'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"])}
        ],
        condition=IfCondition(PythonExpression(["'", task, "' == 'map' and '", auto_map, "' == 'true'"]))
    )

    # C2. Manual Mapper (Custom Teleop + Save) - Runs if auto_map='false'
    manual_mapper = Node(
        package='grazen_system',
        executable='manual_mapper',
        name='manual_mapper',
        output='screen',
        prefix='xterm -e', # Opens in new window
        parameters=[{
            'map_path': os.path.join(pkg_grazen, 'maps', 'warehouse_map')
        }],
        condition=IfCondition(PythonExpression(["'", task, "' == 'map' and '", auto_map, "' == 'false'"]))
    )

    # ========================================================================
    # 4. NAVIGATION MODE (Task = 'nav')
    # ========================================================================
    
    nav2_mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_nav, 'launch', 'navigation2.launch.py')),
        launch_arguments={
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
            'map': map_file,
            'params_file': nav_params
        }.items(),
        condition=IfCondition(PythonExpression(["'", task, "' == 'nav'"]))
    )

    mission_manager = Node(
        package='grazen_system',
        executable='mission_manager',
        output='screen',
        parameters=[{
            'mode': mode,
            'task': task,
            'waypoints_file': PathJoinSubstitution([pkg_grazen, 'config', 'waypoints.txt'])
        }],
        condition=IfCondition(PythonExpression(["'", task, "' == 'nav'"]))
    )

    return LaunchDescription([
        mode_arg,
        task_arg,
        auto_map_arg,
        gz_server,
        gz_client,
        robot_state_publisher,
        robot_spawn_launch,
        rviz_node,
        slam_toolbox,
        nav2_mapping,
        explore_lite,
        manual_mapper,
        nav2_mission,
        mission_manager
    ])