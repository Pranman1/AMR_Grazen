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
    sim_world_arg = DeclareLaunchArgument('sim_world', default_value='warehouse', description='warehouse or arena')
    map_name_arg = DeclareLaunchArgument('map_name', default_value='', description='Override map name')
    
    mode = LaunchConfiguration('mode')
    task = LaunchConfiguration('task')
    auto_map = LaunchConfiguration('auto_map')
    sim_world = LaunchConfiguration('sim_world')
    map_name_input = LaunchConfiguration('map_name')
    
    # --- DYNAMIC MAP NAME LOGIC ---
    # If map_name provided: use it
    # If sim mode: use sim_world_map (arena_map, warehouse_map)
    # If real mode: use 'real_map' as default
    final_map_name = PythonExpression([
        "'", map_name_input, "' if '", map_name_input, "' != '' else ",
        "('", sim_world, "_map' if '", mode, "' == 'sim' else 'real_map')"
    ])

    # --- PATHS CONSTRUCTION ---
    nav_params = PathJoinSubstitution([pkg_grazen, 'config', 'grazen_params.yaml'])
    explore_params = PathJoinSubstitution([pkg_grazen, 'config', 'explore_params.yaml'])
    slam_params = PathJoinSubstitution([pkg_grazen, 'config', 'slam_params.yaml'])
    
    # Map file path (Points to install/grazen_system/share/grazen_system/maps/[name])
    map_base_path = PathJoinSubstitution([pkg_grazen, 'maps', final_map_name])
    
    # RViz Config
    rviz_config = os.path.join(pkg_tb3_nav, 'rviz', 'tb3_navigation2.rviz')

    # ========================================================================
    # 1. SIMULATION LAYER (Gazebo + Robot)
    # ========================================================================
    
    gz_server_warehouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_aws, 'worlds', 'small_warehouse', 'small_warehouse.world'),
            'extra_gazebo_args': '-s libgazebo_ros_state.so'
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim' and '", sim_world, "' == 'warehouse'"]))
    )

    gz_server_arena = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world'),
            'extra_gazebo_args': '-s libgazebo_ros_state.so'
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim' and '", sim_world, "' == 'arena'"]))
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
    # Launch RViz in mapping mode (sim or real); nav2_mission handles its own RViz in nav mode
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(PythonExpression(["'", task, "' == 'map'"]))
    )

    # ========================================================================
    # 3. MAPPING MODE (Task = 'map')
    # ========================================================================
    
    # FIX: Laser Scan Resampler (Real Robot Only)
    # Problem: Real LDS-02 produces 250-256 readings/scan (variable due to motor timing)
    #          SLAM Toolbox locks to first scan count, rejects mismatches → intermittent mapping
    # Solution: Resample all scans to fixed 360 readings via linear interpolation
    #          Input: /scan (variable) → Output: /scan_filtered (constant 360)
    # Impact: Sim unaffected (already has constant scans, this node won't run)
    scan_resampler = Node(
        package='grazen_system',
        executable='scan_resampler',
        name='scan_resampler',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real' and '", task, "' == 'map'"]))
    )
    
    # A. SLAM Toolbox - SIM MODE (uses raw /scan with constant 360 readings)
    slam_toolbox_sim = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim' and '", task, "' == 'map'"]))
    )
    
    # A. SLAM Toolbox - REAL MODE (uses /scan_filtered with resampled 360 readings)
    slam_toolbox_real = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}],
        remappings=[('scan', 'scan_filtered')],  # ✅ Simple string remapping (no PythonExpression)
        condition=IfCondition(PythonExpression(["'", mode, "' == 'real' and '", task, "' == 'map'"]))
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

    # C1. Auto Mapper (Explore Lite)
    explore_lite = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_params, {'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"])}],
        condition=IfCondition(PythonExpression(["'", task, "' == 'map' and '", auto_map, "' == 'true'"]))
    )
    
    # C1b. Auto Mapper Monitor (watches explore_lite, saves map when done)
    auto_mapper = Node(
        package='grazen_system',
        executable='auto_mapper',
        name='auto_mapper',
        output='screen',
        parameters=[{
            'map_path': PathJoinSubstitution([map_base_path]),
            'completion_timeout': 10.0,  # Seconds with no frontiers = done
            'save_interval': 0.0         # Disabled backup saves (0 = off)
        }],
        condition=IfCondition(PythonExpression(["'", task, "' == 'map' and '", auto_map, "' == 'true'"]))
    )

    # C2. Manual Mapper (FIXED: Saves to install directory)
    manual_mapper = Node(
        package='grazen_system',
        executable='manual_mapper',
        name='manual_mapper',
        output='screen',
        prefix='xterm -e', # Opens in new window
        parameters=[{
            'map_path': PathJoinSubstitution([map_base_path]) # <--- PASSES BASE NAME
        }],
        condition=IfCondition(PythonExpression(["'", task, "' == 'map' and '", auto_map, "' == 'false'"]))
    )

    # ========================================================================
    # 4. NAVIGATION MODE (Task = 'nav')
    # ========================================================================
    
    # FIX: Nav Mission now constructs the map path correctly
    nav2_mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_nav, 'launch', 'navigation2.launch.py')),
        launch_arguments={
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
            # The Nav2 mission needs the full path WITH the .yaml extension
            'map': [map_base_path, '.yaml'], 
            'params_file': nav_params,
            'use_rviz': 'False' # Disable double RViz
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
        sim_world_arg,
        map_name_arg,
        gz_server_warehouse,
        gz_server_arena,
        gz_client,
        robot_state_publisher,
        robot_spawn_launch,
        rviz_node,
        scan_resampler,      # Resample variable scans to fixed 360 (real mode only)
        slam_toolbox_sim,    # SLAM for sim mode (uses /scan)
        slam_toolbox_real,   # SLAM for real mode (uses /scan_filtered)
        nav2_mapping,
        explore_lite,
        auto_mapper,         # Monitors explore_lite, saves map when done (auto_map mode only)
        manual_mapper,
        nav2_mission,
        mission_manager
    ])