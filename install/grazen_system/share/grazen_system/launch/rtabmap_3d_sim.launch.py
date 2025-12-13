# ==============================================================================
# RTAB-Map 3D SLAM Simulation Launch File
# ==============================================================================
# Generates 2D occupancy grid (for Nav2) + 3D point cloud (for drones)
# Uses TurtleBot3 Waffle with modified depth camera + LDS LiDAR
#
# Usage:
#   ros2 launch grazen_system rtabmap_3d_sim.launch.py
#   ros2 launch grazen_system rtabmap_3d_sim.launch.py world:=house
#   ros2 launch grazen_system rtabmap_3d_sim.launch.py world:=arena
#
# Teleop (in separate terminal):
#   ros2 run turtlebot3_teleop teleop_keyboard
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- PATHS ---
    pkg_grazen = get_package_share_directory('grazen_system')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # --- ARGUMENTS ---
    world_arg = DeclareLaunchArgument(
        'world', 
        choices=['arena', 'house', 'warehouse'],
        description='Gazebo world: arena, house, or warehouse (REQUIRED)'
    )
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch in localization mode (use existing map)'
    )
    
    world = LaunchConfiguration('world')
    localization = LaunchConfiguration('localization')
    
    # --- RTAB-MAP PARAMETERS (from working demo) ---
    rtabmap_params = {
        'frame_id': 'base_footprint',
        'use_sim_time': True,
        'subscribe_rgbd': True,         # Use synced RGBD
        'subscribe_scan': True,         # Fuse LiDAR
        'use_action_for_goal': True,
        # Registration
        'Reg/Strategy': '1',            # ICP (trust LiDAR)
        'Reg/Force3DoF': 'true',        # 2D robot
        'RGBD/NeighborLinkRefining': 'True',
        # Grid Generation
        'Grid/RayTracing': 'true',      # Fill empty space
        'Grid/3D': 'false',             # 2D occupancy for Nav2
        'Grid/RangeMax': '3.5',
        'Grid/RangeMin': '0.2',         # Ignore robot body
        'Grid/Sensor': '2',             # BOTH camera + LiDAR
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05', # Floor threshold
        'Grid/MaxObstacleHeight': '0.4',
        # Optimizer
        'Optimizer/GravitySigma': '0',  # Disable IMU (2D)
    }
    
    # Topic remappings for TurtleBot3 Waffle camera
    rtabmap_remappings = [
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
    ]
    
    # ========================================================================
    # 1. GAZEBO SIMULATION (Using system.launch.py pattern)
    # ========================================================================
    
    # Warehouse World
    gz_server_warehouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_aws, 'worlds', 'small_warehouse', 'small_warehouse.world'),
        }.items(),
        condition=IfCondition(PythonExpression(["'", world, "' == 'warehouse'"]))
    )
    
    # Arena World (turtlebot3_world)
    gz_server_arena = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world'),
        }.items(),
        condition=IfCondition(PythonExpression(["'", world, "' == 'arena'"]))
    )
    
    # House World
    gz_server_house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_house.world'),
        }.items(),
        condition=IfCondition(PythonExpression(["'", world, "' == 'house'"]))
    )
    
    # Gazebo Client (GUI)
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    
    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Spawn TurtleBot3 Waffle
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'x_pose': '-2.0',
            'y_pose': '-0.5'
        }.items()
    )
    
    # ========================================================================
    # 2. RTAB-MAP SLAM (from working demo)
    # ========================================================================
    
    # RGBD Sync - Syncs RGB + Depth images (CRITICAL!)
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': False,  # Exact sync for simulation
            'use_sim_time': True
        }],
        remappings=rtabmap_remappings
    )
    
    # RTAB-Map SLAM Mode (creates new map)
    rtabmap_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=rtabmap_remappings,
        arguments=['-d']  # Delete previous database on start
    )
    
    # RTAB-Map Localization Mode (uses existing map)
    rtabmap_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params,
            {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True'
            }
        ],
        remappings=rtabmap_remappings
    )
    
    # RTAB-Map Visualization
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_params],
        remappings=rtabmap_remappings
    )
    
    # ========================================================================
    # 3. OBSTACLE DETECTION (for Nav2 costmaps)
    # ========================================================================
    
    # Convert depth image to point cloud
    point_cloud_xyz = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='point_cloud_xyz',
        output='screen',
        parameters=[{
            'decimation': 2,
            'max_depth': 3.0,
            'voxel_size': 0.02,
            'use_sim_time': True
        }],
        remappings=[
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/camera_info'),
            ('cloud', '/camera/cloud')
        ]
    )
    
    # Segment floor from obstacles
    obstacles_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        parameters=[rtabmap_params],
        remappings=[
            ('cloud', '/camera/cloud'),
            ('obstacles', '/camera/obstacles'),
            ('ground', '/camera/ground')
        ]
    )
    
    # ========================================================================
    # 4. NAVIGATION (Nav2 for path planning)
    # ========================================================================
    
    nav2_params = os.path.join(pkg_grazen, 'config', 'grazen_params.yaml')
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params
        }.items()
    )
    
    # RViz (Nav2 visualization)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py'))
    )
    
    # ========================================================================
    # LAUNCH!
    # ========================================================================
    
    return LaunchDescription([
        # Force Waffle model (has camera)
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        
        # Arguments
        world_arg,
        localization_arg,
        
        # Gazebo (world-specific)
        gz_server_warehouse,
        gz_server_arena,
        gz_server_house,
        gz_client,
        
        # Robot
        robot_state_publisher,
        spawn_robot,
        
        # RTAB-Map SLAM
        rgbd_sync,
        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz,
        
        # Obstacle detection (for Nav2)
        point_cloud_xyz,
        obstacles_detection,
        
        # Navigation
        nav2_bringup,
        rviz,
    ])
