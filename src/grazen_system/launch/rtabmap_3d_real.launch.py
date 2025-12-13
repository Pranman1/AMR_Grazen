# ==============================================================================
# RTAB-Map 3D SLAM for Real TurtleBot3 with OAK-D Camera
# ==============================================================================
# Fuses wheel odometry + LDS-02 LiDAR + OAK-D RGB-D camera
#
# PREREQUISITES (run on robot first):
#   Terminal 1: ros2 launch turtlebot3_bringup robot.launch.py
#   Terminal 2: ros2 launch depthai_ros_driver camera.launch.py
#
# THEN (on robot or laptop with ROS_DOMAIN_ID set):
#   ros2 launch grazen_system rtabmap_3d_real.launch.py
#
# TELEOP (separate terminal):
#   ros2 run turtlebot3_teleop teleop_keyboard
# ==============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- ARGUMENTS ---
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='true = localization mode (use existing map), false = SLAM mode'
    )
    viz_arg = DeclareLaunchArgument(
        'viz',
        default_value='true',
        description='Launch rtabmap_viz visualization'
    )
    
    localization = LaunchConfiguration('localization')
    viz = LaunchConfiguration('viz')
    
    # --- RTAB-MAP PARAMETERS ---
    rtabmap_params = {
        'frame_id': 'base_footprint',
        'use_sim_time': False,          # REAL robot!
        'subscribe_rgbd': True,         # Use synced RGBD
        'subscribe_scan': True,         # Fuse LiDAR
        'use_action_for_goal': True,
        # Registration
        'Reg/Strategy': '1',            # ICP (trust LiDAR)
        'Reg/Force3DoF': 'true',        # 2D robot
        'RGBD/NeighborLinkRefining': 'True',
        # Depth constraints (match LiDAR range)
        'Rtabmap/MaxDepth': '3.5',      # Ignore depth beyond 3.5m
        'Rtabmap/MinDepth': '0.2',      # Ignore depth closer than 20cm
        'Vis/MaxDepth': '3.5',          # Visual features max depth
        'Vis/MinDepth': '0.2',          # Visual features min depth
        # Grid Generation
        'Grid/RayTracing': 'true',
        'Grid/3D': 'false',             # 2D occupancy for Nav2
        'Grid/RangeMax': '3.5',
        'Grid/RangeMin': '0.2',
        'Grid/Sensor': '2',             # BOTH camera + LiDAR
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '0.4',
        # Noise filtering
        'Grid/NoiseFilteringRadius': '0.1',
        'Grid/NoiseFilteringMinNeighbors': '5',
        # Optimizer
        'Optimizer/GravitySigma': '0',
    }
    
    # --- OAK-D TOPIC REMAPPINGS ---
    # OAK-D publishes to /oak/* topics
    oakd_remappings = [
        ('rgb/image', '/oak/rgb/image_raw'),
        ('rgb/camera_info', '/oak/rgb/camera_info'),
        ('depth/image', '/oak/stereo/image_raw'),
    ]
    
    # --- CAMERA TF (OAK-D position on robot) ---
    # Adjust x, y, z, yaw, pitch, roll based on your mount
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oakd_tf',
        arguments=[
            '--x', '0.1',       # 10cm forward from base
            '--y', '0',
            '--z', '0.2',       # 20cm up
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'oak-d-base-frame'
        ]
    )
    
    # Optical frame transform (camera convention)
    optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oakd_optical_tf',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '-1.5708',   # -90 degrees
            '--pitch', '0',
            '--roll', '-1.5708',  # -90 degrees
            '--frame-id', 'oak-d-base-frame',
            '--child-frame-id', 'oak_rgb_camera_optical_frame'
        ]
    )
    
    # --- RGBD SYNC (syncs RGB + depth) ---
    # OAK-D has ~30-100ms offset between RGB and depth
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,              # Approximate sync for real sensors
            'approx_sync_max_interval': 0.05, # 50ms max (tighter sync, less drift)
            'use_sim_time': False,
            'qos': 2,                         # BEST_EFFORT for sensor data
        }],
        remappings=oakd_remappings
    )
    
    # --- RTAB-MAP SLAM MODE ---
    rtabmap_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params, {'qos_image': 2, 'qos_scan': 2, 'qos_odom': 2}],
        remappings=oakd_remappings,
        arguments=['-d']  # Delete previous database
    )
    
    # --- RTAB-MAP LOCALIZATION MODE ---
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
                'Mem/InitWMWithAllNodes': 'True',
                'qos_image': 2,
                'qos_scan': 2,
                'qos_odom': 2,
            }
        ],
        remappings=oakd_remappings
    )
    
    # --- RTAB-MAP VISUALIZATION ---
    rtabmap_viz = Node(
        condition=IfCondition(viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_params, {'qos_image': 2, 'qos_scan': 2, 'qos_odom': 2}],
        remappings=oakd_remappings
    )
    
    # --- OBSTACLE DETECTION (for Nav2) ---
    point_cloud_xyz = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='point_cloud_xyz',
        output='screen',
        parameters=[{
            'decimation': 2,        # Match sim (was 4, too aggressive)
            'max_depth': 3.0,
            'voxel_size': 0.02,     # Match sim (was 0.05, too coarse)
            'use_sim_time': False,
            'qos': 2,
        }],
        remappings=[
            ('depth/image', '/oak/stereo/image_raw'),
            ('depth/camera_info', '/oak/rgb/camera_info'),
            ('cloud', '/oak/cloud')
        ]
    )
    
    obstacles_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        parameters=[rtabmap_params],
        remappings=[
            ('cloud', '/oak/cloud'),
            ('obstacles', '/oak/obstacles'),
            ('ground', '/oak/ground')
        ]
    )
    
    # ========================================================================
    # LAUNCH!
    # ========================================================================
    
    return LaunchDescription([
        # Arguments
        localization_arg,
        viz_arg,
        
        # Camera TF
        camera_tf,
        optical_tf,
        
        # RTAB-Map
        rgbd_sync,
        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz,
        
        # Obstacle detection
        point_cloud_xyz,
        obstacles_detection,
    ])

