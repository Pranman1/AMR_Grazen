import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # --- PATHS ---
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # --- ARGUMENTS ---
    # Default to 'arena' (Turtlebot3 World) if not specified
    world_arg = DeclareLaunchArgument(
        'world', 
        default_value='arena', 
        description='arena or warehouse'
    )
    
    # --- LOGIC TO SELECT WORLD FILE ---
    # We construct the full path based on the 'world' argument
    # If world is 'warehouse', point to AWS. Else point to TB3 world.
    world_file = LaunchConfiguration('world')
    
    # Note: For simplicity in this script, we just assume if it's not the full path,
    # we default to the standard Turtlebot3 world. 
    # If you want to switch between AWS and TB3, use the standard gazebo launch include below.
    
    # --- CONFIGURATION ---
    use_sim_time = 'true'
    
    # 1. RTAB-MAP PARAMETERS (LiDAR Mode)
    rtabmap_parameters={
          'frame_id': 'base_footprint',
          'subscribe_depth': True,
          'subscribe_rgb': True,
          'subscribe_scan': True,
          'subscribe_odom_info': True, 
          'approx_sync': True,
          'use_sim_time': True,
          'qos_image': 2,
          'qos_scan': 2, 
          'qos_odom': 2,
          'queue_size': 20,

          # -- LIDAR FUSION --
          'Reg/Strategy': '1',       # 1=ICP (Trust LiDAR)
          'Reg/Force3DoF': 'true',   # 2D Mode
          'Grid/FromDepth': 'false', # Lidar Map
          'Grid/RangeMax': '3.5',    
          'Grid/RayTracing': 'true', 

          # -- 3D CLEANING --
          'Cloud/VoxelSize': '0.05', 
          'Cloud/MaxDepth': '2.5',   
          'Rtabmap/DetectionRate': '1',
    }

    # 2. TOPIC REMAPPINGS (Sim Waffle Pi -> RTAB-Map)
    remappings=[
          ('rgb/image',       '/camera/image_raw'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image',     '/camera/depth/image_raw'),
          ('scan',            '/scan'),
          ('odom',            '/odom')
    ]

    # --- GAZEBO SERVER ---
    # We use the standard gazebo launch. 
    # To switch worlds cleanly, we pass the world path directly.
    # For now, let's hardcode the logic to allow 'world:=arena' to work easily.
    
    tb3_world_path = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world')
    
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': tb3_world_path}.items()
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # --- ROBOT STATE PUBLISHER ---
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- SPAWN ROBOT (Forces Waffle Pi) ---
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '-2.0',
            'y_pose': '-0.5'
        }.items()
    )

    # --- RTAB-MAP NODE ---
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=remappings,
        arguments=['--delete_db_on_start'] 
    )

    # --- VISUALIZER ---
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=remappings
    )

    return LaunchDescription([
        # FORCE WAFFLE PI: This overrides your terminal environment
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        
        world_arg,
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_turtlebot,
        rtabmap_slam,
        rtabmap_viz
    ])