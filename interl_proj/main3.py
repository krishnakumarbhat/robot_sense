# all_components.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments
    mode = DeclareLaunchArgument('mode', default_value='realtime',
                                 description='Operation mode: realtime, record, or playback')
    bag_file = DeclareLaunchArgument('bag_file', default_value='',
                                    description='Path to bag file for playback mode')
    
    # Launch components based on mode
    return LaunchDescription([
        mode,
        bag_file,
        
        # Camera node (only in realtime or record modes)
        Node(
            package='realsense_driver',
            executable='realsense_node',
            name='realsense_camera',
            parameters=[{'mode': LaunchConfiguration('mode')}],
            condition=IfCondition(PythonExpression(["'playback' != '", LaunchConfiguration('mode'), "'"])),
        ),
        
        # Recorder node (only in record mode)
        Node(
            package='data_recorder',
            executable='recorder_node',
            name='data_recorder',
            condition=IfCondition(PythonExpression(["'record' == '", LaunchConfiguration('mode'), "'"])),
        ),
        
        # Player node (only in playback mode)
        Node(
            package='data_player',
            executable='player_node',
            name='data_player',
            parameters=[{'bag_file': LaunchConfiguration('bag_file')}],
            condition=IfCondition(PythonExpression(["'playback' == '", LaunchConfiguration('mode'), "'"])),
        ),
        
        # ML node (always active)
        Node(
            package='ml_interface',
            executable='ml_node',
            name='ml_processor',
        ),
        
        # Controller node (always active)
        Node(
            package='actuator_controller',
            executable='controller_node',
            name='actuator_controller',
        ),
    ])