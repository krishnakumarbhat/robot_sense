from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Camera width resolution'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Camera height resolution'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Camera frame rate'
    )

    return LaunchDescription([
        # Launch arguments
        width_arg,
        height_arg,
        fps_arg,
        
        # RealSense camera node
        Node(
            package='realsense_driver',
            executable='realsense_node',
            name='realsense_camera',
            output='screen',
            parameters=[{
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps')
            }]
        ),
        
        # Data recorder node
        Node(
            package='data_recorder',
            executable='recorder_node',
            name='data_recorder',
            output='screen',
            parameters=[{
                'topics_to_record': [
                    '/camera/color/image_raw',
                    '/camera/depth/image_rect_raw',
                    '/camera/pointcloud',
                    '/camera/imu',
                    '/camera/trigger'
                ],
                'bag_file_prefix': 'realsense_mapping'
            }]
        )
    ])