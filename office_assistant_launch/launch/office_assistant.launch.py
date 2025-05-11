from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare top-level arguments (TIAGo core + custom modules)
    declare_args = [
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('navigation', default_value='True'),
        DeclareLaunchArgument('moveit', default_value='True'),
        DeclareLaunchArgument('is_public_sim', default_value='True'),
        DeclareLaunchArgument('world_name', default_value='pal_office'),
        DeclareLaunchArgument('use_yolo', default_value='False'),
        DeclareLaunchArgument('use_voice', default_value='False'),
        DeclareLaunchArgument('use_task_manager', default_value='False'),
    ]

    # TIAGo simulation base launch
    tiago_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tiago_gazebo'),
                'launch',
                'tiago_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'navigation': LaunchConfiguration('navigation'),
            'moveit': LaunchConfiguration('moveit'),
            'is_public_sim': LaunchConfiguration('is_public_sim'),
            'world_name': LaunchConfiguration('world_name')
        }.items()
    )

    # Optional nodes
    yolo_node = Node(
        package='perception_yolov8',
        executable='yolo_node',
        name='yolo_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_yolo'))
    )

    voice_node = Node(
        package='voice_command_interface',
        executable='voice_listener',
        name='voice_interface',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_voice'))
    )

    task_node = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_task_manager'))
    )

    return LaunchDescription(declare_args + [
        tiago_gazebo_launch,
        yolo_node,
        voice_node,
        task_node
    ])

