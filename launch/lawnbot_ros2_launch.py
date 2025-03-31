# import launch
# import launch_ros.actions

# def generate_launch_description():
#     return launch.LaunchDescription([
#         launch_ros.actions.Node(
#             package='lawnbot_ros2',
#             executable='main',
#             name='main_logic_controller',),
#         launch_ros.actions.Node(
#             package='lawnbot_ros2',
#             executable='motor',
#             name='motor_controller'),
#         launch_ros.actions.Node(
#             package='lawnbot_ros2',
#             executable='path',
#             name='path_publisher'),
#         # launch_ros.actions.Node(
#         #     package='lawnbot_ros2',
#         #     executable='light_controller',
#         #     name='light_controller'),
#         # launch_ros.actions.Node(
#         #     package='lawnbot_ros2',
#         #     executable='pump_controller',
#         #     name='pump_controller'),
        
#   ])

import launch
import launch_ros.actions
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnShutdown

def generate_launch_description():
    # Define nodes
    main_logic_controller = launch_ros.actions.Node(
        package='lawnbot_ros2',
        executable='main',
        name='main_logic_controller'
    )

    motor_controller = launch_ros.actions.Node(
        package='lawnbot_ros2',
        executable='motor',
        name='motor_controller'
    )

    path_publisher = launch_ros.actions.Node(
        package='lawnbot_ros2',
        executable='path',
        name='path_publisher'
    )

    light_controller = launch_ros.actions.Node(
        package='lawnbot_ros2',
        executable='light',
        name='light_controller'
    )
    pump_controller = launch_ros.actions.Node(
        package='lawnbot_ros2',
        executable='pump',
        name='pump_controller'
    )
    
    picamera_controller = launch_ros.actions.Node(
        package='lawnbot_ros2',
        executable='picamera',
        name='picamera_controller'
    )
    object_detector = launch_ros.actions.Node(
        package='lawnbot_ros2',
        executable='object_detector',
        name='object_detector'
    )
    vid_streamer = launch_ros.actions.Node(
        package='lawnbot_ros2',
        executable='vid_streamer',
        name='vid_streamer'
    )

    # Handle Ctrl+C (SIGINT) for graceful shutdown
    handle_sigint = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(msg="Caught Ctrl+C, shutting down lawnbot gracefully!")]
        )
    )

    return launch.LaunchDescription([
        object_detector,
        picamera_controller,
        motor_controller,
        path_publisher,
        light_controller,
        pump_controller,
        main_logic_controller,
        vid_streamer,
        handle_sigint
    ])
