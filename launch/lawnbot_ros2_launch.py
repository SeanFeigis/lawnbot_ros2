import launch
import launch_ros.actions
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnShutdown

def generate_launch_description():
    # Lifecycle Nodes
    object_detector = launch_ros.actions.LifecycleNode(
        package='lawnbot_ros2',
        executable='object_detector',
        name='object_detector'
    )

    picamera_controller = launch_ros.actions.LifecycleNode(
        package='lawnbot_ros2',
        executable='picamera',
        name='picamera_controller'
    )

    main_logic_controller = launch_ros.actions.LifecycleNode(
        package='lawnbot_ros2',
        executable='main',
        name='main_logic_controller'
    )

    path_publisher = launch_ros.actions.LifecycleNode(
        package='lawnbot_ros2',
        executable='path',
        name='path_publisher'
    )

    motor_controller = launch_ros.actions.LifecycleNode(
        package='lawnbot_ros2',
        executable='motor',
        name='motor_controller'
    )

    # Regular Nodes (No Lifecycle)
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

    # Lifecycle Manager Node
    lifecycle_manager = launch_ros.actions.Node(
        package='lawnbot_ros2',
        executable='lifecycle_manager',
        name='lifecycle_manager'
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
        main_logic_controller,
        path_publisher,
        motor_controller,
        light_controller,
        pump_controller,
        main_logic_controller,
        vid_streamer,
        lifecycle_manager,
        handle_sigint
    ])
