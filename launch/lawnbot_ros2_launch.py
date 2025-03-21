import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='lawnbot_ros2',
            executable='main',
            name='main_logic_controller',),
        launch_ros.actions.Node(
            package='lawnbot_ros2',
            executable='motor',
            name='motor_controller'),
        launch_ros.actions.Node(
            package='lawnbot_ros2',
            executable='path',
            name='path_publisher'),
        launch_ros.actions.Node(
            package='lawnbot_ros2',
            executable='light_controller',
            name='light_controller'),
        launch_ros.actions.Node(
            package='lawnbot_ros2',
            executable='pump_controller',
            name='pump_controller'),
        
  ])