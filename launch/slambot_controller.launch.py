import launch
import launch_ros.actions

def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
            ),
        launch_ros.actions.Node(
                package='slambot_controller',
                executable='slambot_keyboard_controller',
            ),
    ])



