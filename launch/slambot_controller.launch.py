import launch
import launch_ros.actions

def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
                package='slambot_controller',
                executable='slambot_keyboard_controller',
                output="screen",
                emulate_tty=True
            ),
    ])



