import launch
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

CONFIG = '/home/nvidia/game_v2/src/serical_device_ros2/config/auto_aim_params.yaml'


def generate_launch_description():
    container = ComposableNodeContainer(
        name='rm_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
    )
    nodes = LoadComposableNodes(
        target_container='rm_container',
        composable_node_descriptions=[
            ComposableNode(
                package='serical_device_ros2',
                plugin='rm_auto_aim::AutoAimNode',
                name='auto_aim',
                parameters=[{
                    'auto_aim_enable': True,    # master gate; actual engage requires mode==33 (require_mode33)
                    'allow_fire': False,        # safety: master fire gate\n                    'aim_mode': 'direct',        # direct pixel-servo (fallback); 'awakening' later
                    'require_mode33': True,   # engage only when /Vision_data.mode==33 (self-aim key)
                    'config_path': CONFIG,
                }],
            ),
            ComposableNode(
                package='serical_device_ros2',
                plugin='rm_auto_aim::RobotCtrlSub',
                name='robot_ctrl',
                parameters=[{'serial_port': '/dev/robomaster', 'serial_baud': 921600}],
            ),
            ComposableNode(
                package='serical_device_ros2',
                plugin='rm_auto_aim::VisionPub',
                name='vision_pub',
                parameters=[{'serial_port': '/dev/robomaster', 'serial_baud': 921600}],
            ),
        ],
    )
    return launch.LaunchDescription([container, nodes])