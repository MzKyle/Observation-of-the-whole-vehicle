#通过读取配置文件并启动相应的ROS 2节点，以便在机器人系统中使用串行驱动进行通信
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

#使用get_package_share_directory函数获取rm_serial_driver包的共享目录，并将其与config/serial_driver.yaml拼接成完整的配置文件路径
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_serial_driver'), 'config', 'serial_driver.yaml')

#创建一个Node对象，指定包名、可执行文件、命名空间、输出方式、是否模拟TTY以及参数文件
    rm_serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([rm_serial_driver_node])#返回启动描述
