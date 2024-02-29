#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import subprocess


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Timer her 1 saniyede bir çalışacak
        self.start_time = time.time()
        self.spawned_robot = False  # Bir kere robotu spawn etmek için flag
        self.get_logger().info('Robot control node has been started.')

    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Lineer hızı 1.0 m/s olarak ayarla
        twist_msg = Twist()
        twist_msg.linear.x = 1.0

        # Belirli bir süre sonra robotun hareketini durdur
        if elapsed_time >= 5.0:
            twist_msg.linear.x = 0.0

            # Bir kere robot spawn edildiyse ve spawn edilmemişse spawn et
            if not self.spawned_robot:
                self.spawn_robot_at_goal()
                self.spawned_robot = True

        self.publisher.publish(twist_msg)

    def spawn_robot_at_goal(self):
        # Robotun spawn edileceği konumu ve oryantasyonu belirtin
        x_position = 0.0
        y_position = 0.0
        z_position = 0.0
        yaw_angle = 0.0

        # Gazebo'nun spawn_entity.py scriptini kullanarak robotu spawn et
        command = [
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'my_robot',  # Bu, robotun adı olabilir, gazebo.launch dosyanızdaki isim ile uyumlu olmalıdır
            '-x', str(x_position),
            '-y', str(y_position),
            '-z', str(z_position),
            '-Y', str(yaw_angle),
            '-topic', '/gazebo/set_model_state',  # Konumu ve oryantasyonu belirtmek için kullanılacak topic
        ]

        # subprocess modülünü kullanarak shell komutunu çalıştır
        subprocess.run(command)


def main():
    rclpy.init()
    node = RobotControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
