import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

class RobotListenerNode(Node):
    def __init__(self):
        super().__init__("robot_control")
        self.get_logger().info('ROS2 Robot Control Node Started')

        self.tf_sub = self.create_subscription(TransformStamped, '/tf', self.tf_callback, 1)

    def tf_callback(self, msg: TransformStamped):
        for transform in msg.transforms:
            if transform.header.frame_id == 'robot_base_frame' and transform.child_frame_id == 'odom':  
                try:
                    translation = transform.transform.translation
                    x = translation.x
                    y = translation.y
                    z = translation.z
                    self.get_logger().debug("x: %.2f, y: %.2f, z: %.2f" % (x, y, z))
                except (AttributeError, TypeError):
                    self.get_logger().warn("TF mesajını işlerken hata oluştu.") 
        

def main():
    rclpy.init()
    node = RobotListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
