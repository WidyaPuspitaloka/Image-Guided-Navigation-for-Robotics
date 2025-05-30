#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2_igtl_bridge.msg import Transform as IGTLTransform
from geometry_msgs.msg import Transform
import time

class TestTransformSender(Node):
    def __init__(self):
        super().__init__('test_transform_sender')
        
        # Create publisher for transforms
        self.transform_pub = self.create_publisher(
            IGTLTransform,
            '/IGTL_TRANSFORM_IN',
            10
        )
        
        # Wait a bit for connections
        self.timer = self.create_timer(3.0, self.send_test_transform)
        
        self.get_logger().info('🧪 Test Transform Sender Ready!')
    
    def send_test_transform(self):
        """Send a test transform"""
        self.get_logger().info('📤 Sending test transform...')
        
        # Create transform message
        msg = IGTLTransform()
        msg.name = "TestTransform"
        
        # Set position (in mm, will be converted to m in receiver)
        msg.transform.translation.x = 500.0  # 0.5m
        msg.transform.translation.y = 0.0    # 0.0m  
        msg.transform.translation.z = 800.0  # 0.8m
        
        # Set orientation (identity quaternion)
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 1.0
        
        # Publish the transform
        self.transform_pub.publish(msg)
        self.get_logger().info('✅ Test transform sent!')
        
        # Stop the timer so it only sends once
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TestTransformSender()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
