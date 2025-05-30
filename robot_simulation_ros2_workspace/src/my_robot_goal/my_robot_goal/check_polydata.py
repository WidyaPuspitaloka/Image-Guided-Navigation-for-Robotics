#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

class PolydataChecker(Node):
    def __init__(self):
        super().__init__('polydata_checker')
        
        # Create subscribers for different message types to see what's available
        try:
            # Try to import and subscribe to polydata message
            from ros2_igtl_bridge.msg import PolyData
            self.polydata_sub = self.create_subscription(
                PolyData,
                '/IGTL_POLYDATA_IN',  # Try input channel
                self.polydata_callback,
                10)
            
            # Also subscribe to output channel
            self.polydata_out_sub = self.create_subscription(
                PolyData,
                '/IGTL_POLYDATA_OUT',  # Try output channel
                self.polydata_callback,
                10)
            
            self.get_logger().info("✅ Successfully imported PolyData message type")
        except ImportError:
            self.get_logger().error("❌ Could not import PolyData message type!")
        
        # Also subscribe to Transform messages which we know work
        try:
            from ros2_igtl_bridge.msg import Transform
            self.transform_sub = self.create_subscription(
                Transform,
                '/IGTL_TRANSFORM_IN',
                self.transform_callback,
                10)
            self.get_logger().info("✅ Successfully imported Transform message type")
        except ImportError:
            self.get_logger().error("❌ Could not import Transform message type!")
        
        # Print available message types
        self.get_logger().info("Checking for available IGTL message types...")
        try:
            import ros2_igtl_bridge.msg as igtl_msgs
            msg_types = [name for name in dir(igtl_msgs) if not name.startswith('_')]
            self.get_logger().info(f"📋 Available message types: {msg_types}")
        except:
            self.get_logger().error("❌ Could not inspect available message types!")

        self.timer = self.create_timer(5.0, self.status_callback)
        self.polydata_count = 0
        self.transform_count = 0
        self.get_logger().info("🔍 Polydata checker running - waiting for messages...")

    def polydata_callback(self, msg):
        self.polydata_count += 1
        self.get_logger().info(f"🚨 Received PolyData message #{self.polydata_count}")
        self.get_logger().info(f"  Name: {msg.name}")
        self.get_logger().info(f"  Points: {len(msg.points)}")
        self.get_logger().info(f"  Polygons: {len(msg.polygons)}")

    def transform_callback(self, msg):
        self.transform_count += 1
        self.get_logger().info(f"🔄 Received Transform message #{self.transform_count}")

    def status_callback(self):
        self.get_logger().info(f"Status Update: Received {self.polydata_count} PolyData messages and {self.transform_count} Transform messages")
        
        # List all available topics
        try:
            from rclpy.topic_endpoint_info import TopicEndpointTypeEnum
            topic_names_and_types = self.get_topic_names_and_types()
            
            igtl_topics = [name for name, types in topic_names_and_types 
                          if any('igtl' in t.lower() for t in types)]
            
            if igtl_topics:
                self.get_logger().info(f"📡 Active IGTL topics: {igtl_topics}")
            else:
                self.get_logger().info("⚠️ No active IGTL topics detected")
                
        except Exception as e:
            self.get_logger().error(f"Error checking topics: {str(e)}")

def main():
    rclpy.init()
    node = PolydataChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
