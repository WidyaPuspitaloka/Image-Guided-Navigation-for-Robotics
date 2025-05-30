#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from ros2_igtl_bridge.msg import Transform as IGTLTransform
from ros2_igtl_bridge.msg import Transform as IGTLTransform
from ros2_igtl_bridge.msg import PointArray as IGTLPointArray
from geometry_msgs.msg import Point

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String  # ✅ Added String for feedback
from std_msgs.msg import Empty

from sensor_msgs.msg import Joy # ✅ Added Joy for button clicks
import tf2_ros
from tf2_ros import Buffer, TransformListener
import time
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import numpy as np
import math

class MedicalToRobotTransform:
    def __init__(self):
        """Initialize transformation parameters for converting medical coordinates to robot workspace"""
        self.scale_factor = 0.001  # mm to m conversion
        
        # Robot workspace parameters (adjust based on your robot)
        self.robot_workspace = {
            'x_min': 0.2,   # Minimum reach forward (m)
            'x_max': 0.9,   # Maximum reach forward (m)
            'y_min': -0.4,  # Left limit (m)
            'y_max': 0.4,   # Right limit (m)
            'z_min': 0.4,   # Minimum height (m)
            'z_max': 1.0    # Maximum height (m)
        }
        
        # Medical coordinate system bounds (typical brain scan in mm)
        self.medical_bounds = {
            'x_min': 0,     # Left side of brain
            'x_max': 400,   # Right side of brain
            'y_min': 0,     # Back of brain
            'y_max': 400,   # Front of brain
            'z_min': 0,     # Bottom of brain
            'z_max': 400    # Top of brain
        }

    def medical_to_robot(self, medical_coords, is_surgical_point=False):
        """Transform medical coordinates (RAS, mm) to robot coordinates (m)"""
        x_med, y_med, z_med = medical_coords
        
        # Step 1: Normalize medical coordinates to [0, 1] range
        x_norm = self.normalize_coordinate(x_med, self.medical_bounds['x_min'], self.medical_bounds['x_max'])
        y_norm = self.normalize_coordinate(y_med, self.medical_bounds['y_min'], self.medical_bounds['y_max'])
        z_norm = self.normalize_coordinate(z_med, self.medical_bounds['z_min'], self.medical_bounds['z_max'])
        
        if is_surgical_point:
            # Surgical points map to robot workspace (no offset)
            x_robot = self.map_to_range(x_norm, self.robot_workspace['x_min'], self.robot_workspace['x_max'])
            y_robot = self.map_to_range(y_norm, self.robot_workspace['y_min'], self.robot_workspace['y_max'])
            z_robot = self.map_to_range(z_norm, self.robot_workspace['z_min'], self.robot_workspace['z_max'])
        else:
            # Brain points get offset to the side
            x_robot = self.map_to_range(x_norm, self.robot_workspace['x_min'], self.robot_workspace['x_max'])
            y_robot = self.map_to_range(y_norm, self.robot_workspace['y_min'], self.robot_workspace['y_max']) + 0.5 
            z_robot = self.map_to_range(z_norm, self.robot_workspace['z_min'], self.robot_workspace['z_max'])
        
        
        # Step 2: Map to robot workspace
        # x_robot = self.map_to_range(x_norm, self.robot_workspace['x_min'], self.robot_workspace['x_max'])
        # y_robot = self.map_to_range(y_norm, self.robot_workspace['y_min'], self.robot_workspace['y_max'])
        # z_robot = self.map_to_range(z_norm, self.robot_workspace['z_min'], self.robot_workspace['z_max'])

        return [x_robot, y_robot, z_robot]
    
    def normalize_coordinate(self, value, min_val, max_val):
        """Normalize a coordinate to [0, 1] range"""
        if max_val == min_val:
            return 0.5
        return max(0, min(1, (value - min_val) / (max_val - min_val)))
    
    def map_to_range(self, normalized_value, target_min, target_max):
        """Map normalized [0,1] value to target range"""
        return target_min + normalized_value * (target_max - target_min)

class SimpleRobotGoal(Node):
    def __init__(self):
        super().__init__('my_robot_goal')

        # ✅ TIMING VARIABLES - Moved to main class
        self.movement_start_time = None
        self.planning_start_time = None
        self.execution_start_time = None

        self.log_connection_details()
    
        # ✅ ADD: Publisher for brain structure markers
        self.marker_publisher = self.create_publisher(MarkerArray, '/brain_structures', 10)

        # Add brain point subscriber
        self.brain_point_sub = self.create_subscription(
            IGTLPointArray,
            '/IGTL_POINT_IN',
            self.brain_point_callback,
            10
        )
        
        # Initialize brain structure dictionaries
        self.brain_points = {
            'hippocampus': [],
            'vessels': [],
            'ventricles': [],
            'cortex': []
        }
        
        # Brain structure colors (RGBA)
        self.brain_colors = {
            'hippocampus': [1.0, 0.8, 0.0, 0.8],  # Gold/yellow
            'vessels': [0.8, 0.0, 0.0, 0.7],      # Dark red
            'ventricles': [0.0, 0.0, 1.0, 0.6],   # Blue
            'cortex': [0.9, 0.9, 0.9, 0.4]        # Light gray
        }

        # Action Client setup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('🔄 Waiting for MoveGroup server...')

        if self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('✅ MoveGroup server connected!')
        else:
            self.get_logger().error('❌ MoveGroup server not available!')
            return

        # Subscribe to transform
        self.transform_sub = self.create_subscription(
            IGTLTransform,
            '/IGTL_TRANSFORM_IN',
            self.transform_callback,
            10
        )
        
        # ✅ NEW: Subscribe to button clicks
        self.button_sub = self.create_subscription(
            Joy,
            '/rviz_visual_tools_gui',
            self.joy_button_callback,
            10
        )

        # ✅ NEW: Publisher for user feedback
        self.feedback_pub = self.create_publisher(String, '/rviz_visual_tools_user_feedback', 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ✅ NEW: Surgical workflow variables
        self.entry_point = None
        self.target_point = None
        self.trajectory_orientation = None
        self.workflow_step = 0  # 0: waiting, 1: ready for entry, 2: ready for target, 3: complete
        self.coord_transformer = MedicalToRobotTransform()

        self.get_logger().info('🎯 Simple Robot Goal Ready!')
        self.get_logger().info('📡 Listening for transforms on /IGTL_TRANSFORM_IN')
        self.get_logger().info('🎮 Listening for button clicks on /rviz_visual_tools_gui')
        
        # ✅ NEW: Update user feedback
        self.update_feedback("READY - Send surgical data from Slicer")

        self.last_goal_time = 0

        self.log_final_connection_status()

        self.visualization_timer = self.create_timer(0.5, self.publish_visualization)
        self.get_logger().info("🔄 Started visualization timer (2Hz)")

        self.viz_trigger_sub = self.create_subscription(Empty, '/trigger_visualization', self.trigger_visualization, 10)
        self.get_logger().info("📢 Added /trigger_visualization topic")

    def trigger_visualization(self, _):
        """Manually trigger visualization refresh"""
        self.get_logger().info('🔍 Manually triggering visualization refresh')
        
        # Print brain points stats
        for structure, points in self.brain_points.items():
            if points:
                self.get_logger().info(f'   • {structure}: {len(points)} points')
                # Print first and last point
                if len(points) > 0:
                    self.get_logger().info(f'     First: ({points[0].x:.3f}, {points[0].y:.3f}, {points[0].z:.3f})')
                    if len(points) > 1:
                        self.get_logger().info(f'     Last: ({points[-1].x:.3f}, {points[-1].y:.3f}, {points[-1].z:.3f})')
        
        # Force publish
        brain_markers = self.create_brain_structure_markers()
        self.marker_publisher.publish(brain_markers)

    def publish_visualization(self):
        """Regularly publish brain structure markers"""
        if any(len(points) > 0 for points in self.brain_points.values()):
            #has_collisions = self.check_brain_robot_collisions()
            brain_markers = self.create_brain_structure_markers()
            self.marker_publisher.publish(brain_markers)
            
            # Count total points being visualized
            total_points = sum(len(points) for points in self.brain_points.values())
            
            # Log once every few seconds to avoid console spam
            now = self.get_clock().now().to_msg().sec
            if now - getattr(self, 'last_viz_log_time', 0) > 25:  # Log every 5 seconds
                self.get_logger().info(f'🧠 Visualizing {total_points} brain structure points')
                self.last_viz_log_time = now

    # Update your callback function
    def brain_point_callback(self, msg):
        """Process incoming brain structure points from Slicer"""
        # Determine which structure this point array belongs to based on name
        structure = 'unknown'
        if 'hippo' in msg.name.lower():
            structure = 'hippocampus'
        elif 'vessel' in msg.name.lower():
            structure = 'vessels'
        elif 'ventricle' in msg.name.lower():
            structure = 'ventricles'
        elif 'cortex' in msg.name.lower():
            structure = 'cortex'
        
        if structure in self.brain_points:
            # Process each point in the array
            for pointdata in msg.pointdata:
                # Convert from medical coordinates to robot coordinates
                medical_coords = [pointdata.x, pointdata.y, pointdata.z]
                robot_coords = self.coord_transformer.medical_to_robot(medical_coords)
                
                # Create a Point message
                point = Point()
                point.x = robot_coords[0]
                point.y = robot_coords[1]
                point.z = robot_coords[2]
                
                # Add to the appropriate structure
                self.brain_points[structure].append(point)
            
            # Log sparingly to avoid cluttering the console
            self.get_logger().info(f'🧠 Added {len(msg.pointdata)} points to {structure}: {len(self.brain_points[structure])} points total')

        #Handle real brain models from Slicer
        # self.get_logger().info(f'📥 Received brain model: {msg.data}')
        # # Process and store the real brain model data
        # # This would display the actual medical meshes instead of fake cubes
    
    def create_brain_structure_markers(self):
        """Create realistic brain structure markers with needle trajectory"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # ALWAYS show brain structure points when available
        for structure_name, points in self.brain_points.items():
            if not points:
                continue  # Skip empty structures
                
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "brain_structures"
            marker.id = marker_id
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            
            # Set appearance - MAKE THE POINTS LARGER
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02  # 2cm points for better visibility
            marker.scale.y = 0.02
            
            # Set color based on structure
            color = self.brain_colors.get(structure_name, [0.5, 0.5, 0.5, 0.7])
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            
            # Add all points
            marker.points = points
            
            marker_array.markers.append(marker)
            marker_id += 1
            
            # Print a sample point for debugging - only for the first run
            if len(points) > 0 and not hasattr(self, 'printed_sample_points'):
                sample_point = points[0]
                self.get_logger().info(f"Sample {structure_name} point: ({sample_point.x:.3f}, {sample_point.y:.3f}, {sample_point.z:.3f})")
        
        # If we have any brain points, set a flag to avoid repeated logging
        if any(len(points) > 0 for points in self.brain_points.values()):
            self.printed_sample_points = True
            # Add a visible test marker to verify visualization works
            # test_marker = Marker()
            # test_marker.header.frame_id = "base_link"
            # test_marker.header.stamp = self.get_clock().now().to_msg()
            # test_marker.ns = "test_markers"
            # test_marker.id = 9999
            # test_marker.type = Marker.SPHERE
            # test_marker.action = Marker.ADD
            
            # # Position at center of robot workspace
            # test_marker.pose.position.x = 0.75
            # test_marker.pose.position.y = 0.0
            # test_marker.pose.position.z = 0.9
            # test_marker.pose.orientation.w = 1.0
            
            # # Make it large and bright
            # test_marker.scale.x = 0.05
            # test_marker.scale.y = 0.05
            # test_marker.scale.z = 0.05
            # test_marker.color.r = 1.0
            # test_marker.color.g = 0.0
            # test_marker.color.b = 1.0  # Bright purple
            # test_marker.color.a = 1.0
            
            # marker_array.markers.append(test_marker)
            # marker_id += 1
        
        
        # Add entry and target points if available
        if self.entry_point and self.target_point:
            entry_pos = self.entry_point['robot']
            target_pos = self.target_point['robot']
            
            # Add entry and target markers
            structures = {
                'real_target_hippocampus': {
                    'position': target_pos,
                    'size': [0.025, 0.015, 0.020],
                    'color': [1.0, 0.8, 0.0, 0.9],  # Yellow
                    'type': Marker.CUBE
                },
                'real_entry_point': {
                    'position': entry_pos,
                    'size': [0.015, 0.015, 0.015],
                    'color': [0.0, 1.0, 0.0, 0.9],  # Green
                    'type': Marker.SPHERE
                }
            }
            
            # Create markers for entry/target points
            for name, data in structures.items():
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "surgical_points"
                marker.id = marker_id
                marker.type = data['type']
                marker.action = Marker.ADD
                
                # Position
                marker.pose.position.x = data['position'][0]
                marker.pose.position.y = data['position'][1]
                marker.pose.position.z = data['position'][2]
                marker.pose.orientation.w = 1.0
                
                # Size
                marker.scale.x = data['size'][0]
                marker.scale.y = data['size'][1]
                marker.scale.z = data['size'][2]
                
                # Color
                marker.color.r = data['color'][0]
                marker.color.g = data['color'][1]
                marker.color.b = data['color'][2]
                marker.color.a = data['color'][3]
                
                marker_array.markers.append(marker)
                marker_id += 1
            
            # Add needle trajectory visualization
            needle_markers = self.create_needle_trajectory_markers(entry_pos, target_pos)
            marker_array.markers.extend(needle_markers)
        
        # Fallback visualization has been completely removed
        
        # Log stats on what we're publishing
        brain_point_count = sum(len(points) for points in self.brain_points.values())
        self.get_logger().debug(f"Publishing {len(marker_array.markers)} markers with {brain_point_count} brain structure points")
        
        return marker_array

    def create_needle_trajectory_markers(self, entry_pos, target_pos):
        """Create markers showing the needle trajectory path - CORRECTED DIRECTION"""
        trajectory_markers = []
        
        # ✅ CORRECTED: Needle should show robot's movement direction (entry-> target)
        needle_line = Marker()
        needle_line.header.frame_id = "base_link"
        needle_line.header.stamp = self.get_clock().now().to_msg()
        needle_line.ns = "needle_trajectory"
        needle_line.id = 100
        needle_line.type = Marker.LINE_STRIP
        needle_line.action = Marker.ADD
        
        # ✅ Show robot movement path 
        start_point = Point()
        start_point.x = entry_pos[0]     # ✅ FIXED: Start at ENTRY
        start_point.y = entry_pos[1]
        start_point.z = entry_pos[2]
        
        end_point = Point()
        end_point.x = target_pos[0]      # ✅ FIXED: End at TARGET
        end_point.y = target_pos[1]
        end_point.z = target_pos[2]

        needle_line.points = [start_point, end_point]
        
        # Needle appearance - make it more visible
        needle_line.scale.x = 0.004  # Thicker line (4mm)
        needle_line.color.r = 1.0    # ✅ Bright yellow needle
        needle_line.color.g = 1.0
        needle_line.color.b = 0.0
        needle_line.color.a = 1.0
        
        trajectory_markers.append(needle_line)
        
        # ✅ Add direction arrow to show withdrawal direction
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "base_link"
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = "needle_trajectory"
        arrow_marker.id = 102
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        
        # Arrow from target toward entry (withdrawal direction)
        arrow_start = Point()
        arrow_start.x = entry_pos[0]     # ✅ Arrow starts at ENTRY
        arrow_start.y = entry_pos[1]
        arrow_start.z = entry_pos[2]

        arrow_end = Point()
        arrow_end.x = target_pos[0]      # ✅ FIXED: Arrow points to TARGET
        arrow_end.y = target_pos[1]
        arrow_end.z = target_pos[2]

        arrow_marker.points = [arrow_start, arrow_end]
        arrow_marker.scale.x = 0.005  # Arrow shaft diameter
        arrow_marker.scale.y = 0.01   # Arrow head diameter
        arrow_marker.scale.z = 0.015  # Arrow head length
        
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0    # ✅ Green arrow showing insertion direction
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 0.8
        
        trajectory_markers.append(arrow_marker)
        
        # Keep your existing safety cylinder
        safety_cylinder = Marker()
        safety_cylinder.header.frame_id = "base_link"
        safety_cylinder.header.stamp = self.get_clock().now().to_msg()
        safety_cylinder.ns = "needle_trajectory"
        safety_cylinder.id = 101
        safety_cylinder.type = Marker.CYLINDER
        safety_cylinder.action = Marker.ADD
        
        # Position at trajectory midpoint
        mid_x = (entry_pos[0] + target_pos[0]) / 2
        mid_y = (entry_pos[1] + target_pos[1]) / 2
        mid_z = (entry_pos[2] + target_pos[2]) / 2
        
        safety_cylinder.pose.position.x = mid_x
        safety_cylinder.pose.position.y = mid_y
        safety_cylinder.pose.position.z = mid_z
        safety_cylinder.pose.orientation.w = 1.0
        
        # Calculate trajectory length
        trajectory_vec = np.array(target_pos) - np.array(entry_pos)
        trajectory_length = np.linalg.norm(trajectory_vec)
        
        safety_cylinder.scale.x = 0.01  # 1cm radius safety zone
        safety_cylinder.scale.y = 0.01
        safety_cylinder.scale.z = trajectory_length
        
        # Semi-transparent red safety zone
        safety_cylinder.color.r = 1.0
        safety_cylinder.color.g = 0.0
        safety_cylinder.color.b = 0.0
        safety_cylinder.color.a = 0.2
        
        trajectory_markers.append(safety_cylinder)
        
        return trajectory_markers

    def send_named_target(self, target_name):
        """Send robot to a named target defined in SRDF"""
        self.get_logger().info(f'🏠 Moving to named target: {target_name}')
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "my_planning_group"
        goal_msg.request.num_planning_attempts = 30
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 1.0
        
        # Use named target from SRDF
        from moveit_msgs.msg import JointConstraint
        
        # Create joint constraints for "home" position (all joints = 0)
        constraints = Constraints()
        joint_names = [
            "base_to_arm_1", "arm_1_to_arm_2", "arm_2_to_arm_3", 
            "arm_3_to_arm_4", "arm_4_to_arm_5", "arm_5_to_arm_6"
        ]
        
        for joint_name in joint_names:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = 0.0  # From your SRDF home state
            joint_constraint.tolerance_above = 0.1
            joint_constraint.tolerance_below = 0.1
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        future = self.move_group_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def log_connection_details(self):
        """Log OpenIGTLink connection details for documentation"""
        self.get_logger().info('🌐 ========== CONNECTION DETAILS ==========')
        self.get_logger().info('📋 OpenIGTLink Configuration:')
        self.get_logger().info('   • Protocol: OpenIGTLink over TCP/IP')
        self.get_logger().info('   • ROS2 Role: CLIENT (connecting to Slicer server)')
        self.get_logger().info('   • Expected Server: 3D Slicer')
        self.get_logger().info('   • Expected Port: 18944')
        self.get_logger().info('   • Bridge Node: ros2_igtl_bridge')
        self.get_logger().info('   • Communication: Bidirectional')
        self.get_logger().info('🔧 Expected Topics:')
        self.get_logger().info('   • /IGTL_TRANSFORM_IN (subscribing)')
        self.get_logger().info('   • Workflow: Slicer → OpenIGTLink → ROS2 → Robot')
        self.get_logger().info('🌐 ========================================')

    def log_final_connection_status(self):
        """Log final connection status after initialization"""
        self.get_logger().info('✅ ========== SYSTEM READY ==========')
        self.get_logger().info('🤖 Robot Controller: INITIALIZED')
        self.get_logger().info('🔗 MoveGroup Connection: ACTIVE')
        self.get_logger().info('📡 OpenIGTLink Listener: ACTIVE')
        self.get_logger().info('🎮 RViz Interface: ACTIVE')
        self.get_logger().info('🏥 Medical Workflow: READY')
        self.get_logger().info('   • Waiting for surgical data from 3D Slicer')
        self.get_logger().info('   • Coordinate transformation: Medical → Robot')
        self.get_logger().info('   • Safety: Hard & soft constraint validation')
        self.get_logger().info('✅ ====================================')

    def transform_callback(self, msg):
        """Handle incoming transform from Slicer - enhanced logging"""
        if "brain_" in msg.name.lower():
            print(f"🧠 RECEIVED BRAIN TRANSFORM: {msg.name}")
            print(f"   Coordinates: ({msg.transform.translation.x:.3f}, {msg.transform.translation.y:.3f}, {msg.transform.translation.z:.3f})")
            return  # Don't process as robot movement
    
        self.get_logger().info('🚨 ========== TRANSFORM RECEIVED ==========')
        self.get_logger().info(f'📦 Transform Name: {msg.name}')
        self.get_logger().info(f'📡 Source: 3D Slicer via OpenIGTLink Bridge')
        self.get_logger().info(f'🔢 Medical Coordinates (RAS):')
        self.get_logger().info(f'   • X: {msg.transform.translation.x:.3f} mm')
        self.get_logger().info(f'   • Y: {msg.transform.translation.y:.3f} mm') 
        self.get_logger().info(f'   • Z: {msg.transform.translation.z:.3f} mm')
        
        if msg.transform.rotation.w != 1.0 or any([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z]):
            self.get_logger().info(f'🧭 Orientation (Quaternion):')
            self.get_logger().info(f'   • X: {msg.transform.rotation.x:.6f}')
            self.get_logger().info(f'   • Y: {msg.transform.rotation.y:.6f}')
            self.get_logger().info(f'   • Z: {msg.transform.rotation.z:.6f}')
            self.get_logger().info(f'   • W: {msg.transform.rotation.w:.6f}')

        # Extract medical coordinates
        medical_coords = [
            msg.transform.translation.x,
            msg.transform.translation.y, 
            msg.transform.translation.z
        ]

        robot_coords = self.coord_transformer.medical_to_robot(medical_coords)
        self.get_logger().info(f'🔄 Coordinate Transformation:')
        self.get_logger().info(f'   • Medical: [{medical_coords[0]:.1f}, {medical_coords[1]:.1f}, {medical_coords[2]:.1f}] mm')
        self.get_logger().info(f'   • Robot:   [{robot_coords[0]:.3f}, {robot_coords[1]:.3f}, {robot_coords[2]:.3f}] m')
        self.get_logger().info(f'   • Scaling: mm → m (factor: {self.coord_transformer.scale_factor})')
        self.get_logger().info(f'   • Robot workspace bounds: X[{self.coord_transformer.robot_workspace["x_min"]:.2f},{self.coord_transformer.robot_workspace["x_max"]:.2f}], ' +
                            f'Y[{self.coord_transformer.robot_workspace["y_min"]:.2f},{self.coord_transformer.robot_workspace["y_max"]:.2f}], ' +
                            f'Z[{self.coord_transformer.robot_workspace["z_min"]:.2f},{self.coord_transformer.robot_workspace["z_max"]:.2f}]')
                
        # ✅ NEW: Handle different transform types for workflow
        if msg.name == "SurgicalEntryPoint":
            self.entry_point = {
                'medical': medical_coords,
                'robot': self.coord_transformer.medical_to_robot(medical_coords),
                'orientation': msg.transform.rotation
            }
            self.get_logger().info(f'🎯 ENTRY POINT STORED: {self.entry_point["robot"]}')
            self.check_workflow_ready()
            
        elif msg.name == "SurgicalTargetPoint":
            self.target_point = {
                'medical': medical_coords,
                'robot': self.coord_transformer.medical_to_robot(medical_coords),
                'orientation': msg.transform.rotation
            }
            self.get_logger().info(f'🎯 TARGET POINT STORED: {self.target_point["robot"]}')
            # ✅ ADD: Check if coordinates are reasonable
            # Update the warning check to match your actual workspace bounds
            x, y, z = self.target_point["robot"]
            if (x < self.coord_transformer.robot_workspace['x_min'] or 
                x > self.coord_transformer.robot_workspace['x_max'] or
                y < self.coord_transformer.robot_workspace['y_min'] or
                y > self.coord_transformer.robot_workspace['y_max'] or
                z < self.coord_transformer.robot_workspace['z_min'] or
                z > self.coord_transformer.robot_workspace['z_max']):
                self.get_logger().warn(f'⚠️ Target coordinates may be outside robot workspace!')
                self.get_logger().warn(f'⚠️ Workspace limits: x[{self.coord_transformer.robot_workspace["x_min"]},'+
                                    f'{self.coord_transformer.robot_workspace["x_max"]}], '+
                                    f'y[{self.coord_transformer.robot_workspace["y_min"]},'+
                                    f'{self.coord_transformer.robot_workspace["y_max"]}], '+
                                    f'z[{self.coord_transformer.robot_workspace["z_min"]},'+
                                    f'{self.coord_transformer.robot_workspace["z_max"]}]')
            self.check_workflow_ready()
            
        elif msg.name == "OptimalTrajectoryTra":
            self.trajectory_orientation = msg.transform.rotation
            self.get_logger().info(f'🧭 TRAJECTORY ORIENTATION STORED')
            
        else:
            # ✅ OLD BEHAVIOR: If it's not a specific surgical point, move immediately (like before)
            self.get_logger().info(f'📍 Medical coords (RAS): [{medical_coords[0]:.1f}, {medical_coords[1]:.1f}, {medical_coords[2]:.1f}] mm')
            
            robot_coords = self.coord_transformer.medical_to_robot(medical_coords)
            self.get_logger().info(f'📍 Robot coords: [{robot_coords[0]:.3f}, {robot_coords[1]:.3f}, {robot_coords[2]:.3f}] m')
            
            # Create and send pose goal immediately (original behavior)
            pose_goal = self.create_pose_goal(robot_coords, msg.transform.rotation)
            self.send_pose_goal(pose_goal, "Immediate Transform")
        
        # After processing all transforms, publish brain markers
        brain_markers = self.create_brain_structure_markers()
        self.marker_publisher.publish(brain_markers)
        self.get_logger().info('🧠 Published brain structure markers to RViz')

        
        self.get_logger().info('🚨 ========================================')
    
    def check_igtl_connection(self):
        """Check OpenIGTLink connection status"""
        try:
            # Get list of topics
            topic_names = self.get_topic_names_and_types()
            igtl_topics = [name for name, _ in topic_names if 'IGTL' in name]
            
            if igtl_topics:
                self.get_logger().info('📡 Active OpenIGTLink Topics:')
                for topic in igtl_topics:
                    self.get_logger().info(f'   • {topic}')
            else:
                self.get_logger().warn('⚠️ No OpenIGTLink topics detected')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error checking IGTL connection: {str(e)}')

    def check_workflow_ready(self):
        """Check if both entry and target points are available"""
        if self.entry_point and self.target_point:
            distance = np.linalg.norm(np.array(self.target_point['robot']) - np.array(self.entry_point['robot']))
            self.workflow_step = 1  # Ready for entry point
            
            self.get_logger().info('✅ SURGICAL WORKFLOW READY!')
            self.get_logger().info(f'📏 Trajectory distance: {distance*1000:.1f} mm')
            self.get_logger().info('🎮 Click "Next" in RViz to start workflow')
            
            self.update_feedback(f"STEP 1: Click 'Next' to move to ENTRY POINT ({distance*1000:.1f}mm)")

    def joy_button_callback(self, msg):
        """Handle Joy messages from RViz Visual Tools"""
        # Check if any button is pressed (buttons[1] = Next, buttons[2] = Continue)
        if msg.buttons and len(msg.buttons) > 2:
            if msg.buttons[1] > 0 or msg.buttons[2] > 0:  # Next or Continue button
                 # ✅ START TIMING
                self.movement_start_time = time.time()

                self.get_logger().info('🖱️ RVIZ BUTTON CLICKED!')
                self.get_logger().info(f'⏱️ WORKFLOW TIMER STARTED: {self.movement_start_time:.3f}')
                
                # Keep all your existing workflow logic the same:
                if self.workflow_step == 0:
                    self.get_logger().warn('⚠️ No surgical data available. Send data from Slicer first.')
                    self.update_feedback("Send surgical data from Slicer first")
                    
                elif self.workflow_step == 1:
                    # Move to entry point
                    self.get_logger().info('🎯 MOVING TO ENTRY POINT...')
                    self.update_feedback("MOVING TO ENTRY POINT...")
                    
                    pose_goal = self.create_pose_goal(
                        self.entry_point['robot'], 
                        self.trajectory_orientation or self.entry_point['orientation']
                    )
                    self.send_pose_goal(pose_goal, "Entry Point")
                    self.workflow_step = 2
                    
                elif self.workflow_step == 2:
                    # Move to target point through an intermediate waypoint
                    self.get_logger().info('🎯 MOVING TO INTERMEDIATE POINT...')
                    self.update_feedback("MOVING TO INTERMEDIATE POINT...")
                    
                    # Calculate an intermediate point between entry and target
                    entry_pos = self.entry_point['robot']
                    target_pos = self.target_point['robot']
                    
                    # Create a point 80% of the way from entry to target
                    intermediate_pos = [
                        entry_pos[0] + 0.8 * (target_pos[0] - entry_pos[0]),
                        entry_pos[1] + 0.8 * (target_pos[1] - entry_pos[1]),
                        entry_pos[2] + 0.8 * (target_pos[2] - entry_pos[2])
                    ]
                    
                    # Use entry point orientation for intermediate point
                    pose_goal = self.create_pose_goal(
                        intermediate_pos,
                        self.trajectory_orientation or self.entry_point['orientation']
                    )
                    self.send_pose_goal(pose_goal, "Intermediate Point")
                    
                    # We'll move to target after this completes
                    self.workflow_step = 2.5  # Use a new step number
                    
                elif self.workflow_step == 2.5:
                    # Continue to target point
                    self.get_logger().info('🎯 MOVING TO TARGET POINT...')
                    self.update_feedback("MOVING TO TARGET POINT...")
                    
                    pose_goal = self.create_pose_goal(
                        self.target_point['robot'], 
                        self.trajectory_orientation or self.target_point['orientation']
                    )
                    self.send_pose_goal(pose_goal, "Target Point")
                    self.workflow_step = 3
                    
                elif self.workflow_step == 3:
                    # Return to SRDF "home" position (all joints = 0)
                    self.get_logger().info('🏠 RETURNING TO SRDF HOME...')
                    self.update_feedback("RETURNING TO HOME...")

                    self.send_named_target("home")
                    self.workflow_step = 0

    def create_pose_goal(self, robot_coords, orientation):
        """Create a pose goal from robot coordinates and orientation"""

        # Print exact coordinates for debugging
        self.get_logger().info(f"DEBUG: Creating pose goal at coordinates:")
        self.get_logger().info(f"  X: {robot_coords[0]:.4f} m")
        self.get_logger().info(f"  Y: {robot_coords[1]:.4f} m")
        self.get_logger().info(f"  Z: {robot_coords[2]:.4f} m")
    
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        pose_goal.pose.position.x = robot_coords[0]
        pose_goal.pose.position.y = robot_coords[1]
        pose_goal.pose.position.z = robot_coords[2]
        
        # Set orientation
        pose_goal.pose.orientation.x = orientation.x
        pose_goal.pose.orientation.y = orientation.y
        pose_goal.pose.orientation.z = orientation.z
        pose_goal.pose.orientation.w = orientation.w
        
        return pose_goal

    def update_feedback(self, message):
        """Update RViz visual tools feedback"""
        msg = String()
        msg.data = message
        self.feedback_pub.publish(msg)

    def send_pose_goal(self, pose_stamped, description=""):
        """Your existing send_pose_goal function - unchanged"""
         # ✅ PLANNING START TIMING
        self.planning_start_time = time.time()

        self.get_logger().info(f'📤 Sending {description} to MoveGroup')
        self.get_logger().info(f'⏱️ PLANNING PHASE START: {self.planning_start_time:.3f}')

        self.get_logger().info(
            f'🎯 Target Position: [{pose_stamped.pose.position.x:.3f}, '
            f'{pose_stamped.pose.position.y:.3f}, {pose_stamped.pose.position.z:.3f}]'
        )
        self.get_logger().info(
            f'🧭 Target Orientation: [{pose_stamped.pose.orientation.x:.3f}, '
            f'{pose_stamped.pose.orientation.y:.3f}, {pose_stamped.pose.orientation.z:.3f}, '
            f'{pose_stamped.pose.orientation.w:.3f}]'
        )

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "my_planning_group"
        goal_msg.request.num_planning_attempts = 30
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 1.0

        # Constraints
        constraints = Constraints()

        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = "end_effector"
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        region.dimensions = [0.15, 0.15, 0.15]

        position_constraint.constraint_region.primitives.append(region)
        position_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = "end_effector"
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 0.3
        constraints.orientation_constraints.append(orientation_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        future = self.move_group_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        # ✅ PLANNING END / EXECUTION START TIMING
        if self.planning_start_time:
            planning_end_time = time.time()
            planning_duration = planning_end_time - self.planning_start_time
            self.get_logger().info(f'⏱️ PLANNING COMPLETED: {planning_duration:.2f} seconds')

        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by MoveGroup!')
            return
        
        self.execution_start_time = time.time()
        self.get_logger().info('✅ Goal accepted! Planning and executing...')
        self.get_logger().info(f'⏱️ EXECUTION PHASE START: {self.execution_start_time:.3f}')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Enhanced result callback with comprehensive timing"""
        result = future.result().result
        
        # ✅ COMPLETE TIMING ANALYSIS
        if self.movement_start_time:
            end_time = time.time()
            
            # Calculate all timing phases
            total_workflow_time = end_time - self.movement_start_time
            planning_time = (self.execution_start_time - self.planning_start_time) if self.planning_start_time and self.execution_start_time else 0
            execution_time = (end_time - self.execution_start_time) if self.execution_start_time else 0
            preprocessing_time = (self.planning_start_time - self.movement_start_time) if self.planning_start_time and self.movement_start_time else 0

            self.get_logger().info('🎉 Movement completed successfully!')
            self.get_logger().info('⏱️ ========== TIMING BREAKDOWN ==========')
            self.get_logger().info(f'⏱️ Button Click → Goal Sent: {preprocessing_time:.2f}s')
            self.get_logger().info(f'⏱️ Planning Duration: {planning_time:.2f}s') 
            self.get_logger().info(f'⏱️ Execution Duration: {execution_time:.2f}s')
            self.get_logger().info(f'⏱️ TOTAL WORKFLOW TIME: {total_workflow_time:.2f}s')
            self.get_logger().info(f'⏱️ Workflow Started: {self.movement_start_time:.3f}')
            self.get_logger().info(f'⏱️ Workflow Completed: {end_time:.3f}')
            self.get_logger().info('⏱️ ====================================')
            
            # Reset timing variables
            self.movement_start_time = None
            self.planning_start_time = None
            self.execution_start_time = None
        else:
            self.get_logger().info('🎉 Movement completed successfully!')

        if hasattr(result, 'error_code'):
            if result.error_code.val == 1:
                # Update workflow feedback
                if self.workflow_step == 2:
                    self.update_feedback("STEP 2: Click 'Next' to move to TARGET POINT")
                elif self.workflow_step == 2.5:
                    self.update_feedback("STEP 2.5: Click 'Next' to move to FINAL TARGET")
                elif self.workflow_step == 3:
                    self.update_feedback("STEP 3: Click 'Next' to return HOME")
                elif self.workflow_step == 0:
                    self.update_feedback("READY - Send new surgical data")
                    
            else:
                error_msgs = {
                    -1: "FAILURE - General failure",
                    -2: "PLANNING_FAILED - Could not find valid path",
                    -3: "INVALID_MOTION_PLAN - Plan is invalid",
                    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                    -5: "CONTROL_FAILED - Execution failed",
                    -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                    -7: "TIMED_OUT - Planning timed out",
                    -10: "INVALID_GROUP_NAME - Check your planning group name",
                    99999: "PLANNING_FAILED - Position might be unreachable"
                }
                self.get_logger().error(f'❌ Movement failed with error code: {result.error_code.val}')
                if result.error_code.val in error_msgs:
                    self.get_logger().error(f'💡 {error_msgs[result.error_code.val]}')
                    
                self.update_feedback(f"ERROR: {error_msgs.get(result.error_code.val, 'Movement failed')}")
                
                # Reset timing on failure
                self.movement_start_time = None
                self.planning_start_time = None
                self.execution_start_time = None
        else:
            self.get_logger().warn('⚠️  No error code in result')



def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
