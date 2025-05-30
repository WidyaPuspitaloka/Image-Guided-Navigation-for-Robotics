import os
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Build the MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("six_dof_arm", package_name="six_dof_arm_moveit")
        .moveit_cpp(
            file_path=get_package_share_directory("my_robot_goal")
            + "/config/motion_planning.yaml"
        )
        .to_moveit_configs()
    )
    
    # Define the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory("six_dof_arm_moveit"),
        "config",
        "moveit.rviz"
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )
    
    # Static transform publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_base",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )
    
    # ROS2 Controllers
    ros2_controllers_path = os.path.join(
        get_package_share_directory("six_dof_arm_moveit"),
        "config",
        "ros2_controllers.yaml",
    )
        
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            moveit_config.robot_description,
        ],
        output="screen",
    )
    
    # Controller spawners - proper Node definitions
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_planning_group_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # Event handlers - spawn joint_state_broadcaster after ros2_control_node has started
    joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                LogInfo(msg="ros2_control_node started, spawning joint_state_broadcaster"),
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )
    
    # Event handlers - spawn controller after joint_state_broadcaster has started
    robot_controller_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                LogInfo(msg="joint_state_broadcaster started, spawning my_planning_group_controller"),
                TimerAction(
                    period=2.0,
                    actions=[robot_controller_spawner]
                )
            ]
        )
    )
    
    # MoveIt Python node
    moveit_py_node = Node(
        name="my_robot_goal",
        package="my_robot_goal",
        executable="my_robot_goal",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    # Event handler - start MoveIt Python node after controllers are spawned
    moveit_py_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_controller_spawner,
            on_start=[
                LogInfo(msg="Controllers ready, starting application node"),
                TimerAction(
                    period=2.0,
                    actions=[moveit_py_node]
                )
            ]
        )
    )
    
    # Move Group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )
    
    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )
      
    # OpenIGTLink Bridge node
    igtl_bridge_node = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        output="screen",
        parameters=[
            {"RIB_server_ip": "192.168.122.1"},  
            {"RIB_port": 18944},                 
            {"RIB_type": "client"},
            {"RIB_reconnect": True},
            {"RIB_reconnect_delay": 5000}
        ]
    )
    
    
    # Return the launch description with proper ordering and event handlers
    return LaunchDescription([
        # Core components
        robot_state_publisher,
        static_tf,
        ros2_control_node,
        
        # Controller spawners with event handlers
        joint_state_broadcaster_event,
        robot_controller_event,
        
        # MoveIt components with event handlers
        move_group_node,
        rviz_node,
        moveit_py_event,
        
        # OpenIGTLink Bridge
        igtl_bridge_node
    ])

