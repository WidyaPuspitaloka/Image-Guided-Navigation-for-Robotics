import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.substitutions import LaunchConfiguration

from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # Change this to your 6 DOF package name

    pkg_share = get_package_share_directory('six_dof_arm_description')

    

    # Get the path to the URDF file - update the filename

    urdf_file_path = os.path.join(pkg_share, 'urdf', 'six_dof_arm.urdf')

    

    # Check if the file exists - display warning if not

    if not os.path.exists(urdf_file_path):

        print(f"WARNING: URDF file not found at {urdf_file_path}")

        # Try to find any URDFs in the package

        for root, dirs, files in os.walk(pkg_share):

            for file in files:

                if file.endswith('.urdf'):

                    print(f"Found URDF file at: {os.path.join(root, file)}")

    

    # Load the URDF as a parameter

    with open(urdf_file_path, 'r') as file:

        robot_description = file.read()

    

    # Launch configuration variables

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_gui = LaunchConfiguration('use_gui')

    

    # Declare launch arguments

    declare_use_sim_time_arg = DeclareLaunchArgument(

        'use_sim_time',

        default_value='false',

        description='If true, use simulated clock'

    )

    

    declare_use_gui_arg = DeclareLaunchArgument(

        'use_gui',

        default_value='true',

        description='If true, show joint_state_publisher_gui'

    )

    

    # Start the robot_state_publisher node

    robot_state_publisher_node = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        name='robot_state_publisher',

        output='screen',

        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]

    )

    

    # Start the joint_state_publisher_gui node

    joint_state_publisher_gui_node = Node(

        package='joint_state_publisher_gui',

        executable='joint_state_publisher_gui',

        name='joint_state_publisher_gui',

        condition=IfCondition(use_gui),

        output='screen'

    )

    

    # If not using the GUI, use the regular joint_state_publisher

    joint_state_publisher_node = Node(

        package='joint_state_publisher',

        executable='joint_state_publisher',

        name='joint_state_publisher',

        condition=UnlessCondition(use_gui),

        output='screen'

    )

    

    # Define RViz configuration file

    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    

    # Start the rviz2 node with specific configuration

    rviz_node = Node(

        package='rviz2',

        executable='rviz2',

        name='rviz2',

        output='screen',

        arguments=['-d', default_rviz_config_path] if os.path.exists(default_rviz_config_path) else [],

        parameters=[{'use_sim_time': use_sim_time}]

    )

    

    # Return the LaunchDescription

    return LaunchDescription([

        declare_use_sim_time_arg,

        declare_use_gui_arg,

        robot_state_publisher_node,

        joint_state_publisher_gui_node,

        joint_state_publisher_node,

        rviz_node

    ])
