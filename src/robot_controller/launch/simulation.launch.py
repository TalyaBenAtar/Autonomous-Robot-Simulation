import os
import xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_dir = get_package_share_directory('robot_controller')

    # Generate the robot description from Xacro
    xacro_file = os.path.join(
        package_dir, 'description', 'robot.urdf.xacro'
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    # Publish transformations between robot links
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn the robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-name', 'autonomous_robot',
            '-string', robot_description,
            '-z', '0.20'
        ],
        output='screen'
    )

    # Connect ROS 2 and Gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )

    # Autonomous movement controller
    controller = Node(
        package='robot_controller',
        executable='movement_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Start the controller after spawning finishes successfully
    start_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=lambda event, context:
                [controller] if event.returncode == 0 else []
        )
    )

    return LaunchDescription([
        gazebo,
        state_publisher,
        bridge,
        start_controller,
        spawn_robot
    ])