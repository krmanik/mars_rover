from http.server import executable
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnExecutionComplete
import os
from os import environ

from ament_index_python.packages import get_package_share_directory

import xacro



# . ../spaceros_ws/install/setup.bash && . ../depends_ws/install/setup.bash
# rm -rf build install log && colcon build && . install/setup.bash

def generate_launch_description():

    mars_rover_demos_path = get_package_share_directory('mars_rover')
    mars_rover_models_path = get_package_share_directory('simulation')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_RESOURCE_PATH':
           ':'.join([environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''), mars_rover_demos_path])}
    
    urdf_model_path = os.path.join(mars_rover_models_path, 'models', 'curiosity_path',
        'urdf', 'curiosity_mars_rover.xacro.urdf')
    mars_world_model = os.path.join(mars_rover_demos_path, 'worlds', 'mars_curiosity.world')

    doc = xacro.process_file(urdf_model_path)
    robot_description = {'robot_description': doc.toxml()}


    arm_node = Node(
        package="mars_rover",
        executable="move_arm",
        output='screen'
    )

    mast_node = Node(
        package="mars_rover",
        executable="move_mast",
        output='screen'
    )

    wheel_node = Node(
        package="mars_rover",
        executable="move_wheel",
        output='screen'
    )

    run_node = Node(
        package="mars_rover",
        executable="run_demo",
        output='screen'
    )

    start_world = ExecuteProcess(
        cmd=['ign gazebo --render-engine ogre', mars_world_model, '-r'],
        output='screen',
        additional_env=env,
        shell=True
    )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description])

    spawn = Node(
        package='ros_ign_gazebo', executable='create',
        arguments=[
            '-name', 'curiosity_mars_rover',
            '-topic', robot_description,
        ],
        output='screen'
    )


    ## Control Components

    component_state_msg = '{name: "IgnitionSystem", target_state: {id: 3, label: ""}}'

    ## a hack to resolve current bug
    set_hardware_interface_active = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
            'controller_manager/set_hardware_component_state',
            'controller_manager_msgs/srv/SetHardwareComponentState',
            component_state_msg]
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_joint_trajectory_controller'],
        output='screen'
    )

    load_mast_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mast_joint_trajectory_controller'],
        output='screen'
    )

    load_wheel_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'wheel_velocity_controller'],
        output='screen'
    )

    load_steer_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'steer_position_controller'],
        output='screen'
    )

    load_suspension_joint_traj_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'wheel_tree_position_controller'],
        output='screen'
    )

    load_camera_bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'],
        output='screen'
    )

    # chain of node for subscribing, processing, and analyzing image from raw image
    image_subscription_node = Node(
        package="mars_rover",
        executable="image_subscriber",
        output='screen'
    )

    image_process_node = Node(
        package="mars_rover",
        executable="image_process",
        output='screen'
    )

    image_analyze_node = Node(
        package="mars_rover",
        executable="image_analyze",
        output='screen'
    )


    return LaunchDescription([
        start_world,
        robot_state_publisher,
        spawn,
        arm_node,
        mast_node,
        wheel_node,
        run_node,

        load_camera_bridge,
        image_subscription_node,
        image_process_node,
        image_analyze_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[set_hardware_interface_active],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_hardware_interface_active,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_joint_traj_controller,
                        load_mast_joint_traj_controller,
                        load_wheel_joint_traj_controller,
                        load_steer_joint_traj_controller,
                        load_suspension_joint_traj_controller],
            )
        ),
    ])