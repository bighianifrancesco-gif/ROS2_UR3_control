from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("my_arm_description")
    ur_share  = get_package_share_directory("ur_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(pkg_share, "worlds", "ur3_imu_world.sdf")
    urdf_path  = os.path.join(pkg_share, "urdf", "ur3_expanded.urdf")

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([
            pkg_share,
            os.path.join(pkg_share, "models"),
            ur_share,
            os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        ])
    )

    # robot_state_publisher provides the robot_description param to gz_ros2_control
    robot_description = ParameterValue(
        Command(f"cat {urdf_path}"),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    peg_fsm = Node(
        package='arm_control',
        executable='peg_insertion_node',
        output='screen',
        parameters=[{
            # you can pass topic names, step sizes, etc here
        }]
    )


    delayed_spawners = TimerAction(period=3.0, actions=[jsb_spawner, jtc_spawner])

    return LaunchDescription([
        set_gz_resource_path,
        robot_state_publisher,   # must exist for gz_ros2_control
        gz,
        delayed_spawners,
        peg_fsm,
    ])
