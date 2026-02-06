# hanford_arm_moveit_config/launch/demo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml
from launch.actions import TimerAction

def generate_launch_description():
    pkg = get_package_share_directory("hanford_arm_moveit_config")

    urdf_xacro = os.path.join(pkg, "config", "pit_robot_robotonly.urdf.xacro")
    srdf_file  = os.path.join(pkg, "config", "pit_robot_robotonly.srdf")
    kin_file   = os.path.join(pkg, "config", "kinematics.yaml")
    ctrls_file = os.path.join(pkg, "config", "moveit_controllers.yaml")
    ros2ctrl   = os.path.join(pkg, "config", "ros2_controllers.yaml")
    init_pos   = os.path.join(pkg, "config", "initial_positions.yaml")  # adjust if different
    sim_time = {"use_sim_time": True}

    mcfg = (
        MoveItConfigsBuilder("pit_robot_robotonly", package_name="hanford_arm_moveit_config")
        .robot_description(
            file_path=urdf_xacro,
            # include this mappings block only if your xacro expects these args:
            mappings={"initial_positions_file": init_pos}
        )
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kin_file)
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"]
        )
        .trajectory_execution(file_path=ctrls_file)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[mcfg.robot_description,sim_time],
        output="screen",
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[mcfg.robot_description, ros2ctrl,sim_time],
        output="screen",
    )

    sp_js  = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--activate"],
        output="screen"
    )
    sp_arm = Node(
        package="controller_manager", executable="spawner",
        arguments=["hanford_manipulator_controller", "--controller-manager", "/controller_manager", "--activate"],
        output="screen"
    )
    sp_lin = Node(
        package="controller_manager", executable="spawner",
        arguments=["linear_axis_controller", "--controller-manager", "/controller_manager", "--activate"],
        output="screen"
)

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            mcfg.to_dict(),
            ctrls_file,
            sim_time,
            {"planning_scene_monitor": {
                "publish_robot_description": True,
                "publish_robot_description_semantic": True
            }},
        ],
        remappings=[  # silence deprecation and match controller publishers
            ("/hanford_manipulator_controller/state",
             "/hanford_manipulator_controller/controller_state"),
            ("/linear_axis_controller/state",
             "/linear_axis_controller/controller_state"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[mcfg.to_dict(),sim_time],
        output="screen",
        remappings=[
            ("/hanford_manipulator_controller/state",
             "/hanford_manipulator_controller/controller_state"),
            ("/linear_axis_controller/state",
             "/linear_axis_controller/controller_state"),
        ],
    )

    relay = Node(
        package="isaac_joint_command_relay",
        executable="isaac_joint_command_relay_node",   # <- adjust if your target name differs
        output="screen",
        parameters=[{
            "arm_state_topic": "/hanford_manipulator_controller/state",
            "lin_state_topic": "/linear_axis_controller/state",
            "publish_rate_hz": 50.0,
            "use_sim_time": True,
        }],
        remappings=[
            ("/linear_axis_controller/state",
             "/linear_axis_controller/controller_state"),
            ("/hanford_manipulator_controller/state",
             "/hanford_manipulator_controller/controller_state"),
        ],
    )


    static_tf_robot_zed = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_robot_camera_to_zed",
        arguments=[
            "0.0", "0.0", "0.0",      # x y z (meters) – start with identity
            "0.0", "-1.57", "0.0",      # roll pitch yaw (radians)
            "camera_link",            # parent: robot camera mount
            "zed_camera_link",      # child: ZED main frame
        ],
        output="screen",
    )

    return LaunchDescription([
        rsp, ros2_control,
        TimerAction(period=0.5, actions=[sp_js]),
        TimerAction(period=0.8, actions=[sp_arm]),
        TimerAction(period=1.1, actions=[sp_lin]),
        move_group, rviz, relay, static_tf_robot_zed,
    ])
