from uuid import uuid4

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    partition = "ur3e_write_q_" + uuid4().hex

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
            "start_joint_controller": "true",
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "false",
            "launch_servo": "false",
        }.items(),
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_moveit",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("ur3e_write_q"), "config", "draw_q.rviz"])],
        parameters=[
            {"use_sim_time": True},
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]),
        ],
        output="screen",
    )
    planner = Node(
        package="ur3e_write_q", executable="plan_q.py",
        parameters=[{"use_sim_time": True}], output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        # Spawn, server, GUI and clock bridge must discover the same isolated world.
        SetEnvironmentVariable("IGN_PARTITION", partition),
        SetEnvironmentVariable("GZ_PARTITION", partition),
        sim,
        # Keep RViz on the standard action name. Scope backend remaps to MoveIt
        # because RViz's internal client node does not inherit action remaps.
        GroupAction([
            SetRemap(src="/move_action", dst="/draw_q/backend_move_action"),
            *[SetRemap(src=f"/move_action/_action/{endpoint}",
                       dst=f"/draw_q/backend_move_action/_action/{endpoint}")
              for endpoint in ("send_goal", "get_result", "cancel_goal", "feedback", "status")],
            moveit,
        ]),
        planner, rviz,
    ])
