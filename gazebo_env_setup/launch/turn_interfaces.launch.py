from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # ── 경로 준비 ──────────────────────────────────────────────────────────────
    env_setup_dir = get_package_share_directory('gazebo_env_setup')
    px4_ros_com_dir = get_package_share_directory('px4_ros_com')
    rviz_config    = os.path.join(env_setup_dir, 'config', 'asp_final_proj.rviz')

    # ── 1. pose_tf_broadcaster.launch.py ─────────────────────────────────────
    pose_tf_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(env_setup_dir, 'launch', 'pose_tf_broadcaster.launch.py')
        )
    )

    # ── 2. topic_bridge.launch.py ────────────────────────────────────────────
    topic_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(env_setup_dir, 'launch', 'topic_bridge.launch.py')
        )
    )

    offboard = Node(
        package='px4_ros_com',
        executable='offboard_control',
        name='offboard_control',
        output='screen',
        parameters=[
            os.path.join(px4_ros_com_dir, 'config', 'offboard_control_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # ── 4. Micro XRCE-DDS Agent ──────────────────────────────────────────────
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent',
        output='screen'
    )

        # ── RViz 노드 ────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # ── LaunchDescription 반환 ───────────────────────────────────────────────
    return LaunchDescription([
        pose_tf_broadcaster,
        topic_bridge,
        offboard,
        micro_xrce_agent,
        rviz_node
    ])
