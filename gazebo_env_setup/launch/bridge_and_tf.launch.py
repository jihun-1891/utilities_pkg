# gazebo_env_setup/launch/bridge_and_tf.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # ── 공통 인자 ────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation time if true',
    )

    # ── 패키지·파일 경로 ─────────────────────────────────────────────────────
    pkg_gazebo_env_setup = get_package_share_directory('gazebo_env_setup')
    
    topic_bridge_launch       = os.path.join(pkg_gazebo_env_setup, 'launch', 'topic_bridge.launch.py')
    pose_tf_broadcaster_launch = os.path.join(pkg_gazebo_env_setup, 'launch', 'pose_tf_broadcaster.launch.py')
    rviz_config               = os.path.join(pkg_gazebo_env_setup, 'config', 'asp_final_proj.rviz')

    # ── 포함 런치 파일들 ─────────────────────────────────────────────────────
    topic_bridge_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(topic_bridge_launch),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )

    pose_tf_broadcaster_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pose_tf_broadcaster_launch),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
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

    # ── LaunchDescription 반환 ──────────────────────────────────────────────
    return LaunchDescription([
        use_sim_time_arg,
        topic_bridge_include,
        pose_tf_broadcaster_include,
        rviz_node,   # ← RViz 추가!
    ])
