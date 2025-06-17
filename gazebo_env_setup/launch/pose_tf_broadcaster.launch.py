# gazebo_env_setup/launch/pose_tf_broadcaster.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    # ─── 런치 인자 선언 ────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation time if true'
    )

    # 모든 ROS 노드에 전역 파라미터로 적용
    set_sim_param = SetParameter(
        name='use_sim_time',
        value=LaunchConfiguration('use_sim_time')
    )

    # ─── ① TF 브로드캐스터 (동적 pose) ─────────────
    pose_tf_broadcaster = Node(
        package='gazebo_env_setup',
        executable='pose_tf_broadcaster',
        name='pose_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ─── ② Static TF: base_link → camera_front ───
    cam_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        output='screen',
        arguments=[
            '0.43', '0.0', '0.26',            # xyz
            '0', '0', '0', '1',               # qx qy qz qw
            'X1_asp/base_link',
            'X1_asp/base_link/camera_front'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ─── ③ Static TF: base_link → gpu_lidar ─────
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        output='screen',
        arguments=[
            '0.60', '0.0', '0.13',
            '0', '0', '0', '1',
            'X1_asp/base_link',
            'X1_asp/base_link/gpu_lidar'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        set_sim_param,
        pose_tf_broadcaster,
        cam_static_tf,
        lidar_static_tf
    ])
