import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # rviz_dir = LaunchConfiguration(
    #     'rviz_dir',
    #     default=os.path.join(
    #         get_package_share_directory('tribot_fake_node'), 'launch')
    # ),

    # launch_dir = os.path.join(get_package_share_directory('tribot_fake_node'), 'launch')
    # world = os.path.join(get_package_share_directory('tribot_fake_node'),
    #                      'worlds', 'turtlebot.world')
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('tribot_fake_node'), 'param', 'tribot.yaml')
    ),

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Needs to be false for transform from [base_footprint] to [base_link]
    use_tf_static = LaunchConfiguration('use_tf_static', default='false')

    urdf_file_name = 'turtlebot3_waffle.urdf'
    # doc = xacro.parse(open(os.path.join(
    #     get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)))
    # xacro.process_doc(doc)
    # urdf = doc.toxml()
    urdf = os.path.join(
    get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),
                                                    'launch'),
                                      '/gazebo.launch.py']),
        )

    # start_gazebo_server = ExecuteProcess(
    #     cmd=['gzserver', '-s', world],
    #     output='screen'
    # )
    # start_gazebo_client = ExecuteProcess(
    #     cmd=['gzclient'],
    #     output='screen'
    # )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tribot_fake_node'),
                         'launch',
                         'rviz2.launch.py'))
            )

    tribot_fake_node = Node(
            package='tribot_fake_node',
            executable='tribot_fake_node',
            output='screen'
            )

    joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            )

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                    {'use_sim_time': use_sim_time}
                    # {'use_tf_static': use_tf_static},
                    # {'robot_description': urdf}
                ],
            arguments=[urdf]
            )

    # TODO(harshil): crashes tribot_fake_node and rviz2
    # robot_localization_node = Node(
    #         package='robot_localization',
    #         executable='ekf_node',
    #         name='ekf_filter_node',
    #         output='screen',
    #         parameters=[
    #                 os.path.join(get_package_share_directory('tribot_gazebo'),
    #                              'config', 'ekf.yaml'),
    #                 {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #             ]
    #         )

    # robot_spawner_node = Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         arguments=['-topic', 'robot_description', '-entity', 'tribot'],
    #         output='screen',
    #     )

    return LaunchDescription([
        LogInfo(msg=['Execute Tribot Fake Node!']),

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Specify parameters',
        ),

        gazebo_launch,
        # rviz_launch,
        # start_gazebo_server,
        # start_gazebo_client,

        tribot_fake_node,

        # Use the inject_entity script, it gives more control
        robot_state_publisher_node,
        joint_state_publisher_node,
        # robot_spawner_node,
        # robot_localization_node,
    ])
