import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tribot_bt_dir = get_package_share_directory('tribot_bt')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    
    lifecycle_nodes = ['main']

    main_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='main',
        namespace=namespace,
        output='screen',
        parameters=[
          tribot_bt_dir + '/config/params.yaml',
          {
            'action_name': 'main',
            'bt_xml_file': tribot_bt_dir + '/behavior_trees_xml/main.xml'
          }
        ])

    nav2_cmd = Node(
        package='tribot_bt',
        executable='nav2_sim_node',
        name='nav2_node',
        namespace=namespace,
        output='screen'
            )


    lifecycle_node_cmd  =  Node(
          package='nav2_lifecycle_manager',
          executable='lifecycle_manager',
          name='lifecycle_manager_navigation',
          output='screen',
          parameters=[{'use_sim_time': True},
                      {'autostart': True},
                      {'node_names': lifecycle_nodes}])
    
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    # ld.add_action(plansys2_cmd)

    ld.add_action(main_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(lifecycle_node_cmd)

    # ld.add_action(transport_cmd)
    # ld.add_action(assemble_cmd)

    return ld
