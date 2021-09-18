import os
import yaml
from launch.substitutions.text_substitution import TextSubstitution
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# TODO(harshil): Take robot initial positions from a text file
# def generate_robot_list(number_of_robots):
#    robots = []
#    for i in range(number_of_robots):
#        robot_name = "tribot" + str(i)
#        x_pos = float(i)
#        robots.append({'name': robot_name,
#                       'x_pose': x_pos,
#                       'y_pose': 0.0,
#                       'z_pose': 0.01})
#    return robots


def generate_robot_list(robots_file):
    robots = []
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
    return robots


def generate_urdf():
    urdf_file_name = 'tribot.urdf.xacro'
    doc = xacro.parse(open(os.path.join(
        get_package_share_directory('tribot_description'), 'urdf', urdf_file_name)))
    xacro.process_doc(doc)
    urdf = doc.toxml()
    return urdf


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('tribot_description'), 'urdf', 'tribot.urdf')
    robots = generate_robot_list(os.path.join(get_package_share_directory('tribot_description'),
                                              'params', 'spawn_params.yaml'))
    spawn_robots_cmds = []

    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                                              get_package_share_directory('tribot_description'),
                                              'launch',
                                              'tribot_spawn.launch.py')),
                launch_arguments={
                    'robot-urdf': urdf,
                    'x': TextSubstitution(text=str(robot['x_pose'])),
                    'y': TextSubstitution(text=str(robot['y_pose'])),
                    'z': TextSubstitution(text=str(robot['z_pose'])),
                    'robot-name': robot['name'],
                    'robot-namespace': robot['name']
                }.items()
            )
        )

    print(str(robots))

    ld = LaunchDescription()
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld
