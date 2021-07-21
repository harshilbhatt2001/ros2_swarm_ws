#!/usr/bin/env python3
import os
import argparse
import xacro
import random
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy

def generate_urdf():
    urdf_file_name = 'tribot.urdf.xacro'
    doc = xacro.parse(open(os.path.join(
        get_package_share_directory('tribot_description'), 'urdf', urdf_file_name)))
    xacro.process_doc(doc)
    urdf = doc.toxml()
    
    return urdf

def main():
    parser = argparse.ArgumentParser(description="Spawn Tribot into Gazebo with Navigation2")
    parser.add_argument('-urdf', '--robot-urdf', type=str, default='dummy_urdf', 
                        help='URDF of the robot to spawn')
    parser.add_argument('-n', '--robot-name', type=str, default='dummy_robot', 
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot-namespace', type=str, default='dummy_robot_ns',
                        help='ROS namespace for topics')
    parser.add_argument('-namespace', '--namespace', type=bool, default=True,
                        help="Wether to enable namespacing")
    parser.add_argument('-x', type=float, default=0,
                        help='x component of initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='y component of initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='z component of initial position [meters]')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = rclpy.create_node('entity_spawner')
    
    node.get_logger().info('Creating service client to connect to /spawn_entity')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        node.get_logger().info('waiting for spawn_entity service...')
        client.wait_for_service()
        node.get_logger().info('...connected!')
    
    #urdf_content = generate_urdf()
    #with open("/dev/null/tribot" + str(random.randint(0, 100)) + ".urdf", "w") as urdf:
    #    print(urdf_content, file=urdf)
    #urdf_file_path = args.robot_urdf
    urdf_file_path = os.path.join(get_package_share_directory('tribot_description'), 'urdf', 'tribot.urdf')
    print(urdf_file_path)

    # remap tranform `/tf` topic so each bot has it's own.
    # This is done by adding `ROS argument entries` to the urdf file for each plugin broadcasting a transform
    # remapping rule, i.e, /tf -> /<robot-id>/tf 
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()
    imu_plugin = None
    diff_drive_plugin = None
    battery_plugin = None
    for plugin in root.iter('plugin'):
        if 'diff_drive_plugin' in plugin.attrib.values():
            diff_drive_plugin = plugin
        elif 'imu_plugin' in plugin.attrib.values():
            imu_plugin = plugin
        elif 'linear_battery_plugin' in plugin.attrib.values():
            battery_plugin = plugin
    
    tag_diff_drive_ros_params = diff_drive_plugin.find('ros')
    tag_diff_drive_ns = ET.SubElement(tag_diff_drive_ros_params, 'namespace')
    tag_diff_drive_ns.text = '/' + args.robot_namespace
    ros_tf_remap = ET.SubElement(tag_diff_drive_ros_params, 'remapping')
    ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'

    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding='unicode')
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)

    if (args.namespace):
        node.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
            args.robot_name, args.robot_namespace, args.x, args.y, args.z))
        
        request.robot_namespace = args.robot_namespace
        print(args.robot_namespace)
    
    else:
        node.get_logger().info('spawning `{}` at {}, {}, {}'.format(
            args.robot_name, args.x, args.y, args.z))
    
    node.get_logger().info('Spawning Tribot using service `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError('exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()