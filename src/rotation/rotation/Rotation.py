#!/user/bin/env python
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
import time



class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('Rotation')
        self.publisher_1 = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        self.publisher_2= self.create_publisher(PoseStamped, '/robot2/goal_pose', 10)
        self.publisher_3 = self.create_publisher(PoseStamped, '/robot3/goal_pose', 10)

        timer_period = 8 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
       

    def timer_callback(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.3
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        self.publisher_1.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.6
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        self.publisher_2.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        self.publisher_3.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)

        time.sleep(13)





        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.6
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z =  0.01
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        self.publisher_1.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)

        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z =  0.01
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        self.publisher_2.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)

        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.3
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z =  0.01
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        self.publisher_3.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)
        time.sleep(13)




        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z =  0.01
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0
     
        self.publisher_1.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)

        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.3
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z =  0.01
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        self.publisher_2.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)

        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 0.6
        goal_pose.pose.position.y = 0.5
        goal_pose.pose.position.z =  0.01
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0

        self.publisher_3.publish(goal_pose)
        self.get_logger().info('Publishing: "%s"' % goal_pose)

        time.sleep(13)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()