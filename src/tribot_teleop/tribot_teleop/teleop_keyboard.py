#!/usr/bin/env python

import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

import termios
import tty

MAX_LIN_VEL = 0.25
MAX_ANG_VEL = 1.5
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


msg = """
Control Your Bot!
---------------------------
Moving around:
        w
   a    x    d
        s
w/s : increase/decrease linear velocity (Max : ~ 0.25)
a/d : increase/decrease angular velocity (Max : ~ 1.5)
space key, x : force stop
CTRL-C to quit
"""

err = """
Communications Failed!
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_velocity(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '
          .format(target_linear_velocity, target_angular_velocity))


def make_simple_profile(output, input, slope):
    if input > output:
        output = min(input, output + slope)
    elif input < output:
        output = max(input, output - slope)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def main():
    if (len(sys.argv) > 2):
        print('usage: ros2 run tribot_teleop teleop_keyboard -- foo_namespace')
        sys.exit(1)

    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')

    try:
        namespace = sys.argv[1]
        pub = node.create_publisher(Twist, namespace + '/cmd_vel', qos)
    except Exception:
        namespace = None
        pub = node.create_publisher(Twist, 'cmd_vel', qos)
    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_velocity(target_linear_velocity, target_angular_velocity)
            elif key == 's':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_velocity(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_velocity(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_velocity(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 'x':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_velocity(target_linear_velocity, target_angular_velocity)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
