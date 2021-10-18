#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub

    while path.poses:
        # Call service client with path
        resp = control_client(path)
        setpoint = resp.setpoint
        new_path = resp.new_path

        # Transform Setpoint from service client
        transform = tf_buffer.lookup_transform(
            robot_frame_id,
            setpoint.header.frame_id,
            rospy.Time(0)
            )
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(
            setpoint,
            transform
            )

        # Create Twist message from the transformed Setpoint
        msg = Twist()
        msg.angular.z = min(
            atan2(transformed_setpoint.point.y, transformed_setpoint.point.x),
            max_angular_velocity
            )
        msg.linear.x = min(
            hypot(transformed_setpoint.point.x, transformed_setpoint.point.y),
            max_linear_velocity
            )

        # Set linear velocity to 0 when angular velocity is close to maximum
        if msg.angular.z > 0.85:
            msg.linear.x = 0

        # Publish Twist
        pub.publish(msg)
        rate.sleep()

        # Call service client again if the returned path is not empty and do stuff again
        path = new_path

    # Send 0 control Twist to stop robot
    msg.angular.z = 0
    msg.linear.x = 0
    pub.publish(msg)
    rate.sleep()

    # Get new path from action server
    get_path()


def get_path():
    global goal_client

    # Get path from action server
    goal_client.wait_for_server()
    goal = irob_assignment_1.msg.GetNextGoalGoal()
    goal_client.send_goal(goal)
    goal_client.wait_for_result()

    # Call move with path from action server
    move(goal_client.get_result().path)


if __name__ == "__main__":
    # Init node
    rospy.init_node('controller')

    # Init publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(15)

    # Init simple action client
    goal_client = actionlib.SimpleActionClient(
        'get_next_goal',
        irob_assignment_1.msg.GetNextGoalAction
        )

    # Init service client
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

    # Init tf buffer and listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path
    get_path()

    # Spin
    rospy.spin()
