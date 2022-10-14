#!/usr/bin/env python3
import roslib
import rospy
from sensor_msgs.msg import JointState as RosJointState
from hiwonder_servo_msgs.msg import JointState as HiwonderJointState

class JointStateMessage:
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher:
    def __init__(self):
        rospy.init_node('hiwonder_joint_state_publisher', anonymous=True)

        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
        self.joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']

        self.servos = []
        self.controllers = []
        self.joint_states = {}

        for controller in sorted(self.joints):
            self.joint_states[controller] = JointStateMessage(controller, 0.0, 0.0, 0.0)
            self.controllers.append(controller)

        # Start controller state subscribers
        [rospy.Subscriber(c + '_controller'+'/state', HiwonderJointState, self.controller_state_handler) for c in self.controllers]

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', RosJointState, queue_size=1)

        rospy.loginfo("Starting Joint State Publisher at " + str(rate) + "Hz")

        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    def controller_state_handler(self, msg):
        js = JointStateMessage(msg.name, msg.current_pos, msg.velocity, 0)
        self.joint_states[msg.name] = js

    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = RosJointState()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(0)
            msg.effort.append(0)

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        self.joint_states_pub.publish(msg)


if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
