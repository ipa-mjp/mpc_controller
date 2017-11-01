#!/usr/bin/python

import rospy
import math
from sensor_msgs.msg import JointState


def jointStatePub():

    rospy.init_node("test_pub_JointState", anonymous=True)

    pub = rospy.Publisher("/mpc_controller/joint_states", JointState, queue_size=1)
    joint_state = JointState()

    joint_state.position = [1.57, 1.57, 1.57, 1.57, 0., 0., 0.]
    joint_state.velocity = [1.57, 1.57, 1.57, 1.57, 0., 0., 0.]
    joint_state.effort = [1.57, 1.57, 1.57, 1.57, 0., 0., 0.]

    r = rospy.Rate(50)

    while not rospy.is_shutdown():
        pub.publish(joint_state)
        r.sleep()



def jointStateSub():

    rospy.init_node("test_sub_JointState", anonymous=True)

    joint_states = [0., 0., 0., 0., 0., 0., 0.];
    rospy.Subscriber("joint_states", JointState, jointStateCallback)

    rospy.sleep(5.0)
    rospy.spin()

def  jointStateCallback(msg):

    print ("\033[95m" + "Joint_names :[ ")
    for i in range(0, len(msg.name)):
         print str(msg.name[i])
    print (" ] " + "\033[0m")

    print ("\033[95m" + "Joint_Position :[ ")
    for i in range(0, len(msg.position)):
         print str(msg.position[i])
    print (" ] " + "\033[0m")

    print ("\033[95m" + "Joint_velocity :[ ")
    for i in range(0, len(msg.velocity)):
         print str(msg.velocity[i])
    print (" ] " + "\033[0m")


if __name__ == '__main__':
  try:
      #jointStatePub()
      jointStateSub()
  except rospy.ROSInterruptException: pass
