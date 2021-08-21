#!/usr/bin/env python

import rospy
import numpy as np
from dragon_valve.dragon_interface import DragonInterface
from tf.transformations import *
from sensor_msgs.msg import JointState

class DragonInterfaceJoy(DragonInterface):
    def __init__(self):
        super(DragonInterfaceJoy, self).__init__()

        self.trigger = False

    def getTrigger(self):
        return self.trigger

    def setTrigger(self, flag):
        self.trigger = flag

    def joyCallback(self, msg):
        if msg.buttons[4] == 1: #L1
            self.trigger = True
        else:
            self.trigger = False

def main():

    rospy.init_node('dragon_valve_motion')
    wind_gimbal = rospy.get_param('~wind_gimbal', False)

    robot = DragonInterfaceJoy()

    rospy.sleep(2)

    joint_state = JointState()
    joint_state.name = ['joint1_pitch']
    joint_state.position = [np.pi / 2]
    robot.setJointAngle(joint_state)
    yaw = euler_from_quaternion(robot.getBaselinkRot())[2]
    robot.targetMotion(robot.getCogPos(), rot = quaternion_from_euler(-np.pi/2, 0.6, yaw))

    while not rospy.is_shutdown():

        if wind_gimbal:
            if robot.getTrigger():
                robot.windGimbal(0) # wind gimbal1 roll
                robot.setTrigger(False)
        else:
            if robot.getTrigger():
                robot.inactivateRotor(0) # stop rotor1
            else:
                robot.inactivateRotor(-1) # activate all rotor


        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
