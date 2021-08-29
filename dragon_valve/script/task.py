#!/usr/bin/env python

import rospy
import smach
import smach_ros
import functools
from dragon_valve.dragon_interface import DragonInterface
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Transform, Inertia, PoseArray, PoseStamped, Wrench, Quaternion
import tf.transformations as tft
import ros_numpy as ros_np
from std_msgs.msg import UInt8, Empty
import copy
import std_srvs.srv
from tf.transformations import *
from sensor_msgs.msg import Joy
import tf2_ros
from jsk_rviz_plugins.msg import OverlayText


def poseConvergenceCheck(robot, target_pos, target_rot, pos_thresh, rot_thresh, vel_thresh):

    current_pos = robot.getCogPos()
    current_rot = robot.getBaselinkRot()
    current_vel = robot.getCogLinearVel()
    # TODO: consider the convergence of velocity

    delta_pos = target_pos - current_pos
    delta_rot = quaternion_multiply(quaternion_inverse(target_rot), current_rot)  # Note: we assume the  z axis of target baselink coincides with the z axis of valve coord
    delta_angle, _, _ = rotation_from_matrix(quaternion_matrix(delta_rot))
    if delta_angle > np.pi:
        delta_angle -= np.pi * 2
    elif delta_angle < -np.pi:
        delta_angle += np.pi * 2

    if robot.debug_view_:
        text = OverlayText()
        text.width = 400
        text.height = 200
        text.left = 10
        text.top = 10
        text.text_size = 12
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.text = 'CoG Diff\n  pos: {:.4g}, yaw: {:.4g}, rot: {:.4g}\n SetPoint\n  x: {:.4g} y: {:.4g} z: {:.4g}\n CurrentPoint\n  x: {:.4g} y: {:.4g} z: {:.4g}'.format(np.linalg.norm(delta_pos), abs(delta_angle), np.linalg.norm(current_vel), target_pos[0], target_pos[1], target_pos[2], current_pos[0], current_pos[1], current_pos[2])

        rospy.loginfo_throttle(0.5, "\n" + text.text)
        robot.nav_debug_pub_.publish(text)

    if np.linalg.norm(delta_pos) < pos_thresh and abs(delta_angle) < rot_thresh and np.linalg.norm(current_vel) < vel_thresh:
        return True
    else:
        return False

def valveApproachConvergenceCheck(robot, z_thresh, roll_pitch_thresh, target_pos, target_rot, pos_thresh, vel_thresh, yaw_thresh):

    current_pos = robot.getCogPos()
    current_rot = robot.getBaselinkRot()
    current_vel = robot.getCogLinearVel()

    # note: we assume the  z axis of target baselink coincides with the z axis of valve coord
    delta_pos = translation_from_matrix(concatenate_matrices(quaternion_matrix(quaternion_inverse(target_rot)),
                                                             translation_matrix(current_pos - target_pos)))
    delta_euler = np.array(euler_from_quaternion(quaternion_multiply(quaternion_inverse(target_rot), current_rot)))

    if robot.debug_view_:
        text = OverlayText()
        text.width = 400
        text.height = 200
        text.left = 10
        text.top = 10
        text.text_size = 12
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.text = 'CoG Diff\n  pos: {}, euler: {}, vel: {:.4g}\n SetPoint\n  x: {:.4g} y: {:.4g} z: {:.4g}\n CurrentPoint\n  x: {:.4g} y: {:.4g} z: {:.4g}'.format(delta_pos, delta_euler, np.linalg.norm(current_vel), target_pos[0], target_pos[1], target_pos[2], current_pos[0], current_pos[1], current_pos[2])

        rospy.loginfo_throttle(0.5, "\n" + text.text)
        robot.nav_debug_pub_.publish(text)

    if np.linalg.norm(delta_pos[:2]) < pos_thresh and np.fabs(delta_pos[2]) < z_thresh and abs(delta_euler[0]) < roll_pitch_thresh and abs(delta_euler[1]) < roll_pitch_thresh and abs(delta_euler[2]) < yaw_thresh and np.linalg.norm(current_vel) < vel_thresh:
        return True
    else:
        return False

class BaseState(smach.State):
    def __init__(self, robot, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.robot = robot

        robot_name = self.robot.getRobotName()
        self.end_effector_name = robot_name + '/' + rospy.get_param('~end_effector_name', 'gripper')
        self.baselink_name = robot_name + '/' + rospy.get_param('~baselink_name', 'baselink')
        self.cog_name = robot_name + '/' + rospy.get_param('~cog_name', 'cog')

        self.valve_type = rospy.get_param('~valve_type', '3bridges')
        self.valve_head_offset = np.pi / int(self.valve_type[0]) / 2 # to set the end effector tips to the center of the vallve openning space.
        if self.valve_type == '3bridges':
            self.valve_head_offset = np.pi / 2  # special case

    def calculateTargetCogPose(self, tip_offset = [0, 0, 0]):

        valve_pose = self.robot.getValvePose()
        target_end_effector_trans = concatenate_matrices(ros_np.numpify(valve_pose),  # base
                                                             translation_matrix(tip_offset),  # origin offset
                                                             euler_matrix(0, 0, self.valve_head_offset),  # yaw offset
                                                             euler_matrix(0, np.pi/2, 0))  # gripper orientation regarding valve


        cog_trans = self.robot.getTF(self.cog_name, parent_frame_id= self.end_effector_name)
        baselink_trans = self.robot.getTF(self.baselink_name, parent_frame_id= self.end_effector_name)

        target_cog_trans = concatenate_matrices(target_end_effector_trans, \
                                                ros_np.numpify(cog_trans.transform))
        target_baselink_trans = concatenate_matrices(target_end_effector_trans, \
                                                     ros_np.numpify(baselink_trans.transform))


        target_cog_pos = translation_from_matrix(target_cog_trans)
        target_cog_rot = quaternion_from_matrix(target_baselink_trans) # use the raw baselink rotation

        return target_cog_pos, target_cog_rot


class Start(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot,
                           outcomes=['succeeded', 'preempted'],
                           io_keys=['init_cog_pos', 'init_cog_yaw'])

        self.task_start = False
        self.task_start_sub = rospy.Subscriber('/task_start', UInt8, self.taskStartCallback)

    def taskStartCallback(self, msg):
        self.task_start = True

    def execute(self, userdata):

        while not self.task_start:
            rospy.sleep(0.1)
            rospy.logdebug_throttle(1.0, "wait to start task")

            if rospy.is_shutdown():
                return 'preempted'

            pass


        if self.robot.getFlightState() == self.robot.ARM_OFF_STATE or \
           self.robot.getFlightState() == self.robot.ARM_ON_STATE:

            self.robot.setJointTorque(True)
            rospy.sleep(0.5)

            if self.robot.getFlightState() == self.robot.ARM_OFF_STATE:
                rospy.loginfo(self.__class__.__name__ + ': arming motors')
                self.robot.start(sleep = 2.0)

            rospy.loginfo(self.__class__.__name__ + ': takeoff')
            self.robot.takeoff()

            while not (self.robot.getFlightState() == self.robot.HOVER_STATE):

                if self.robot.getFlightState() == self.robot.LAND_STATE or \
                   self.robot.getFlightState() == self.robot.STOP_STATE or \
                   self.robot.getFlightState() == self.robot.ARM_OFF_STATE:
                    rospy.logwarn(self.__class__.__name__ + ': task preempted')
                    return 'preempted'

                if rospy.is_shutdown():
                    return 'preempted'

                rospy.sleep(0.1)
                pass

            rospy.loginfo(self.__class__.__name__ + ': reach hovering')


        userdata.init_cog_pos = self.robot.getCogPos()

        # check the valve msg
        if self.robot.getValvePose() is None:
            rospy.logwarn(self.__class__.__name__ + ': no valve pose message received')
            return 'preempted'

        return 'succeeded'

class Pose(BaseState):
    def __init__(self, robot,
                 outcomes=['succeeded', 'failed'],
                 io_keys=['reverse_joint']):
        BaseState.__init__(self, robot, outcomes=outcomes, input_keys=[], output_keys=[], io_keys=io_keys)
        self.joint_conv_thresh = rospy.get_param('~pose/joint/conv_thresh', 0.017)
        self.rot_conv_thresh = rospy.get_param('~pose/rot/conv_thresh', 0.1)

    def execute(self, userdata):

        # joint angle
        joint_state = JointState()
        joint_state.name = ['joint1_pitch']
        valve_coord_z_axis = quaternion_matrix(ros_np.numpify(self.robot.getValvePose().orientation))[:3, 2]
        if valve_coord_z_axis[2] < -0.001: # downward
            joint_state.position = [-np.pi / 2]
            userdata.reverse_joint = True
        else:  # upward
            joint_state.position = [np.pi / 2]
            userdata.reverse_joint = False
        self.robot.setJointAngle(joint_state)

        if np.abs(valve_coord_z_axis[2]) < np.cos(30):
            # need to change the rotation first
            _, target_rot = self.calculateTargetCogPose()
            # workaround for the joint angle of joint1_pitch
            # TODO: use robot model to get the precise kinematics
            target_rot = quaternion_multiply(target_rot, quaternion_from_euler(joint_state.position[0], 0, 0))
            self.robot.targetMotion(self.robot.getCogPos(), rot = target_rot)
        else:
            target_rot = self.robot.getBaselinkRot()


        while not self.robot.getForceSkipFlag():

            joint_angle = self.robot.getJointState().position[self.robot.getJointState().name.index('joint1_pitch')]
            diff_angle = joint_angle - joint_state.position[0]
            curr_robot_rot = self.robot.getBaselinkRot()
            diff_rot = quaternion_multiply(quaternion_inverse(target_rot), curr_robot_rot)

            if np.fabs(diff_angle) < self.joint_conv_thresh and \
               np.arccos(np.fabs(diff_rot[3])) * 2 < self.rot_conv_thresh:
                break

            if self.robot.getTaskHaltFlag():
                rospy.logwarn(self.__class__.__name__  + ": taks is halted")
                return 'failed'

            rospy.sleep(0.1)

        self.robot.resetForceSkipFlag()
        rospy.sleep(1) # workaround for final convergence

        return 'succeeded'


class Approach(BaseState):
    def __init__(self, robot, motion = 'approach',
                 outcomes=['succeeded', 'failed'],
                 input_keys=[],
                 output_keys=[],
                 io_keys=['target_cog_pos', 'target_cog_rot', 'est_wrench_offset', 'only_yaw']):
        BaseState.__init__(self, robot, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)

        self.motion = motion

        self.tip_offset = rospy.get_param('~' + motion + '/tip_offset', [0, 0, 0])
        self.pos_conv_thresh = rospy.get_param('~' + motion + '/pos_conv_thresh', 0.06)
        self.z_conv_thresh = rospy.get_param('~' + motion + '/z_conv_thresh', 0.06)
        self.vel_conv_thresh = rospy.get_param('~' + motion + '/vel_conv_thresh', 0.1)
        self.yaw_conv_thresh = rospy.get_param('~' + motion + '/yaw_conv_thresh', 0.1)
        self.roll_pitch_conv_thresh = rospy.get_param('~' + motion + '/roll_pitch_conv_thresh', 0.1)

        self.convergent_func = functools.partial(valveApproachConvergenceCheck, self.robot, self.z_conv_thresh, self.roll_pitch_conv_thresh) # special convergence check function

    def execute(self, userdata):

        target_pos, target_rot = self.calculateTargetCogPose(self.tip_offset)

        if userdata.only_yaw:
            # only extract yaw angle
            yaw = euler_from_quaternion(target_rot)[2]
            target_rot = quaternion_from_euler(0, 0, yaw)

        conv_flag = self.robot.goPoseWaitConvergence(target_pos, target_rot,
                                                     pos_thresh = self.pos_conv_thresh,
                                                     vel_thresh = self.vel_conv_thresh,
                                                     rot_thresh = self.yaw_conv_thresh,
                                                     timeout = 30,
                                                     check_func = self.convergent_func)

        # restore the offset of estimated wrench for further manipulation phase
        userdata.est_wrench_offset = self.robot.getEstimatedWrench()

        # TODO: do we need to return 'failed' if not reach convergence?
        if not conv_flag:
            while not self.robot.getForceSkipFlag():
                rospy.loginfo(self.__class__.__name__ + "_" + self.motion +": wait to force skip this convergence condition")
                rospy.sleep(1.0)

                if self.robot.getTaskHaltFlag():
                    rospy.logwarn(self.__class__.__name__  + "_" + self.motion + ": taks is halted")
                    return 'failed'

        self.robot.resetForceSkipFlag()

        userdata.target_cog_pos = target_pos
        userdata.target_cog_rot = target_rot

        return 'succeeded'

class Contact(Approach):
    def __init__(self, robot):
        Approach.__init__(self, robot, motion = 'contact',
                          input_keys=['only_yaw', 'reverse_joint'],
                          output_keys=[], io_keys=['target_cog_pos', 'target_cog_rot', 'init_torque'])

        self.rate = rospy.get_param('~contact/rate', 10.0) # hz
        self.angular_velocity = rospy.get_param('~contact/angular_velocity', 0.2) # rad/s
        self.init_torque = rospy.get_param('~contact/init_torque', 0.1) # Nm
        self.incre_torque_rate = rospy.get_param('~contact/incre_torque_rate', 1) # Nm/s
        self.move_thresh = rospy.get_param('~contact/move_thresh', 0.1) # valve or robot yaw angle move thresh
        self.check_interval = rospy.get_param('~contact/check_interval', 1.0) # s
        self.action = rospy.get_param('~manipulate/action', 'open')


    def execute(self, userdata):

        rospy.sleep(2.0) # wait for the convergence of approach adjust.

        delta_t = 1 / self.rate
        start_t = rospy.get_time()

        turn_direction = 0
        if self.action == 'open':
            turn_direction = 1
            # TODO: add check of joint angles
        elif self.action == 'close':
            turn_direction = -1
            # TODO: add check of joint angles
        else:
            rospy.logerror(self.__class__.__name__  + "_" + self.motion + ": no support action of {}, please choose in ['open', 'close']".format(self.action))
            return 'failed'

        contact = False
        prev_contact_t = rospy.get_time() + 2.0 # workaround

        init_valve_pose = ros_np.numpify(self.robot.getValvePose())
        init_valve_pos = translation_from_matrix(init_valve_pose)
        init_valve_rot = quaternion_from_matrix(init_valve_pose)

        target_pos_z = userdata.target_cog_pos[2]
        if not userdata.only_yaw:
            target_pos_z = translation_from_matrix(concatenate_matrices(inverse_matrix(init_valve_pose), translation_matrix(userdata.target_cog_pos)))[2] # w.r.t. valve coord
        prev_yaw = 0

        while True:

            # 1. rotation and angular velocity
            # Note: yaw is w.r.t. the valve coord (i.e., around the z axis of valve coord)
            curr_robot_rot = self.robot.getBaselinkRot()
            curr_yaw = euler_from_quaternion(quaternion_multiply(quaternion_inverse(init_valve_rot), curr_robot_rot))[2]
            if userdata.only_yaw:
                curr_yaw = euler_from_quaternion(curr_robot_rot)[2]
            else:
                if userdata.reverse_joint: # downward (0.001 is to handle sxsxsthe lateral one); TODO: set this as userdata which is determined at begininng
                    curr_robot_rot_dash = quaternion_multiply(curr_robot_rot, quaternion_from_euler(0, np.pi, 0))
                    curr_yaw = euler_from_quaternion(quaternion_multiply(quaternion_inverse(init_valve_rot), curr_robot_rot_dash))[2]

            turn_vel = turn_direction * self.angular_velocity
            delta_yaw = delta_t * turn_vel
            target_yaw = curr_yaw + delta_yaw # base on current yaw angle
            target_rot = quaternion_about_axis(target_yaw, (0,0,1))

            if not userdata.only_yaw:
                target_rot = quaternion_multiply(init_valve_rot, target_rot)

            if userdata.reverse_joint: # downward
                # turn the z axis 180 deg
                target_rot = quaternion_multiply(target_rot, quaternion_from_euler(0, np.pi, 0))
                if userdata.only_yaw:
                    target_rot = quaternion_about_axis(curr_yaw - delta_yaw, (0,0,1))
                    delta_yaw = -delta_yaw # reverse

            # 2. position and linear velocity

            local_cog = translation_from_matrix(concatenate_matrices(inverse_matrix(init_valve_pose), translation_matrix(self.robot.getCogPos()))) # w.r.t. valve coord
            if userdata.only_yaw:
                local_cog = self.robot.getCogPos() - init_valve_pos # w.r.t word coord

            r = np.linalg.norm(local_cog[:2])
            target_theta = np.arctan2(local_cog[1], local_cog[0]) + delta_yaw

            target_pos = np.array([r * np.cos(target_theta), r * np.sin(target_theta), target_pos_z]) # w.r.t. valve coord
            target_pos = translation_from_matrix(concatenate_matrices(init_valve_pose,
                                                                      translation_matrix(target_pos)))  # w.r.t. world coord

            if userdata.only_yaw:
                target_pos = np.array([init_valve_pos[0] + r * np.cos(target_theta), init_valve_pos[1] + r * np.sin(target_theta), target_pos_z])

            target_linear_vel = r * turn_vel * np.array([-np.sin(target_theta), np.cos(target_theta), 0])
            target_angular_vel = np.array([0, 0, turn_vel])

            if not userdata.only_yaw:
                target_linear_vel = translation_from_matrix(concatenate_matrices(quaternion_matrix(init_valve_rot),
                                                                                 translation_matrix(target_linear_vel)))
                if userdata.reverse_joint: # downward
                    target_angular_vel = -target_angular_vel # should be w.r.t. baselink frame
            else:
                if userdata.reverse_joint: # downward
                    target_angular_vel = -target_angular_vel
                    target_linear_vel = -target_linear_vel


            #print(" target linear vel: {}; target_angular_vel: {}, valve_z_axis[2]: {}, local_cog: {}, delta_yaw: {}, target_theta: {}".format(target_linear_vel, target_angular_vel, valve_z_axis[2], local_cog, delta_yaw, target_theta))
            self.robot.targetMotion(target_pos, rot = target_rot, linear_vel = target_linear_vel, angular_vel = target_angular_vel)


            # 3. external wrench which gradualy increases
            valve_force = np.array([0,0,0])
            valve_torque = np.array([0,0, self.init_torque * turn_direction]) # w.r.t valve coord
            if not userdata.only_yaw:
                valve_torque = translation_from_matrix(concatenate_matrices(quaternion_matrix(init_valve_rot),
                                                                            translation_matrix((valve_torque))))
            else:
                if userdata.reverse_joint: # downward
                    valve_torque = -valve_torque # reverse


            self.robot.addExternalWrench('valve', 'cog', valve_force, valve_torque)

            # check the contact with valve
            if rospy.get_time() - prev_contact_t > self.check_interval and not contact:

                # check the angle change within the duration of self.check_interval
                if np.abs(curr_yaw - prev_yaw) < self.move_thresh:
                    contact = True
                    rospy.loginfo(self.__class__.__name__  + "_" + self.motion + ": contact with valve handle, torque: {}".format(self.init_torque))

                prev_yaw = curr_yaw
                prev_contact_t = rospy.get_time()


            # 4. check the move of valve
            delta_valve_yaw = euler_from_quaternion(quaternion_multiply(quaternion_inverse(init_valve_rot),
                                                                  ros_np.numpify(self.robot.getValvePose().orientation)))[2]
            if turn_direction * delta_valve_yaw > self.move_thresh:
                rospy.loginfo(self.__class__.__name__  + "_" + self.motion + ": valve move with torque of {}".format(self.init_torque))
                userdata.init_torque = valve_torque
                userdata.target_cog_pos = target_pos
                userdata.target_cog_rot = target_rot
                return 'succeeded'

            if self.robot.getTaskHaltFlag():
                rospy.logwarn(self.__class__.__name__  + "_" + self.motion + ": taks is halted")
                self.robot.clearExternalWrench('valve')

                target_rot = self.robot.getBaselinkRot()
                if userdata.only_yaw:
                    target_rot = quaternion_from_euler(0, 0, self.robot.getBaselinkRPY()[2])

                self.robot.targetMotion(self.robot.getCogPos(), rot = target_rot)
                return 'failed'

            # TODO: give a more precise start time to incremently increase torque, and increase the self.incre_torque_rate
            # current 0.2 Nm/s is too slow, waste of time
            # The sart time can be calcualte from the valve type and the gripper offset
            # e.g. type3: the turn angle is around 60 deg (1.08 rad), the speed is self.angular_velocity = 0.2 rad/s, thus time is about 5 sec. give a margin of 80%
            # we can set start time as 4 second
            self.init_torque += self.incre_torque_rate * delta_t

            rospy.sleep(delta_t)



class Manipulate(Approach):
    def __init__(self, robot):
        Approach.__init__(self, robot, motion = 'manipulate',
                          input_keys=['init_grasp_cog_pos', 'init_grasp_cog_rot', 'init_torque', 'est_wrench_offset', 'only_yaw', 'reverse_joint'],
                          output_keys=[], io_keys=[])

        self.angular_velocity = rospy.get_param('~manipulate/angular_velocity', 1.0) # rad/s
        self.round_num = rospy.get_param('~manipulate/round_num', 1)
        self.torque_adjust_roll_k = rospy.get_param('~manipulate/torque_adjust_roll_k', 1.0)
        self.torque_adjust_yaw_k = rospy.get_param('~manipulate/torque_adjust_yaw_k', 1.0)
        self.roll_moment_thresh = rospy.get_param('~manipulate/roll_moment_thresh', 0.04) # rad
        self.torque_limit = rospy.get_param('~manipulate/torque_limit', 3.0) # Nm
        self.yaw_velocity_thresh = rospy.get_param('~manipulate/yaw_velocity_thresh', 0.1) # rad/s
        self.rate = rospy.get_param('~manipulate/rate', 20.0) # hz
        self.action = rospy.get_param('~manipulate/action', 'open')
        self.fixed_traj = rospy.get_param('~manipulate/fixed_traj', False)
        self.debug_mode = rospy.get_param('~debug_mode', False)
        self.wind_start_delay = rospy.get_param('~manipulate/wind_start_delay', 2.0) # sec
        self.wind_duration = rospy.get_param('~manipulate/wind_duration', 3.0) # sec

        if self.debug_mode:
            self.fixed_traj = True

    def execute(self, userdata):

        if self.debug_mode:
            rospy.sleep(3.0) # workaround to wait for the convergence of approach adjust.

        sum_turn_angle = 0

        init_valve_pose = ros_np.numpify(self.robot.getValvePose())
        init_valve_pos = translation_from_matrix(init_valve_pose)
        init_valve_rot = quaternion_from_matrix(init_valve_pose)
        valve_z_axis = quaternion_matrix(init_valve_rot)[:3, 2]
        gimbal_wind_flag = False
        if np.abs(valve_z_axis[2]) < np.cos(70.0 / 180 * np.pi): # < 70deg, need wind rotor
            gimbal_wind_flag = True

        target_pos_z = userdata.init_grasp_cog_pos[2]
        if not userdata.only_yaw:
            target_pos_z = translation_from_matrix(concatenate_matrices(inverse_matrix(init_valve_pose), translation_matrix(userdata.init_grasp_cog_pos)))[2] # w.r.t. valve coord

        curr_yaw = 0
        curr_robot_rot = self.robot.getBaselinkRot()
        prev_yaw = euler_from_quaternion(quaternion_multiply(quaternion_inverse(init_valve_rot), curr_robot_rot))[2]

        if userdata.reverse_joint: # downward
            rot_dash = quaternion_multiply(curr_robot_rot, quaternion_from_euler(0, np.pi, 0))
            prev_yaw = euler_from_quaternion(quaternion_multiply(quaternion_inverse(init_valve_rot), rot_dash))[2]
        if userdata.only_yaw:
            prev_yaw = euler_from_quaternion(curr_robot_rot)[2]

        prev_target_rot = quaternion_from_euler(0,0,0) # identity

        delta_t = 1 / self.rate
        start_t = rospy.get_time()
        fixed_r = -1
        target_theta = 0

        turn_direction = 0
        if self.action == 'open':
            turn_direction = 1
            # TODO: add check of joint angles
        elif self.action == 'close':
            turn_direction = -1
            # TODO: add check of joint angles
        else:
            rospy.logerror(self.__class__.__name__  + "_" + self.motion + ": no support action of {}, please choose in ['open', 'close']".format(self.action))
            return 'failed'

        est_wrench_offset = userdata.est_wrench_offset
        valve_force = [0,0,0] # [x,y,z] w.r.t world frame
        valve_torque = userdata.init_torque

        self.stable_manipulate = False

        last_winding_t = -1000

        if not self.debug_mode:
            self.robot.addExternalWrench('valve', 'cog', valve_force, valve_torque)

        while True:

            # 0. check whether need to wind
            if gimbal_wind_flag:
                g_vector =  translation_from_matrix(concatenate_matrices(quaternion_matrix(quaternion_inverse(prev_target_rot)),
                                                                         translation_matrix(np.array([0,0,-1]))))

                start_wind = False
                phi = np.arctan2(g_vector[1], g_vector[0])
                delta_angle = np.pi/2 - 0.6 # heuristic parameter
                if phi > delta_angle - 0.05 and phi < delta_angle + 0.05  and self.action == 'open':
                    start_wind = True
                if phi > -delta_angle - 0.05 and phi < -delta_angle + 0.05 and self.action == 'close':
                    start_wind = True

                if start_wind and rospy.get_time() - last_winding_t > np.pi / self.angular_velocity:
                    rospy.loginfo("start winding gimbal1 roll")

                    # hovering
                    self.robot.targetMotion(self.robot.getCogPos(), rot = self.robot.getBaselinkRot())
                    self.robot.clearExternalWrench('valve')

                    rospy.sleep(self.wind_start_delay) # sleep to wait the stable hovering

                    # send wind command for gimbal1_roll (id is 0)
                    self.robot.windGimbal(0)

                    rospy.sleep(self.wind_duration) # sleep to wait the stable hovering
                    last_winding_t = rospy.get_time()

                    self.stable_manipulate = False


            # 1. rotation and angular velocity
            # Note: yaw is w.r.t. the valve coord (i.e., around the z axis of valve coord)
            curr_robot_rot = self.robot.getBaselinkRot()
            curr_yaw = euler_from_quaternion(quaternion_multiply(quaternion_inverse(init_valve_rot), curr_robot_rot))[2]
            if userdata.only_yaw:
                curr_yaw = euler_from_quaternion(curr_robot_rot)[2]
            else:
                if userdata.reverse_joint: # downward
                    curr_robot_rot_dash = quaternion_multiply(curr_robot_rot, quaternion_from_euler(0, np.pi, 0))
                    curr_yaw = euler_from_quaternion(quaternion_multiply(quaternion_inverse(init_valve_rot), curr_robot_rot_dash))[2]


            delta = curr_yaw - prev_yaw
            if userdata.reverse_joint and userdata.only_yaw: # downward
                delta = - delta
            if delta > np.pi:
                delta -= np.pi * 2
            if delta < -np.pi:
                delta += np.pi * 2
            sum_turn_angle += turn_direction * delta

            if sum_turn_angle > self.round_num * np.pi * 2:
                rospy.logwarn(self.__class__.__name__  + "_" + self.motion + ": complete valve manipulation")
                break

            if self.robot.getTaskHaltFlag():
                rospy.logwarn(self.__class__.__name__  + "_" + self.motion + ": task is halted")
                break

            turn_vel = turn_direction * self.angular_velocity
            delta_yaw = delta_t * turn_vel
            target_yaw = curr_yaw + delta_yaw # base on current yaw angle
            target_rot = quaternion_about_axis(target_yaw, (0,0,1))

            if not userdata.only_yaw:
                target_rot = quaternion_multiply(init_valve_rot, target_rot)

            if userdata.reverse_joint: # downward
                # turn the z axis 180 deg
                target_rot = quaternion_multiply(target_rot, quaternion_from_euler(0, np.pi, 0))
                if userdata.only_yaw:
                    target_rot = quaternion_about_axis(curr_yaw - delta_yaw, (0,0,1))
                    delta_yaw = -delta_yaw # reverse


            # 2. position and linear velocity

            local_cog = translation_from_matrix(concatenate_matrices(inverse_matrix(init_valve_pose), translation_matrix(self.robot.getCogPos()))) # w.r.t. valve coord
            if userdata.only_yaw:
                local_cog = self.robot.getCogPos() - init_valve_pos # w.r.t word coord

            r = np.linalg.norm(local_cog[:2])
            if self.fixed_traj and self.stable_manipulate:
                r = fixed_r
            target_theta = np.arctan2(local_cog[1], local_cog[0]) + delta_yaw

            rospy.loginfo_throttle(1.0, "radius of manipulation trajectory: {}".format(r))

            target_pos = np.array([r * np.cos(target_theta), r * np.sin(target_theta), target_pos_z]) # w.r.t. valve coord
            target_pos = translation_from_matrix(concatenate_matrices(init_valve_pose,
                                                                      translation_matrix(target_pos)))  # w.r.t. world coord
            if userdata.only_yaw:
                target_pos = np.array([init_valve_pos[0] + r * np.cos(target_theta), init_valve_pos[1] + r * np.sin(target_theta), target_pos_z])

            target_linear_vel = r * turn_vel * np.array([-np.sin(target_theta), np.cos(target_theta), 0])
            target_angular_vel = np.array([0, 0, turn_vel])
            if not userdata.only_yaw:
                target_linear_vel = translation_from_matrix(concatenate_matrices(quaternion_matrix(init_valve_rot),
                                                                                 translation_matrix(target_linear_vel)))
                if userdata.reverse_joint: # downward
                    target_angular_vel = -target_angular_vel # should be w.r.t. baselink frame
            else:
                if userdata.reverse_joint: # downward
                    target_angular_vel = -target_angular_vel
                    target_linear_vel = -target_linear_vel

            self.robot.targetMotion(target_pos, rot = target_rot, linear_vel = target_linear_vel, angular_vel = target_angular_vel)


            # 3. consider the centripetal force
            vel = translation_from_matrix(concatenate_matrices(quaternion_matrix(quaternion_inverse(init_valve_rot)),
                                                                      translation_matrix(self.robot.getCogLinearVel())))
            if userdata.only_yaw:
                vel = self.robot.getCogLinearVel()
            vel[2] = 0
            valve_force = self.robot.getMass() * (np.linalg.norm(vel) **2) / r * np.array([-np.cos(target_theta), -np.sin(target_theta), 0]) # TODO: LPF
            if not userdata.only_yaw:
                valve_force = translation_from_matrix(concatenate_matrices(quaternion_matrix(init_valve_rot),
                                                                           translation_matrix(valve_force)))

            # 4.1 adjust torquce according to roll moment
            # Note: the external wrench estimation is not correct when contact with valve,
            # since the rotational motion is not based on a free rigid body (roll and pitch are independent)
            control_pid = self.robot.getControlPid() # w.r.t. CoG frame
            moment = np.array([control_pid.roll.p_term[0] + control_pid.roll.i_term[0],
                               control_pid.pitch.p_term[0] + control_pid.pitch.i_term[0],
                               control_pid.yaw.p_term[0] + control_pid.yaw.i_term[0]])
            roll_moment = translation_from_matrix(concatenate_matrices(quaternion_matrix(quaternion_inverse(prev_target_rot)),
                                                                       translation_matrix(moment)))[0]
            # we assume the roll axis (i.e., x axis) of baselink is along the moment arm. If not, we have to adjust this part
            if np.abs(roll_moment) < self.roll_moment_thresh:
                roll_moment = 0
            adjust_torque = roll_moment * self.torque_adjust_roll_k * delta_t
            if userdata.only_yaw:
                if userdata.reverse_joint: # downward
                    adjust_torque = -adjust_torque
                valve_torque[2] += adjust_torque
            else:
                valve_torque += translation_from_matrix(concatenate_matrices(quaternion_matrix(init_valve_rot),
                                                                             translation_matrix(np.array([0, 0, adjust_torque]))))

            # 4.2 adjust torquce according to yaw velocity
            curr_vel = translation_from_matrix(concatenate_matrices(quaternion_matrix(quaternion_inverse(init_valve_rot)),
                                                                    translation_matrix(self.robot.getCogAngularVel())))[2]
            delta_vel = turn_vel - curr_vel
            if np.abs(curr_vel) > 0.8 * np.abs(turn_vel) and not self.stable_manipulate:
                rospy.loginfo("the manipulation is stable")
                self.stable_manipulate = True
                fixed_r = np.linalg.norm(local_cog[:2])


            if np.abs(delta_vel) < self.yaw_velocity_thresh:
                delta_vel = 0
            adjust_torque = delta_vel * self.torque_adjust_yaw_k * delta_t
            if userdata.only_yaw:
                if userdata.reverse_joint: # downward
                    adjust_torque = -adjust_torque
                valve_torque[2] += adjust_torque
            else:
                valve_torque += translation_from_matrix(concatenate_matrices(quaternion_matrix(init_valve_rot),
                                                                             translation_matrix(np.array([0, 0, adjust_torque]))))

            # check the limit of torque
            if np.abs(np.linalg.norm(valve_torque)) > self.torque_limit and not self.debug_mode:
                rospy.logwarn(self.__class__.__name__  + "_" + self.motion + ": reach the limit of torque {}, finish manipulation".format(valve_torque))
                break

            # TODO:please also add the divergence of roll pitch to check the close reaction
            # if self.stable_manipulate and np.abs(curr_vel) < 0.3 * np.abs(turn_vel):
            #     rospy.logwarn(self.__class__.__name__  + "_" + self.motion + ": reach the stuck, the turning velocity {}, finish manipulation".format(curr_vel))
            #     break


            if not self.debug_mode:
                self.robot.addExternalWrench('valve', 'cog', valve_force, valve_torque)

            prev_yaw = curr_yaw
            prev_target_rot = target_rot
            rospy.sleep(delta_t)

        # relax the final waiting position
        target_rot = self.robot.getBaselinkRot()
        if userdata.only_yaw:
            target_rot = quaternion_from_euler(0, 0, self.robot.getBaselinkRPY()[2])

        self.robot.targetMotion(self.robot.getCogPos(), rot = target_rot)

        if not self.debug_mode:
            self.robot.clearExternalWrench('valve')
        rospy.sleep(2.0) # for final convergence to the end pose
        return "succeeded"


class Finish(BaseState):
    def __init__(self, robot, status = 'success'):
        BaseState.__init__(self, robot, outcomes=['preempted'],
                           input_keys=['init_cog_pos', 'approach_cog_pos', 'only_yaw'])
        self.status = status
        self.tip_z_offset = rospy.get_param('~finish/tip_z_offset', 0.15)

    def execute(self, userdata):

        self.robot.resetTaskHaltFlag() # reset task halt, since we are finishing the task

        if self.status == 'success':
            rospy.loginfo(self.__class__.__name__ + '_' + self.status + ': leave girpper from valve')

            valve_pose = ros_np.numpify(self.robot.getValvePose())
            target_pos = translation_from_matrix(concatenate_matrices(inverse_matrix(valve_pose),
                                                                      translation_matrix(self.robot.getCogPos())))
            target_pos[2] += self.tip_z_offset
            target_pos = translation_from_matrix(concatenate_matrices(valve_pose, translation_matrix(target_pos)))



            self.robot.goPoseWaitConvergence(target_pos, rot = None, pos_thresh = 0.1, vel_thresh = 0.1)
            rospy.sleep(2.0) # for final convergence to the end pose


        # joint angle
        roll = euler_from_quaternion(self.robot.getBaselinkRot())[0]
        pitch = euler_from_quaternion(self.robot.getBaselinkRot())[1]
        if np.abs(pitch) < np.pi/4 and np.abs(roll) < np.pi/4: 
            joint_state = JointState()
            joint_state.name = ['joint1_pitch']
            joint_state.position = [0]
            self.robot.setJointAngle(joint_state)



        rospy.loginfo_throttle(0.5, self.__class__.__name__ + '_' + self.status + ': back to home')
        self.robot.goPoseWaitConvergence(userdata.init_cog_pos, rot = None, pos_thresh = 0.2, vel_thresh = 0.2)
        rospy.loginfo(self.__class__.__name__ + '_' + self.status + ': landing')
        self.robot.land()
        return 'preempted'


def main():
    rospy.init_node('dragon_valve_motion')

    only_yaw = rospy.get_param('~only_yaw', False)
    debug_view = rospy.get_param('~debug_view', True)
    debug_mode = rospy.get_param('~debug_mode', True)

    robot = DragonInterface(debug_view)

    sm_top = smach.StateMachine(outcomes=['preempted'])
    sm_top.userdata.init_cog_pos = None
    sm_top.userdata.approach_cog_pos = None
    sm_top.userdata.approach_cog_rot = None
    sm_top.userdata.grasp_cog_pos = None
    sm_top.userdata.grasp_cog_rot = None
    sm_top.userdata.init_torque = [0, 0, 0]
    sm_top.userdata.est_wrench_offset = None
    sm_top.userdata.only_yaw = only_yaw
    sm_top.userdata.reverse_joint = False


    with sm_top:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'Approach',
                                            'preempted':'preempted'},
                               remapping={'init_cog_pos':'init_cog_pos'})

        sm_approach = smach.StateMachine(outcomes=['succeeded', 'failed'],
                                         input_keys=['approach_cog_pos', 'approach_cog_rot',
                                                      'grasp_cog_pos', 'grasp_cog_rot',
                                                     'est_wrench_offset',
                                                     'reverse_joint',
                                                     'only_yaw'],
                                         output_keys=['approach_cog_pos', 'approach_cog_rot',
                                                      'grasp_cog_pos', 'grasp_cog_rot',
                                                      'est_wrench_offset',
                                                      'reverse_joint',])

        with sm_approach:

            smach.StateMachine.add('Pose', Pose(robot),
                                   transitions={'succeeded':'RoughApproach',
                                                'failed':'failed'},
                                   remapping={'reverse_joint':'reverse_joint'})

            smach.StateMachine.add('RoughApproach', Approach(robot, motion = 'approach'),
                                   transitions={'succeeded':'Adjust',
                                                'failed':'failed'},
                                   remapping={'target_cog_pos':'approach_cog_pos',
                                              'target_cog_rot':'approach_cog_rot',
                                              'est_wrench_offset':'est_wrench_offset',
                                              'only_yaw':'only_yaw'})

            smach.StateMachine.add('Adjust', Approach(robot, motion = 'adjust'),
                                   transitions={'succeeded':'succeeded',
                                                'failed':'failed'},
                                   remapping={'target_cog_pos':'grasp_cog_pos',
                                              'target_cog_rot':'grasp_cog_rot',
                                              'est_wrench_offset':'est_wrench_offset',
                                              'only_yaw':'only_yaw'})

        smach.StateMachine.add('Approach', sm_approach,
                               transitions={'succeeded': 'Contact' if not debug_mode else 'Manipulate',
                                            'failed':'Fail'},
                               remapping={'approach_cog_pos':'approach_cog_pos',
                                          'approach_cog_rot':'approach_cog_rot',
                                          'grasp_cog_pos':'grasp_cog_pos',
                                          'grasp_cog_rot':'grasp_cog_rot',
                                          'est_wrench_offset':'est_wrench_offset',
                                          'only_yaw':'only_yaw'})

        smach.StateMachine.add('Contact', Contact(robot),
                               transitions={'succeeded':'Manipulate',
                                            'failed':'Fail'},
                               remapping={'target_cog_pos':'grasp_cog_pos',
                                          'target_cog_rot':'grasp_cog_rot',
                                          'init_torque':'init_torque',
                                          'reverse_joint':'reverse_joint',
                                          'only_yaw':'only_yaw'})

        smach.StateMachine.add('Manipulate', Manipulate(robot),
                               transitions={'succeeded':'Success',
                                            'failed':'Fail'},
                               remapping={'init_grasp_cog_pos':'grasp_cog_pos',
                                          'init_grasp_cog_rot':'grasp_cog_rot',
                                          'init_torque':'init_torque',
                                          'est_wrench_offset':'est_wrench_offset',
                                          'reverse_joint':'reverse_joint',
                                          'only_yaw':'only_yaw'})

        smach.StateMachine.add('Success', Finish(robot, 'success'),
                               transitions={'preempted':'preempted'},
                               remapping={'init_cog_pos':'init_cog_pos',
                                          'approach_cog_pos':'approach_cog_pos',
                                          'only_yaw':'only_yaw'})


        smach.StateMachine.add('Fail', Finish(robot, 'fail'),
                               transitions={'preempted':'preempted'},
                               remapping={'init_cog_pos':'init_cog_pos',
                                          'approach_cog_pos':'approach_cog_pos',
                                          'only_yaw':'only_yaw'})


        sis = smach_ros.IntrospectionServer('task_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
