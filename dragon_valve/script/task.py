#!/usr/bin/env python

import rospy
import smach
import smach_ros
from dragon_valve.dragon_interface import DragonInterface
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Transform, Inertia, PoseArray, PoseStamped, Wrench
import tf.transformations as tft
import ros_numpy as ros_np
from std_msgs.msg import UInt8, Empty
import copy
import std_srvs.srv
from tf.transformations import *
from sensor_msgs.msg import Joy
import tf2_ros


class BaseState(smach.State):
    def __init__(self, robot, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.robot = robot

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
        userdata.init_cog_yaw = self.robot.getCogRPY()[2]

        # check the valve msg
        if self.robot.getValvePose() is None:
            rospy.logwarn(self.__class__.__name__ + ': no valve pose message received')
            return 'preempted'

        return 'succeeded'

class Joint(BaseState):
    def __init__(self, robot,
                 outcomes=['succeeded', 'failed']):
        BaseState.__init__(self, robot, outcomes=outcomes, input_keys=[], output_keys=[], io_keys=[])
        self.joint_conv_thresh = rospy.get_param('~joint/conv_thresh', 0.017)

    def execute(self, userdata):

        # joint angle
        joint_state = JointState()
        joint_state.name = ['joint1_pitch']
        valve_coord_z_axis = tft.quaternion_matrix(ros_np.numpify(self.robot.getValvePose().orientation))[:3, 2]
        if valve_coord_z_axis[2] >= 0: # upward
            joint_state.position = [np.pi / 2]
        else:  # downward
            joint_state.position = [-np.pi / 2]
        self.robot.setJointAngle(joint_state)


        while not self.robot.getForceSkipFlag():

            joint_angle = self.robot.getJointState().position[self.robot.getJointState().name.index('joint1_pitch')]
            if np.fabs(joint_angle - joint_state.position[0]) < self.joint_conv_thresh:
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
                 io_keys=['target_cog_pos', 'target_cog_yaw', 'est_wrench_offset']):
        BaseState.__init__(self, robot, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)

        self.motion = motion
        self.valve_type = rospy.get_param('~valve_type', '3bridges')
        self.valve_head_offset = np.pi / int(self.valve_type[0]) / 2 # to set the end effector tips to the center of the vallve openning space.
        if self.valve_type == '3bridges':
            self.valve_head_offset = np.pi / 2  # special case

        robot_name = self.robot.getRobotName()
        self.end_effector_name = robot_name + '/' + rospy.get_param('~end_effector_name', 'gripper')
        self.baselink_name = robot_name + '/' + rospy.get_param('~baselink_name', 'baselink')
        self.cog_name = robot_name + '/' + rospy.get_param('~cog_name', 'cog')

        self.tip_offset = rospy.get_param('~' + motion + '/tip_offset', [0, 0, 0])
        self.pos_conv_thresh = rospy.get_param('~' + motion + '/pos_conv_thresh', 0.06)
        self.yaw_conv_thresh = rospy.get_param('~' + motion + '/yaw_conv_thresh', 0.1)
        self.att_conv_thresh = rospy.get_param('~' + motion + '/att_conv_thresh', 0.06)

    # TODO: end effector SE(3) pose, and joint angle according to valve orientation
    def calculateTargetCogPose(self, tip_offset = [0, 0, 0]):

        valve_pose = self.robot.getValvePose()
        target_end_effector_trans = tft.concatenate_matrices(ros_np.numpify(valve_pose),  # base
                                                             tft.translation_matrix(tip_offset),  # origin offset
                                                             tft.euler_matrix(0, 0, self.valve_head_offset),  # yaw offset
                                                             tft.euler_matrix(0, np.pi/2, 0))  # gripper orientation regarding valve


        cog_trans = self.robot.getTF(self.cog_name, parent_frame_id= self.end_effector_name)

        target_cog_trans = tft.concatenate_matrices(target_end_effector_trans, \
                                                    ros_np.numpify(cog_trans.transform))

        target_cog_pos = tft.translation_from_matrix(target_cog_trans)
        target_cog_yaw = tft.euler_from_matrix(target_cog_trans)[2]

        return target_cog_pos, target_cog_yaw

    def execute(self, userdata):

        target_cog_pos, target_cog_yaw = self.calculateTargetCogPose(self.tip_offset)

        # TODO: change to SE(3)
        conv_flag = self.robot.goPoseWaitConvergence(target_cog_pos, target_cog_yaw, pos_conv_thresh = self.pos_conv_thresh, yaw_conv_thresh = self.yaw_conv_thresh, timeout = 30)

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

        userdata.target_cog_pos = target_cog_pos
        userdata.target_cog_yaw = target_cog_yaw

        return 'succeeded'

class Contact(Approach):
    def __init__(self, robot):
        Approach.__init__(self, robot, motion = 'contact',
                          input_keys=[],
                          output_keys=[], io_keys=['target_cog_pos', 'target_cog_yaw', 'init_torque'])

        self.rate = rospy.get_param('~contact/rate', 10.0) # hz
        self.angular_velocity = rospy.get_param('~contact/angular_velocity', 0.2) # rad/s
        self.init_torque = rospy.get_param('~contact/init_torque', 0.1) # Nm
        self.incre_torque_rate = rospy.get_param('~contact/incre_torque_rate', 1) # Nm/s
        self.move_thresh = rospy.get_param('~contact/move_thresh', 0.1) # valve or robot yaw angle move thresh
        self.check_interval = rospy.get_param('~contact/check_interval', 1.0) # s
        self.action = rospy.get_param('~manipulate/action', 'open')


    def execute(self, userdata):

        rospy.sleep(3.0) # workaround to wait for the convergence of approach adjust.

        init_valve_yaw = tft.euler_from_quaternion(ros_np.numpify(self.robot.getValvePose().orientation))[2] # TODO: SE(3)
        prev_yaw = self.robot.getCogRPY()[2]
        delta_t = 1 / self.rate
        start_t = rospy.get_time()

        valve_pos = ros_np.numpify(self.robot.getValvePose().position)
        valve_coord_z_axis = tft.quaternion_matrix(ros_np.numpify(self.robot.getValvePose().orientation))[:3, 2]
        turn_direction = 0
        if self.action == 'open':
            turn_direction = valve_coord_z_axis[2] / np.fabs(valve_coord_z_axis[2])
        elif self.action == 'close':
            turn_direction = -valve_coord_z_axis[2] / np.fabs(valve_coord_z_axis[2])
        else:
            rospy.logerror(self.__class__.__name__  + "_" + self.motion + ": no support action of {}, please choose in ['open', 'close']".format(self.action))
            return 'failed'

        contact = False
        prev_contact_t = rospy.get_time() + 2.0 # workaround

        while True:

            curr_yaw = self.robot.getCogRPY()[2]

            # provide yaw motion to contact
            # if contact:
            #     target_yaw = curr_yaw
            #     self.robot.goYawVel(np.array([0,0,0]), target_yaw)
            # else:
            #     target_vel_yaw = turn_direction * self.angular_velocity
            #     target_yaw = curr_yaw + delta_t * target_vel_yaw
            #     self.robot.goYawVel(np.array([0,0,0]), target_yaw)

            target_vel_yaw = turn_direction * self.angular_velocity
            delta_yaw = delta_t * target_vel_yaw
            target_yaw = curr_yaw + delta_yaw # base on current yaw angle

            target_pos = userdata.target_cog_pos
            local_cog = self.robot.getCogPos()[:2] - valve_pos[:2]

            r = np.linalg.norm(local_cog)
            target_theta = np.arctan2(local_cog[1], local_cog[0]) + delta_yaw

            target_pos[:2] = valve_pos[:2] + r * np.array([np.cos(target_theta), np.sin(target_theta)])
            target_vel = r * target_vel_yaw * np.array([-np.sin(target_theta), np.cos(target_theta), 0]) # TODO: SE(3)

            self.robot.targetMotion(target_pos, target_yaw = target_yaw, target_vel = target_vel, target_vel_yaw = target_vel_yaw)
            #self.robot.goYawVel(target_vel, curr_yaw, target_vel_yaw)


            # add external wrench which gradually increases
            valve_force = [0,0,0] # [x,y,z] w.r.t world frame
            valve_torque = [0,0, self.init_torque * turn_direction] # workaround valve torque
            self.robot.addExternalWrench('valve', 'cog', valve_force, valve_torque)

            # check the contact with valve
            if rospy.get_time() - prev_contact_t > self.check_interval and not contact:

                # check the angle change within the duration of self.check_interval
                if np.abs(curr_yaw - prev_yaw) < self.move_thresh:
                    contact = True
                    rospy.loginfo(self.__class__.__name__  + "_" + self.motion + ": contact with valve handle, torque: {}".format(self.init_torque))

                prev_yaw = curr_yaw
                prev_contact_t = rospy.get_time()


            # check the move of valve
            valve_yaw = tft.euler_from_quaternion(ros_np.numpify(self.robot.getValvePose().orientation))[2]
            if turn_direction * (valve_yaw - init_valve_yaw) > self.move_thresh:
                rospy.loginfo(self.__class__.__name__  + "_" + self.motion + ": valve move with torque of {}".format(self.init_torque))
                userdata.init_torque = valve_torque
                userdata.target_cog_pos = target_pos
                userdata.target_cog_yaw = target_yaw
                return 'succeeded'

            if self.robot.getTaskHaltFlag():
                rospy.logwarn(self.__class__.__name__  + "_" + self.motion + ": taks is halted")
                self.robot.clearExternalWrench('valve')
                self.robot.targetMotion(self.robot.getCogPos(), target_yaw = curr_yaw)
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
                          input_keys=['init_grasp_cog_pos', 'init_grasp_cog_yaw', 'init_torque', 'est_wrench_offset'],
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

    def execute(self, userdata):

        sum_turn_angle = 0
        prev_yaw = self.robot.getCogRPY()[2] # TODO: change to SE(3)
        valve_coord_z_axis = tft.quaternion_matrix(ros_np.numpify(self.robot.getValvePose().orientation))[:3, 2]
        turn_direction = 0
        if self.action == 'open':
            turn_direction = valve_coord_z_axis[2] / np.fabs(valve_coord_z_axis[2])
        elif self.action == 'close':
            turn_direction = -valve_coord_z_axis[2] / np.fabs(valve_coord_z_axis[2])
        else:
            rospy.logerror(self.__class__.__name__  + "_" + self.motion + ": no support action of {}, please choose in ['open', 'close']".format(self.action))
            return 'failed'

        delta_t = 1 / self.rate
        start_t = rospy.get_time()
        fixed_r = -1
        target_theta = 0

        valve_pos = ros_np.numpify(self.robot.getValvePose().position)

        est_wrench_offset = userdata.est_wrench_offset
        valve_force = [0,0,0] # [x,y,z] w.r.t world frame
        valve_torque = userdata.init_torque
        self.robot.addExternalWrench('valve', 'cog', valve_force, valve_torque)

        while True:
            curr_yaw = self.robot.getCogRPY()[2] # TODO: do we need Baselink?

            delta = curr_yaw - prev_yaw
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

            target_vel_yaw = turn_direction * self.angular_velocity
            delta_yaw = delta_t * target_vel_yaw
            target_yaw = curr_yaw + delta_yaw # base on current yaw angle


            target_pos = userdata.init_grasp_cog_pos
            local_cog = self.robot.getCogPos()[:2] - valve_pos[:2]

            if (rospy.get_time() - start_t > np.pi/2 / self.angular_velocity or sum_turn_angle > np.pi/2) and self.fixed_traj: # 5.0 second is an adhoc value
                # method1: use pre-defined circle trajectory
                if fixed_r < 0:
                    fixed_r = np.linalg.norm(local_cog)
                    target_theta = np.arctan2(local_cog[1], local_cog[0])

                r = fixed_r
                target_theta += delta_yaw
            else:
                # method2: use current CoG position to calculate the next target CoG position
                # assumption: the relative transform between robot and valve is constant due to the gripper
                r = np.linalg.norm(local_cog)
                target_theta = np.arctan2(local_cog[1], local_cog[0]) + delta_yaw

            rospy.loginfo_throttle(1.0, "radius of manipulation trajectory: {}".format(np.linalg.norm(local_cog)))
            target_pos[:2] = valve_pos[:2] + r * np.array([np.cos(target_theta), np.sin(target_theta)])
            target_vel = r * target_vel_yaw * np.array([-np.sin(target_theta), np.cos(target_theta), 0]) # TODO: SE(3)

            self.robot.targetMotion(target_pos, target_yaw = target_yaw, target_vel = target_vel, target_vel_yaw = target_vel_yaw)

            # consider the centripetal force
            actual_vel = np.linalg.norm(np.array([self.robot.getCogLinearVel()[0], self.robot.getCogLinearVel()[1], 0])) # TODO: SE(3)
            valve_force = self.robot.getMass() * actual_vel * actual_vel / r * np.array([-np.cos(target_theta), -np.sin(target_theta), 0]) # TODO: SE(3) + LPF

            # adjust torquce according to roll moment
            # TODO: SE(3)
            # Note: the external wrench estimation is not correct when contact with valve, sine the rotational motion is not based on a free rigid body (roll and pitch are independent)
            control_pid = self.robot.getControlPid()
            roll_moment = control_pid.roll.p_term[0] + control_pid.roll.i_term[0]
            if np.abs(roll_moment) < self.roll_moment_thresh:
                roll_moment = 0
            adjust_valve_torque = turn_direction * roll_moment * self.torque_adjust_roll_k * delta_t # TODO: direction of valve
            valve_torque[2] += adjust_valve_torque # TODO: SE(3)

            delta_vel = target_vel_yaw - self.robot.getCogAngularVel()[2]
            if np.abs(delta_vel) < self.yaw_velocity_thresh:
                delta_vel = 0
            adjust_valve_torque = turn_direction * delta_vel * self.torque_adjust_yaw_k * delta_t
            valve_torque[2] += adjust_valve_torque # TODO: SE(3)

            # check the limit of torque
            if np.abs(valve_torque[2]) > self.torque_limit:
                rospy.logwarn(self.__class__.__name__  + "_" + self.motion + ": reach the limit of torque {}, finish manipulation".format(valve_torque[2]))
                break

            self.robot.addExternalWrench('valve', 'cog', valve_force, valve_torque)

            prev_yaw = curr_yaw
            rospy.sleep(delta_t)

        # relax the final waiting position
        self.robot.targetMotion(self.robot.getCogPos(), target_yaw = self.robot.getCogRPY()[2])

        self.robot.clearExternalWrench('valve')

        rospy.sleep(2.0) # for final convergence to the end pose
        return "succeeded"


class Finish(BaseState):
    def __init__(self, robot, status = 'success'):
        BaseState.__init__(self, robot, outcomes=['preempted'],
                           input_keys=['init_cog_pos', 'init_cog_yaw',
                                        'approach_cog_pos', 'approach_cog_yaw'])
        self.status = status
        self.tip_z_offset = rospy.get_param('~finish/tip_z_offset', 0.15)

    def execute(self, userdata):

        self.robot.resetTaskHaltFlag() # reset task halt, since we are finishing the task

        if self.status == 'success':
            rospy.loginfo(self.__class__.__name__ + '_' + self.status + ': leave girpper from valve')
            # TODO: extend to SE(3)
            target_pos = self.robot.getCogPos()
            target_pos[2] += self.tip_z_offset
            self.robot.goPoseWaitConvergence(target_pos, self.robot.getCogRPY()[2], \
                                            pos_conv_thresh = 0.1, yaw_conv_thresh = 0.2)
            rospy.sleep(2.0) # for final convergence to the end pose


        # joint angle
        joint_state = JointState()
        joint_state.name = ['joint1_pitch']
        joint_state.position = [0]
        self.robot.setJointAngle(joint_state)

        rospy.loginfo_throttle(0.5, self.__class__.__name__ + '_' + self.status + ': back to home')
        self.robot.goPoseWaitConvergence(userdata.init_cog_pos, self.robot.getCogRPY()[2], \
                                        pos_conv_thresh = 0.2, yaw_conv_thresh = 0.2)
        rospy.loginfo(self.__class__.__name__ + '_' + self.status + ': landing')
        self.robot.land()
        return 'preempted'


def main():
    rospy.init_node('dragon_valve_motion')
    sm_top = smach.StateMachine(outcomes=['preempted'])
    sm_top.userdata.init_cog_pos = None
    sm_top.userdata.init_cog_yaw = None
    sm_top.userdata.approach_cog_pos = None
    sm_top.userdata.approach_cog_yaw = None
    sm_top.userdata.grasp_cog_pos = None
    sm_top.userdata.grasp_cog_yaw = None
    sm_top.userdata.init_torque = None
    sm_top.userdata.est_wrench_offset = None


    debug_view = rospy.get_param('~debug_view', True)
    robot = DragonInterface(debug_view)


    with sm_top:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'Approach',
                                            'preempted':'preempted'},
                               remapping={'init_cog_pos':'init_cog_pos',
                                          'init_cog_yaw':'init_cog_yaw'})

        sm_approach = smach.StateMachine(outcomes=['succeeded', 'failed'],
                                         input_keys=['approach_cog_pos', 'approach_cog_yaw',
                                                      'grasp_cog_pos', 'grasp_cog_yaw',
                                                     'est_wrench_offset'],
                                         output_keys=['approach_cog_pos', 'approach_cog_yaw',
                                                      'grasp_cog_pos', 'grasp_cog_yaw',
                                                      'est_wrench_offset'])

        with sm_approach:

            smach.StateMachine.add('Joint', Joint(robot),
                                   transitions={'succeeded':'RoughApproach',
                                                'failed':'failed'})

            smach.StateMachine.add('RoughApproach', Approach(robot, motion = 'approach'),
                                   transitions={'succeeded':'Adjust',
                                                'failed':'failed'},
                                   remapping={'target_cog_pos':'approach_cog_pos',
                                              'target_cog_yaw':'approach_cog_yaw',
                                              'est_wrench_offset':'est_wrench_offset'})

            smach.StateMachine.add('Adjust', Approach(robot, motion = 'adjust'),
                                   transitions={'succeeded':'succeeded',
                                                'failed':'failed'},
                                   remapping={'target_cog_pos':'grasp_cog_pos',
                                              'target_cog_yaw':'grasp_cog_yaw',
                                              'est_wrench_offset':'est_wrench_offset'})

        smach.StateMachine.add('Approach', sm_approach,
                               transitions={'succeeded':'Contact',
                                            'failed':'Fail'},
                               remapping={'approach_cog_pos':'approach_cog_pos',
                                          'approach_cog_yaw':'approach_cog_yaw',
                                          'grasp_cog_pos':'grasp_cog_pos',
                                          'grasp_cog_yaw':'grasp_cog_yaw',
                                          'est_wrench_offset':'est_wrench_offset'})

        smach.StateMachine.add('Contact', Contact(robot),
                               transitions={'succeeded':'Manipulate',
                                            'failed':'Fail'},
                               remapping={'target_cog_pos':'grasp_cog_pos',
                                          'target_cog_yaw':'grasp_cog_yaw',
                                          'init_torque':'init_torque'})

        smach.StateMachine.add('Manipulate', Manipulate(robot),
                               transitions={'succeeded':'Success',
                                            'failed':'Fail'},
                               remapping={'init_grasp_cog_pos':'grasp_cog_pos',
                                          'init_grasp_cog_yaw':'grasp_cog_yaw',
                                          'init_torque':'init_torque',
                                          'est_wrench_offset':'est_wrench_offset'})

        smach.StateMachine.add('Success', Finish(robot, 'success'),
                               transitions={'preempted':'preempted'},
                               remapping={'init_cog_pos':'init_cog_pos',
                                          'init_cog_yaw':'init_cog_yaw',
                                          'approach_cog_pos':'approach_cog_pos',
                                          'approach_cog_yaw':'approach_cog_yaw'})


        smach.StateMachine.add('Fail', Finish(robot, 'fail'),
                               transitions={'preempted':'preempted'},
                               remapping={'init_cog_pos':'init_cog_pos',
                                          'init_cog_yaw':'init_cog_yaw',
                                          'approach_cog_pos':'approach_cog_pos',
                                          'approach_cog_yaw':'approach_cog_yaw'})


        sis = smach_ros.IntrospectionServer('task_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
