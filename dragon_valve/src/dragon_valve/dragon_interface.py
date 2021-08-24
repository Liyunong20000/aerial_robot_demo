import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav, PoseControlPid
import numpy as np
import ros_numpy as ros_np
from tf.transformations import *
from std_msgs.msg import Empty, Int8
import math
from std_msgs.msg import UInt8
from jsk_rviz_plugins.msg import OverlayText
from std_srvs.srv import SetBool, SetBoolRequest
import tf2_ros
from geometry_msgs.msg import PoseStamped, Wrench, Vector3, WrenchStamped, Quaternion, QuaternionStamped
from sensor_msgs.msg import Joy
from gazebo_msgs.srv import ApplyBodyWrenchRequest, BodyRequest

class DragonInterface(object):
    def __init__(self, debug_view = False):

        # flight states:
        self.ARM_OFF_STATE = 0
        self.START_STATE = 1
        self.ARM_ON_STATE = 2
        self.TAKEOFF_STATE = 3
        self.LAND_STATE = 4
        self.HOVER_STATE = 5
        self.STOP_STATE = 6


        self.debug_view_ = debug_view
        self.joint_state_ = JointState()
        self.cog_odom_ = Odometry()
        self.baselink_odom_ = Odometry()
        self.flight_state_ = self.ARM_OFF_STATE
        self.target_pos_ = np.array([0,0,0])

        self.robot_name = rospy.get_param('~robot_name', 'dragon')
        self.mass = rospy.get_param('~robot_mass', 7.3) # TODO: get from robot model
        self.default_pos_thresh = rospy.get_param('~default_pos_thresh', 0.1) # m
        self.default_rot_thresh = rospy.get_param('~default_rot_thresh', 0.05) # rad
        self.joint_state_sub_ = rospy.Subscriber('joint_states', JointState, self.jointStateCallback)
        self.joint_ctrl_pub_ = rospy.Publisher('joints_ctrl', JointState, queue_size = 1)
        self.cog_odom_sub_ = rospy.Subscriber('uav/cog/odom', Odometry, self.cogOdomCallback)
        self.baselink_odom_sub_ = rospy.Subscriber('uav/baselink/odom', Odometry, self.baselinkOdomCallback)
        self.nav_pub_ = rospy.Publisher('uav/nav', FlightNav, queue_size = 1)
        self.rotation_pub_ = rospy.Publisher('target_rotation_motion', Odometry, queue_size = 1)
        self.final_rotation_pub_ = rospy.Publisher('final_target_baselink_rot', QuaternionStamped, queue_size = 1) # TODO: change the name
        self.start_pub_ = rospy.Publisher('teleop_command/start', Empty, queue_size = 1)
        self.takeoff_pub_ = rospy.Publisher('teleop_command/takeoff', Empty, queue_size = 1)
        self.land_pub_ = rospy.Publisher('teleop_command/land', Empty, queue_size = 1)
        self.force_landing_pub_ = rospy.Publisher('teleop_command/force_landing', Empty, queue_size = 1)
        self.halt_pub_ = rospy.Publisher('teleop_command/halt', Empty, queue_size = 1)
        self.flight_state_sub_ = rospy.Subscriber('flight_state', UInt8, self.flightStateCallback)
        self.est_wrench_sub_ = rospy.Subscriber('estimated_external_wrench', WrenchStamped, self.estimatedExternalWrenchCallback)
        self.control_pid_sub_ = rospy.Subscriber('debug/pose/pid', PoseControlPid, self.controlPidCallback)
        self.set_joint_torque_client_ = rospy.ServiceProxy('joints/torque_enable', SetBool)

        self.add_wrench_pub = rospy.Publisher('apply_external_wrench', ApplyBodyWrenchRequest, queue_size = 1)
        self.gimbal_wind_pub = rospy.Publisher('wind_gimbal', Int8, queue_size = 1)
        self.inactivate_rotor_pub = rospy.Publisher('inactive_rotor', Int8, queue_size = 1)

        if self.debug_view_:
            self.nav_debug_pub_ = rospy.Publisher('~nav_debug', OverlayText, queue_size = 1)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TODO: move following implementations to inherited taks-oriented class
        self.valve_pose_ = None
        self.valve_pose_topic_name = rospy.get_param('~valve_pose_topic', 'valve_pose')
        self.valve_pose_sub = rospy.Subscriber(self.valve_pose_topic_name, PoseStamped, self.valvePoseCallback)

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback)
        self.prev_joy_state = Joy()
        self.halt_task_ = False
        self.force_skip_ = False

    def jointStateCallback(self, msg):
        self.joint_state_ = msg

    def cogOdomCallback(self, msg):
        self.cog_odom_ = msg

    def baselinkOdomCallback(self, msg):
        self.baselink_odom_ = msg

    def flightStateCallback(self, msg):
        self.flight_state_ = msg.data

    def getRobotName(self):
        return self.robot_name

    def setJointTorque(self, state):
        req = SetBoolRequest()
        req.data = state
        try:
            self.set_joint_torque_client_(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def setJointAngle(self, target_joint_state):
        self.joint_ctrl_pub_.publish(target_joint_state)

    def setXYPosOffset(self, xy_pos_offset_):
        self.xy_pos_offset_ = xy_pos_offset_

    def getJointState(self):
        return self.joint_state_

    def start(self, sleep = 1.0):
        self.start_pub_.publish()
        rospy.sleep(sleep)

    def takeoff(self):
        self.takeoff_pub_.publish()

    def land(self):
        self.land_pub_.publish()

    def forceLanding(self):
        self.force_landing_pub_.publish()

    def halt(self):
        self.halt_pub_.publish()

    def getBaselinkOdom(self):
        return self.baselink_odom_

    def getCogOdom(self):
        return self.cog_odom_

    def getBaselinkPos(self):
        return ros_np.numpify(self.baselink_odom_.pose.pose.position)

    def getBaselinkRot(self):
        return ros_np.numpify(self.baselink_odom_.pose.pose.orientation)

    def getBaselinkRPY(self):
        return euler_from_quaternion(ros_np.numpify(self.baselink_odom_.pose.pose.orientation))

    def getBaselinkLinearVel(self):
        return ros_np.numpify(self.baselink_odom_.twist.twist.linear)

    def getBaselinkAngularVel(self):
        return ros_np.numpify(self.baselink_odom_.twist.twist.angular)

    def getCogPos(self):
        return ros_np.numpify(self.cog_odom_.pose.pose.position)

    def getCogRot(self):
        return ros_np.numpify(self.cog_odom_.pose.pose.orientation)

    def getCogRPY(self):
        return euler_from_quaternion(ros_np.numpify(self.cog_odom_.pose.pose.orientation))

    def getCogLinearVel(self):
        return ros_np.numpify(self.cog_odom_.twist.twist.linear)

    def getCogAngularVel(self):
        return ros_np.numpify(self.cog_odom_.twist.twist.angular)

    def getFlightState(self):
        return self.flight_state_

    def getTargetPos(self):
        return self.target_pos_

    def targetMotion(self, pos, rot = None, linear_vel = None, angular_vel = None):

        nav_msg = FlightNav()
        nav_msg.control_frame = nav_msg.WORLD_FRAME

        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.target = FlightNav.COG
        mode = FlightNav.POS_VEL_MODE

        if linear_vel is None:
            mode = FlightNav.POS_MODE
            linear_vel = np.array([0, 0, 0])

        nav_msg.pos_xy_nav_mode = mode
        nav_msg.pos_z_nav_mode = mode
        nav_msg.target_pos_x = pos[0]
        nav_msg.target_pos_y = pos[1]
        nav_msg.target_pos_z = pos[2]
        nav_msg.target_vel_x = linear_vel[0]
        nav_msg.target_vel_y = linear_vel[1]
        nav_msg.target_vel_z = linear_vel[2]
        self.nav_pub_.publish(nav_msg)

        self.target_pos_ = pos

        if rot is not None:
            if angular_vel is None:
                rotation_msg = QuaternionStamped()
                rotation_msg.header.stamp = nav_msg.header.stamp
                rotation_msg.quaternion = ros_np.msgify(Quaternion, rot)
                self.final_rotation_pub_.publish(rotation_msg)
            else:
                rotation_msg = Odometry()
                rotation_msg.header.stamp = nav_msg.header.stamp
                rotation_msg.header.frame_id = "baselink"
                rotation_msg.pose.pose.orientation = ros_np.msgify(Quaternion, rot)
                rotation_msg.twist.twist.angular = ros_np.msgify(Vector3, np.array(angular_vel))
                self.rotation_pub_.publish(rotation_msg)

    # TODO: extend to SE(3)
    def goPoseWaitConvergence(self, pos, rot, pos_thresh = 0.1, vel_thresh = 0.1, rot_thresh = 0.1, timeout = 30, check_func = None):

        self.targetMotion(pos, rot = rot)
        start_time = rospy.get_time()

        if check_func is None:
            check_func = self.posYawConvergenceCheck

        while not check_func(pos, rot, pos_thresh, vel_thresh, rot_thresh):
            elapsed_time = rospy.get_time() - start_time
            if elapsed_time > timeout and timeout > 0:
                return False

            # TODO: use decorator @ to wrap these functions
            if self.force_skip_:
                rospy.logwarn("Force skip go pos convergence check")
                force_skip_ = False
                return True

            if self.halt_task_:
                rospy.logwarn("Halt the task")
                return False

            if rospy.is_shutdown():
                return False

            rospy.sleep(0.1)

        return True

    def posYawConvergenceCheck(self, target_pos, target_rot, pos_thresh, vel_thresh, rot_thresh):

        if isinstance(pos_thresh, list):
            if len(pos_thresh) != 3:
                rospy.logerr_thpostle(1.0, "wrong number of pos_thresh: {}, should be 3".format(pos_thresh))
                pos_thresh = self.default_pos_thresh
            else:
                pos_thresh = pos_thresh[2]

        if isinstance(rot_thresh, list):
            if len(rot_thresh) != 3:
                rospy.logerr_throttle(1.0, "wrong number of rot_thresh: {}, should be 3".format(rot_thresh))
                rot_thresh = self.default_rot_thresh
            else:
                rot_thresh = rot_thresh[2]

        if target_rot is None:
            target_yaw = self.getBaselinkRPY()[2]
        else:
            target_yaw = euler_from_quaternion(target_rot)[2]

        current_yaw = self.getBaselinkRPY()[2]
        current_vel = self.getCogLinearVel()

        delta_pos = target_pos - self.getCogPos()

        delta_yaw = target_yaw - current_yaw
        if delta_yaw > np.pi:
            delta_yaw -= np.pi * 2
        elif delta_yaw < -np.pi:
            delta_yaw += np.pi * 2

        if self.debug_view_:
            text = OverlayText()
            text.width = 400
            text.height = 200
            text.left = 10
            text.top = 10
            text.text_size = 12
            text.line_width = 2
            text.font = "DejaVu Sans Mono"
            text.text = 'CoG Diff\n  pos: {:.4g}, yaw: {:.4g}, vel: {:.4g}\n SetPoint\n  x: {:.4g} y: {:.4g} z: {:.4g} yaw: {:.4g}\n CurrentPoint\n  x: {:.4g} y: {:.4g} z: {:.4g} yaw: {:.4g}'.format(np.linalg.norm(delta_pos), abs(delta_yaw), np.linalg.norm(current_vel), target_pos[0], target_pos[1], target_pos[2], target_yaw, self.getCogPos()[0], self.getCogPos()[1], self.getCogPos()[2], current_yaw)

            rospy.loginfo_throttle(0.5, "\n" + text.text)
            self.nav_debug_pub_.publish(text)

        if np.linalg.norm(delta_pos) < pos_thresh and abs(delta_yaw) < rot_thresh and np.linalg.norm(current_vel) < vel_thresh:
            return True
        else:
            return False

    def getTF(self, frame_id, wait=0.5, parent_frame_id='world'):
        trans = self.tf_buffer.lookup_transform(parent_frame_id, frame_id, rospy.Time.now(), rospy.Duration(wait))
        return trans

    def valvePoseCallback(self, msg):
        self.valve_pose_ = msg.pose

        rospy.loginfo_once("get pose of valve")

    def getValvePose(self):
        return self.valve_pose_

    def getTaskHaltFlag(self):
        return self.halt_task_

    def resetTaskHaltFlag(self):
        self.halt_task_ = False

    def getForceSkipFlag(self):
        return self.force_skip_

    def resetForceSkipFlag(self):
        self.force_skip_ = False

    def joyCallback(self, msg):
        if (msg.buttons[4] == 1) and (self.prev_joy_state.buttons[4] == 0): #L1
            self.halt_task_ = True
            rospy.loginfo('Halt Task!!')

        if (msg.buttons[5] == 1) and (self.prev_joy_state.buttons[5] == 0): #R1
            rospy.loginfo('Force skip while function')
            # force shift to next state
            self.force_skip_ = True

        self.prev_joy_state = msg


    def addExternalWrench(self, wrench_name, reference_frame, force, torque):

        wrench = Wrench(force = Vector3(*force), torque = Vector3(*torque))
        msg = ApplyBodyWrenchRequest(body_name = wrench_name, reference_frame = reference_frame, wrench = wrench)
        self.add_wrench_pub.publish(msg)

    def clearExternalWrench(self, wrench_name):

        try:
            clear_external_wrench = rospy.ServiceProxy('clear_external_wrench', BodyRequest)
            resp = clear_external_wrench(body_name = wrench_name)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def estimatedExternalWrenchCallback(self, msg):
        self.est_wrench = msg.wrench


    def getEstimatedWrench(self):
        return self.est_wrench

    def getMass(self):
        return self.mass

    def controlPidCallback(self, msg):
        self.control_pid = msg

    def getControlPid(self):
        return self.control_pid

    def windGimbal(self, gimbal):
        msg = Int8()
        msg.data = gimbal
        self.gimbal_wind_pub.publish(msg)

    def inactivateRotor(self, rotor):
        msg = Int8()
        msg.data = rotor

        self.inactivate_rotor_pub.publish(msg)
