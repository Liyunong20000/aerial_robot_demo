#!/bin/bash

rosbag record /hydrus/channel_center/pos /hydrus/joint_states /hydrus/joints_ctrl /hydrus/place/channel_center/pos /hydrus/task2_motion/apporach/object/global_pos /hydrus/task2_motion/apporach/object/local_pos /hydrus/task2_motion/grasp/object/global_pos /hydrus/task2_motion/grasp/object/local_pos /hydrus/task2_motion/target_pose /hydrus/uav/cog/odom /hydrus/uav/baselink/odom /hydrus/ungrasp/channel_center/pos /hydrus/debug/pose/pid /hydrus/servo/states
