#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# license removed for brevity


import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#import tf2_ros

from hsr_image_collector.msg import TakePictureRequestMsg
from image_saver import ImageSaver
from handyman.msg import HandymanMsg as HandyManMsg

class Msg(object):

    def __init__(self):
        self.msg_are_you_ready      = "Are_you_ready?";
        self.msg_setting_completed  = "Setting_completed";
        self.msg_score_received     = "Score";
        self.msg_stop               = "Stop_process";

        self.msg_i_am_ready         = "I_am_ready";
        self.msg_images_captured    = "Images_captured";
        self.msg_object_placed      = "Object_placed"
        self.msg_score_saved        = "Score_saved";
        self.msg_stopped            = "Process_stopped";


class PlacingDataCollection(object):

    def __init__(self):

        self.furniture_type = "lower"

        # def step
        self.step = 0
        self.score = 0
        self.camera_height = 0
        self.num_dir = 1
        self.dir_name = "ponNet%s" % self.num_dir
        self.strategy  = 0

        #def max number of images per directory
        self.max_images_dir = 100
        self.img_in_dir = 0

        # def arm joint trajectory
        self.arm_joint_trajectory = JointTrajectory()
        self.arm_joint_names = [ 'arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint' ]
        self.arm_joint_point = JointTrajectoryPoint()
        self.arm_joint_trajectory.joint_names = self.arm_joint_names
        self.arm_joint_trajectory.points.append(self.arm_joint_point)

        # def head joint trajectory
        self.head_joint_trajectory = JointTrajectory()
        self.head_joint_names = [ 'head_tilt_joint', 'head_pan_joint']
        self.head_joint_point = JointTrajectoryPoint()
        self.head_joint_trajectory.joint_names = self.head_joint_names
        self.head_joint_trajectory.points.append(self.head_joint_point)

        # def gripper joint trajectory
        self.gripper_joint_trajectory = JointTrajectory()
        self.gripper_joint_names = ['hand_l_proximal_joint', 'hand_r_proximal_joint']
        self.gripper_joint_point = JointTrajectoryPoint()
        self.gripper_joint_trajectory.joint_names = self.gripper_joint_names
        self.gripper_joint_trajectory.points.append(self.gripper_joint_point)

        # def status flags
        self.start = None
        self.finished = None
        self.failed = None

        # def msgs
        self.msg = Msg()


        # def topic names
        self.sub_msg_to_robot_topic_name = "/handyman/message/to_robot"
        self.pub_msg_to_moderator_topic_name = "/handyman/message/to_moderator"
        self.pub_arm_trajectory_topic_name = "/hsrb/arm_trajectory_controller/command"
        self.pub_gripper_trajectory_topic_name = "/hsrb/gripper_trajectory_controller/command"
        self.pub_head_trajectory_topic_name = "/hsrb/head_trajectory_controller/command"

        # def subscriber
        self.msg_robot_subscriber = rospy.Subscriber(self.sub_msg_to_robot_topic_name, HandyManMsg, self.msg_to_robot_callback)

        # def publisher
        self.msg_publisher = rospy.Publisher(
            self.pub_msg_to_moderator_topic_name, HandyManMsg, queue_size=10)


        self.arm_trj_publisher = rospy.Publisher(
            self.pub_arm_trajectory_topic_name, JointTrajectory, queue_size=10)

        self.gripper_trj_publisher = rospy.Publisher(
            self.pub_gripper_trajectory_topic_name, JointTrajectory, queue_size=10)

        self.head_trj_publisher = rospy.Publisher(
            self.pub_head_trajectory_topic_name, JointTrajectory, queue_size=10)

        self.image_saver = ImageSaver()

        self.output_dir = self.image_saver.set_directory(self.dir_name)

        self.reset()


    def reset(self):
        self.start = False
        self.finished = False
        self.failed = False

        head_pos = [ 0.0, 0.0 ]
        self.head_joint_trajectory.points[0].positions = head_pos

        gripper_pos = [ 0.0, 0.0 ]
        self.gripper_joint_trajectory.points[0].positions = gripper_pos

################ RECEIVE MESSAGE ##################
    def msg_to_robot_callback(self, ros_data):
        # parse ros_data
        message = ros_data.message
        detail = ros_data.detail
        if message == self.msg.msg_are_you_ready:
            if self.step == 0:
                rospy.loginfo('ARE YOU READY?')
                self.start = True
                self.step += 1

        if message == self.msg.msg_setting_completed:
            if self.step == 2:
                meta = detail.split("-")
                if meta[0] == "L":
                    self.furniture_type = "lower"
                else :
                    self.furniture_type = "upper"
                self.camera_height = meta[1]
                self.strategy = meta[2]
                rospy.loginfo('SETTING COMPLETED')
                rospy.loginfo('CAMERA HEIGHT RECEIVED: %s' % self.camera_height)
                rospy.loginfo("STRATEGY RECEIVED %s" % self.strategy)
                self.step += 1

        if message == self.msg.msg_score_received:
            if self.step == 7:
                sc = detail.split("-")
                self.score = sc[0]
                self.target = sc[1]
                rospy.loginfo('SCORE RECEIVED: %s' % self.score)
                rospy.loginfo('Target RECEIVED: %s' % self.target)
                self.step += 1

        if message == self.msg.msg_stop:
            rospy.loginfo('STOPPING')
            self.step = 9

################ SEND MESSAGE ##################
    def send_message(self, msg):
         #rospy.loginfo('Send message: %s' % msg)
         handyman_msg = HandyManMsg()
         handyman_msg.message = msg
         self.msg_publisher.publish(handyman_msg)

############### ROBOT CONTROL ###################                           OK

    def move_arm(self, positions, duration):
        #rospy.loginfo('call move_arm')
        try:
            self.arm_joint_trajectory.points[0].positions = positions
            self.arm_joint_trajectory.points[0].time_from_start = duration
            self.arm_trj_publisher.publish(self.arm_joint_trajectory)
        except Exception as e:
            rospy.logerr('failed to moving arm')
            rospy.logerr(e)
            rospy.logerr(type(e))

    def move_head(self, positions, duration):
        #rospy.loginfo('call move_head')
        try:
            self.head_joint_trajectory.points[0].positions = positions
            self.head_joint_trajectory.points[0].time_from_start = duration
            self.head_trj_publisher.publish(self.head_joint_trajectory)
        except Exception as e:
            rospy.logerr('failed to moving head')
            rospy.logerr(e)
            rospy.logerr(type(e))


    def close_hand(self):
        """
        closing hand
        """

        rospy.loginfo('call close_hand')
        try:
            duration = rospy.Duration()
            duration.secs = 1
            gripper_pos = [-0.01, 0.01]
            self.gripper_joint_trajectory.points[0].positions = gripper_pos
            self.gripper_joint_trajectory.points[0].time_from_start = duration
            self.gripper_trj_publisher.publish(self.gripper_joint_trajectory)
            rospy.sleep(rospy.Duration(1.5))
        except Exception as error:
            rospy.logerr(error)
            rospy.logerr(type(error))

    def open_hand(self):

        rospy.loginfo('call open_hand')
        try:
            duration = rospy.Duration()
            duration.secs = 1
            gripper_pos = [0.611, -0.611]
            self.gripper_joint_trajectory.points[0].positions = gripper_pos
            self.gripper_joint_trajectory.points[0].time_from_start = duration
            self.gripper_trj_publisher.publish(self.gripper_joint_trajectory)
            rospy.sleep(rospy.Duration(0.5))
        except Exception as error:
            rospy.logerr('failed to open hand')
            rospy.logerr(error)
            rospy.logerr(type(error))


    def set_arm_position(self, position):
        if position == "default":
            self.move_arm([0, 0, 0, -1.57, 0], rospy.Duration(1))
            rospy.sleep(rospy.Duration(2))

        elif position == "capturing_lower":
            self.move_arm([0, 0, 1.57, -1.57, 0], rospy.Duration(1))
            rospy.sleep(rospy.Duration(2))
        elif position == "grasping_lower":
            self.close_hand()
            self.move_arm([0.155, -1.57, 0, 0, 0], rospy.Duration(1))
            rospy.sleep(rospy.Duration(2))


        elif position == "capturing_upper":
            self.move_arm([0.25, 0, 1.57, -1.57, 0], rospy.Duration(1))
            rospy.sleep(rospy.Duration(2))
        elif position == "grasping_upper":
            self.close_hand()
            self.move_arm([0.42, -1.57, 0, 0, 0], rospy.Duration(1))
            rospy.sleep(rospy.Duration(2))

        self.ready_for_capture = True


    def set_head_position(self, position):
        if position == "default":
            self.move_head([0, 0], rospy.Duration(1))
        elif position == "capturing_lower":
            self.move_head([-0.55, 0.0], rospy.Duration(1))
        elif position == "capturing_upper":
            self.move_head([-0.35, 0.0], rospy.Duration(1))
        self.set_arm_position(position)

    def set_capturing_position(self):
        self.open_hand()
        if self.furniture_type == "lower":
            self.set_head_position("capturing_lower")
        else:
            self.set_head_position("capturing_upper")


    ###############################################################################

    def capturing(self):
        if self.ready_for_capture:
            if(self.img_in_dir >= self.max_images_dir):
                self.num_dir += 1
                self.dir_name = "ponNet%s" % self.num_dir
                self.output_dir = self.image_saver.set_directory(self.dir_name)
                self.img_in_dir = 0

            self.image_saver.save_img('rgb')
            self.image_saver.save_img('depth')
            self.score_file = open('%s/y-%s.txt' % (self.output_dir, self.num_dir), "a")
            self.metadata_file = open('%s/meta-%s.txt' % (self.output_dir, self.num_dir), "a")
            self.img_in_dir += 1
            rospy.sleep(rospy.Duration(1))
            self.step += 1

    def save_score(self):
        self.score_file.write(self.score + "\n")
        self.score_file.close()
        self.save_metadata()

    def save_metadata(self):
        rospy.loginfo("Saving metadata...")
        self.metadata_file.write(self.camera_height + ":" + self.target + ":" + self.score + ":" + self.strategy +"\n")
        self.metadata_file.close()

    def run(self):
        rospy.init_node('PlacingDataCollection', anonymous=True)
        rospy.loginfo('Placing Data Collection start!')
        r = rospy.Rate(10) # 10hz


        while not rospy.is_shutdown():
            if self.failed:
                rospy.loginfo('Process Failed')
                self.step =0

            # Initialize
            rospy.sleep(1.)
            if self.step == 0:
                print("Waiting to start")

            # Ready
            elif self.step == 1:
                rospy.loginfo('I am ready')
                self.send_message(self.msg.msg_i_am_ready)
                self.step += 1

            # WaitForCompleted
            elif self.step == 2:
                rospy.loginfo("Waiting for setting completed")

            # GoToPosition
            elif self.step == 3:
                self.ready_for_capture = False
                self.set_capturing_position()
                self.step += 1

            # CaptureImages
            elif self.step == 4:
                rospy.loginfo("Capturing images")
                self.capturing()


            # FinishCapturing
            elif self.step == 5:
                rospy.loginfo('Finished capturing images')
                self.send_message(self.msg.msg_images_captured)
                self.step += 1

            #Moving to place object
            elif self.step == 6:
                if self.furniture_type == "lower":
                    self.set_arm_position("grasping_lower")  #For lower furniture
                else:
                    self.set_arm_position("grasping_upper")  #For upper furniture
                self.send_message(self.msg.msg_object_placed)
                self.set_capturing_position()
                self.step += 1

            # WaitForScore
            elif self.step == 7:
                rospy.loginfo("Waiting for score")

            # ScoreSaved
            elif self.step == 8:
                self.save_score()
                rospy.loginfo('Score saved')
                self.send_message(self.msg.msg_score_saved)
                self.step = 0

            # StopProcess
            elif self.step == 9:
                rospy.loginfo('Stopped')
                self.send_message(self.msg.msg_stopped)
                self.step += 1

            # ProcessFinished
            elif self.step == 10:
                #if self.finished:
                rospy.loginfo('Process Finished')
                self.step = 0

if __name__ == '__main__':
    try:
        placing = PlacingDataCollection()
        placing.run()
    except rospy.ROSInterruptException: pass
