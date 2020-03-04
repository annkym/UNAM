#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Image Saver Module
"""

#import datetime
import os
from queue import Queue

from PIL import Image
import numpy as np

import rospy
import roslib.packages
from sensor_msgs.msg import Image as ImageMsg

from hsr_image_collector.msg import TakePictureRequestMsg
import csv

class ImageSaver(object):
    """
    ImageSaver
    """

    def __init__(self):
        """
        Initialization
        """

        self.output_img_dir = os.path.join(roslib.packages.get_pkg_dir('handyman'),
                              "placing_dataset")

        # make dirs for img output
        try:
            for target_dir in [self.output_img_dir]:
                os.mkdir(target_dir)

        except OSError as os_err:
            rospy.logerr(os_err)


        self.take_pic_interval_sec = 0.5  # sec


        # camera topic
        self.sub_head_center_img_raw_topic_name = "/hsrb/head_rgbd_sensor/depth/image_raw"
        self.sub_head_rgbd_img_raw_topic_name = "/hsrb/head_rgbd_sensor/rgb/image_raw"


        # def subscribers
        self.head_img_raw_subscriber = rospy.Subscriber(self.sub_head_center_img_raw_topic_name,
                                                        ImageMsg,
                                                        self.head_center_img_callback)

        # def subscribers
        self.xtion_img_raw_subscriber = rospy.Subscriber(self.sub_head_rgbd_img_raw_topic_name,
                                                         ImageMsg,
                                                         self.xtion_img_callback)
        self.xtion_img_index = 0
        self.head_img_index = 0


    def head_center_img_callback(self, ros_img_msg):
        self.depth_data = ros_img_msg


    def xtion_img_callback(self, ros_img_msg):
        """
        call back of xtion image topic
        """
        self.rgb_data = ros_img_msg


    @classmethod
    def convert_img_to_pil(cls, ros_img_msg, camera_type):
        """
        Converting data type from ros/Image to PIL/Image
        """

        try:
            np_arr = np.fromstring(ros_img_msg.data, np.uint8)
            np_arr = np_arr.reshape((ros_img_msg.height, ros_img_msg.width, 3))
            if camera_type == "hand":
                np_arr = np.rot90(np_arr)
            rospy.loginfo(np_arr.shape)
            pil_img = Image.fromarray(np_arr)
            return pil_img
        except Exception as err:
            rospy.logerr(err)
            rospy.is_shutdown()

    def convert_img_to_csv(self, ros_img_msg, file_name, img_output_path):
        np_arr = np.array(np.fromstring(ros_img_msg.data, np.uint16))
        reshaped_array = np_arr.reshape((ros_img_msg.height, ros_img_msg.width))
        np.savetxt(file_name, reshaped_array, fmt="%i")



    def set_directory(self, directory):

            new_output_dir = os.path.join(self.output_img_dir, directory)

            # for RGB camera
            #self.xtion_img_index = 0
            self.xtion_output_dir = new_output_dir

            # for DEPTH head camera
            #self.head_img_index = 0
            self.head_center_output_dir = new_output_dir

            # make dirs for img output
            try:
                for target_dir in [new_output_dir]:
                    os.mkdir(target_dir)

            except OSError as os_err:
                rospy.logerr(os_err)

            return new_output_dir


    def save_img(self, camera_type):
        """
        Saving image
        """

        if camera_type == 'depth':
            ros_img_data = self.depth_data
            output_dir = self.head_center_output_dir
            self.head_img_index += 1
            img_index = self.head_img_index
            file_output_path = os.path.join(
                output_dir, '%04d_depth.csv' % img_index)
            img_output_path = os.path.join(
                output_dir, '%04d_depth.png' % img_index)
            self.convert_img_to_csv(ros_img_data, file_output_path, img_output_path)
            rospy.loginfo('Finish CSV')

        elif camera_type == 'depth-final':
            ros_img_data = self.depth_data
            output_dir = self.head_center_output_dir
            #self.head_img_index += 1
            img_index = self.head_img_index
            file_output_path = os.path.join(
                output_dir, '%04d_depth-final.csv' % img_index)
            img_output_path = os.path.join(
                output_dir, '%04d_depth-final.png' % img_index)
            self.convert_img_to_csv(ros_img_data, file_output_path, img_output_path)
            rospy.loginfo('Finish final CSV')

        elif camera_type == 'rgb':
            ros_img_data = self.rgb_data
            output_dir = self.xtion_output_dir
            self.xtion_img_index += 1
            img_index = self.xtion_img_index
            img_output_path = os.path.join(
                output_dir, '%04d_rgb.jpg' % img_index)
            rospy.loginfo('Saving image-->')
            pil_img = self.convert_img_to_pil(ros_img_data, camera_type)
            pil_img.save(img_output_path)
            rospy.loginfo('Finish saving')

        elif camera_type == 'rgb-final':
            ros_img_data = self.rgb_data
            output_dir = self.xtion_output_dir
            #self.xtion_img_index += 1
            img_index = self.xtion_img_index
            img_output_path = os.path.join(
                output_dir, '%04d_rgb-final.jpg' % img_index)
            rospy.loginfo('Saving image-->')
            pil_img = self.convert_img_to_pil(ros_img_data, camera_type)
            pil_img.save(img_output_path)
            rospy.loginfo('Finish saving final RGB')

        else:
            rospy.logerr(
                'camera_type must be depth or rgb, but in real %s' % str(camera_type))


    def __call__(self):
        """
        running
        """

        rospy.loginfo('start runnning')
        rospy.init_node('ImageSaver', anonymous=True)
        rospy.loginfo('ImageSaver start!')
        self.take_pic_interval_sec = float(
            rospy.get_param('~interval_sec', 0.5))
        rospy.loginfo("interval sec is %f sec" % self.take_pic_interval_sec)
        rospy.spin()


def run():
    """
    Running
    """
    try:
        image_saver = ImageSaver()
        image_saver()

    except rospy.ROSInterruptException:
        pass
