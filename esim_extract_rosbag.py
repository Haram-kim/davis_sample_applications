#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 Sangil Lee

"""
Extract topics from a rosbag.
"""

import os
import argparse
import numpy as np
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import IPython

bridge = CvBridge()

def main():
    """
    Extract a topic from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("base_dir", nargs='?', default="./dataset", help="Output directory.")
    args = parser.parse_args()
    bag = rosbag.Bag(args.bag_file, "r")

    print "Extract topics from %s into %s" %(args.bag_file, args.base_dir)
    if not os.path.exists(os.path.join(args.base_dir,"image_dvs/")):
        os.makedirs(os.path.join(args.base_dir,"image_dvs"), mode=0o777)
    if not os.path.exists(os.path.join(args.base_dir,"image_rgbd/")):
        os.makedirs(os.path.join(args.base_dir,"image_rgbd"), mode=0o777)
    if not os.path.exists(os.path.join(args.base_dir,"depth/")):
        os.makedirs(os.path.join(args.base_dir,"depth"), mode=0o777)
	
    text_image_dvs = open(os.path.join(args.base_dir,"image_dvs.txt"), 'w')
    text_image_rgbd = open(os.path.join(args.base_dir,"image_rgbd.txt"), 'w')
    text_depth = open(os.path.join(args.base_dir,"depth.txt"), 'w')
    text_events = open(os.path.join(args.base_dir,"events.txt"), 'w')
    text_imu = open(os.path.join(args.base_dir,"imu.txt"), 'w')
    text_gt_pose = open(os.path.join(args.base_dir,"pose.txt"), 'w')

    text_image_dvs.write("# DVS images\n")
    text_image_dvs.write("# timestamp filename\n")
    text_image_rgbd.write("# RGBD images\n")
    text_image_rgbd.write("# timestamp filename\n")
    text_depth.write("# RGBD depth\n")
    text_depth.write("# timestamp filename\n")
    text_events.write("# events\n")
    text_events.write("# timestamp x y polarity\n")
    text_imu.write("# imu\n")
    text_imu.write("# acceleration gyroscope\n")
    text_imu.write("# timestamp ax ay az gx gy gz\n")
    text_gt_pose.write("# timestamp x y z qx qy qz qw\n")
    

    for topic, msg, t in bag.read_messages(topics=["/camera/rgb/image_color", "/camera/depth_registered/image", "/dvs/events", "/dvs/image_raw", "/dvs/imu", "/vicon/"]):

        if topic == "/dvs/image_raw":
            save_image(msg, t, args.base_dir, "image_dvs", text_image_dvs)
        elif topic == "/camera/rgb/image_color":
            save_image(msg, t, args.base_dir, "image_rgbd", text_image_rgbd)
        elif topic == "/camera/depth_registered/image":
            save_depth(msg, t, args.base_dir, "depth", text_depth)
        elif topic == "/dvs/events":
            save_event(msg, text_events)
        elif topic == "/dvs/imu":
            save_imu(msg, t, text_imu)
        elif topic == "/vicon/":
            save_pose(msg, t, text_gt_pose)

        print "\rTime passed: %i.%09i [s]" %(t.secs, t.nsecs),

    text_image_dvs.close()
    text_image_rgbd.close()
    text_depth.close()
    text_events.close()
    text_imu.close()
    text_gt_pose.close()
    bag.close()

    return

def save_image(msg, t, base_dir, output_dir, text):
    """
    save image into output directory
    """
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    filename = os.path.join(output_dir, "%i.%09i.png" %(t.secs, t.nsecs))

    cv2.imwrite(os.path.join(base_dir, filename), cv_img)
    text.write("%i.%09i\t%s\n" %( t.secs, t.nsecs, filename ))

def save_depth(msg, t, base_dir, output_dir, text):
    """
    save image into output directory
    """
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    cv_img = np.uint16(cv_img * 255)
    filename = os.path.join(output_dir, "%i.%09i.png" %(t.secs, t.nsecs))

    cv2.imwrite(os.path.join(base_dir, filename), cv_img)
    text.write("%i.%09i\t%s\n" %( t.secs, t.nsecs, filename ))

def save_event(msg, text):
    """
    save events into output directory
    """
    for e in msg.events:
        text.write("%i.%09i\t%i\t%i\t%i\n" %( e.ts.secs, e.ts.nsecs, e.x, e.y, e.polarity+0))

def save_imu(msg, t, text):
    """
    save imu into output directory
    """
    text.write("%i.%09i\t%f\t%f\t%f\t%f\t%f\t%f\n" 
            %( t.secs, t.nsecs, 
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))

def save_pose(msg, t, text):
    """
    save pose into output directory
    """
    text.write("%i.%09i\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" 
            %( t.secs, t.nsecs,
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

if __name__ == '__main__':
    main()
