

#!/usr/bin/env python
"""
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi,   Extract images from a rosbag.       %
% University of Bonn- MSc Robotics & Geodetic Engineering   %
% Alireza.Ahmadi@uni-bonn.de                                %
% AlirezaAhmadi.xyz                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
"""

import os
import argparse

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
import rosbag
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

# Instantiate CvBridge
bridge = CvBridge()

count = 0

def imageExtractor(args):

    print "Extract images from %s on topic %s into %s" % (args.bagfile,
                                                          args.topicA, args.outputA)

    bag = rosbag.Bag(args.bagfile, "r")
    bridge = CvBridge()
    for topic, msg, t in bag.read_messages(topics=[args.topicA]):

        if args.encoding == "passthrough":
            # The depth image is a single-channel float32 image
            # the values is the distance in mm in z axis
            depth_image = bridge.imgmsg_to_cv2(msg, '32FC1')
            max_m_for_kinect = 5.0 ## You'll have to check out online for exact value 
            # depth_image = np.clip(depth_image,0,max_m_for_kinect) ## Filter all values > 5.0 m
            scaling_factor = 8.5
            # print(depth_image, np.min(depth_image), np.max(depth_image))
            depth_image = depth_image*scaling_factor #scaling the image to [0;65535]
            cv_image_array = np.array(depth_image,dtype=np.uint16) ## Creating the cv2 image

            cv2.imwrite(os.path.join(args.outputA, "%06i.png" % count), cv_image_array)


        elif args.encoding == "bgr8":
            cv_image_array = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(os.path.join(args.outputA, "%06i.png" % count), cv_image_array)

        cv2.imshow("Image from my node", cv_image_array)
        cv2.waitKey(1)

        # print "Wrote image %i" % count
        if args.frameNum < count and args.frameNum != 0: 
            break

    bag.close()


def imagePairExtractor(args, count):
    print "Needs to be run as a ros node!!"



def syncCallback(rgbImage, depthImage):
    global count 
    if count <= args.frameNum:
        try:
            # Convert your ROS Image message to OpenCV2
            cv_rgb = bridge.imgmsg_to_cv2(rgbImage, "bgr8")

        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite(os.path.join(args.outputA, "%06i.png" % count), cv_rgb)
            # cv2.imwrite('rgb_image.png', cv_rgb)
        
        try:
            # Convert your ROS Image message to OpenCV2
            # The depth image is a single-channel float32 image
            # the values is the distance in mm in z axis
            cv_depth = bridge.imgmsg_to_cv2(depthImage, "32FC1")
            
        except CvBridgeError, e:
            print(e)
        else:

            maxDepth = 5.0 ## check out online for exact value 
            scaling_factor = 8.5
            # print(cv_depth, np.min(cv_depth), np.max(cv_depth))
            cv_depth = cv_depth * scaling_factor #scaling the image to [0;65535]
            cv_depth = np.array(cv_depth,dtype=np.uint16) ## Creating the cv2 image
            cv2.imwrite(os.path.join(args.outputB, "%06i.png" % count), cv_depth)
            # cv2.imwrite('depth_image.png', cv_depth)

        count += 1



def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("frameNum", type=int, help="Number of images to extract.")
    parser.add_argument("--bagfile", help="Input ROS bag.", required= True)
    parser.add_argument("--outputA", help="Output directory A.", required= True)
    parser.add_argument("--outputB", help="Output directory B.", required= False)
    parser.add_argument("--topicA", help="Image topic A.", required= True)
    parser.add_argument("--topicB", help="Image topic B.", required= False)
    parser.add_argument("--sync", help="sync function? true or false.", default=False, required= False)
    parser.add_argument("--encoding", help="depth: passthrough, colored image: bgr8")

    global args
    args = parser.parse_args()

    if args.sync:
        
        if args.topicA == None or args.topicB == None:
            print "both rgb and depth topics must have right value not None!!"

        rospy.init_node('imageExtractor', anonymous=True)

        rgbSub = message_filters.Subscriber(args.topicA, Image)
        depthSub = message_filters.Subscriber(args.topicB, Image)

        ts = message_filters.TimeSynchronizer([rgbSub, depthSub], 10)
        ts.registerCallback(syncCallback)

        rospy.spin()
    else:
        imageExtractor(args)

    return

if __name__ == '__main__':
    main()
