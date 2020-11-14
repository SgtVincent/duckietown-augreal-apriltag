#!/usr/bin/env python3

import numpy as np
import os
import math
import cv2
import rospy
import yaml
import sys
import rospkg
from copy import deepcopy

from dt_apriltags import Detector
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from rospy import Subscriber, Publisher
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped

from renderClass import Renderer



"""

This is a template that can be used as a starting point for the CRA1 exercise.
You need to project the model file in the 'models' directory on an AprilTag.
To help you with that, we have provided you with the Renderer class that render the obj file.

"""

class ARNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ARNode, self).__init__(node_name=node_name,node_type=NodeType.GENERIC)
        self.veh_name = rospy.get_namespace().strip("/")
        self.node_name = rospy.get_name().strip("/")

        # initialize renderer
        rospack = rospkg.RosPack()
        self.renderer = Renderer(rospack.get_path('augmented_reality_apriltag') + '/src/models/duckie.obj')

        # initialize apriltag detector

        self.at_detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
            #searchpath=[]
            )
        self.at_camera_params = None
        # self.at_tag_size = 0.065 # fixed param 
        self.at_tag_size = 2 # to scale pose matrix to homography
        # initialize member variables
        self.bridge = CvBridge()
        self.camera_info_received = False
        self.camera_info = None
        self.camera_P = None
        self.camera_K = None
        self.cv2_img = None 

        # initialize subs and pubs
        self.sub_compressed_img = Subscriber("/{}/camera_node/image/compressed".format(self.veh_name),
                                             CompressedImage, self.cb_image, queue_size=1)
        self.loginfo("Subcribing to topic {}".format("/{}/camera_node/image/compressed".format(self.veh_name)))

        self.sub_camera_info = Subscriber("/{}/camera_node/camera_info".format(self.veh_name), CameraInfo,
                                          self.cb_camera_info, queue_size=1)
        self.loginfo("Subcribing to topic {}".format("/{}/camera_node/camera_info".format(self.veh_name)))

        self.pub_aug_img = Publisher(
            "/{}/{}/augmented_image/compressed".format(self.veh_name, self.node_name),
            CompressedImage, queue_size=1)
        self.loginfo("Publishing to topic {}".format(
            "/{}/{}/augmented_image/compressed".format(self.veh_name, self.node_name)
        ))


    def cb_image(self, msg):
        # cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        self.img_msg = msg


        return
    
    def aug_image_and_pub(self, msg):
        
        cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        # TODO: 1. Detect the AprilTag and extract its reference frame
        greyscale_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        
        tags = self.at_detector.detect(greyscale_img, True,
                                       self.at_camera_params, self.at_tag_size)

        # TODO: 2. Estimate the homography matrix
        # Read function estimate_pose_for_tag_homography in https://github.com/AprilRobotics/apriltag/blob/master/apriltag_pose.c
        # The pose_R and pose_t have been already computed
        pass

        # TODO 3. Derive the transformation from the reference frame of the AprilTag to the target image reference frame
        # project from X_T in frame T to pixel: P_camera @ [R|t] @ X_T
        # return P_T = P_camera @ [R|t]
        def compose_P_from_tag(P_camera, tag):

            T = np.concatenate(
                (
                    np.concatenate((tag.pose_R, tag.pose_t), axis=1),
                    np.array([[0, 0, 0, 1.0]])
                ),
                axis=0)
            self.logdebug("P_camera is \n {}".format(P_camera))
            
            P = P_camera @ T
            self.logdebug("P is \n {}".format(P))
            # norm_P = P/P[2,2]
            # self.logdebug("norm_P is \n {}".format(norm_P))
            return P

        # TODO 4. Project our 3D model in the image (pixel space) and draw it
        aug_img = cv2_img
        for tag in tags:
            aug_img = self.renderer.render(
                aug_img,
                compose_P_from_tag(self.camera_P, tag)
            )

        aug_image_msg = self.bridge.cv2_to_compressed_imgmsg(aug_img)
        aug_image_msg.header = msg.header
        self.pub_aug_img.publish(aug_image_msg)



    def cb_camera_info(self, msg):

        # self.logdebug("camera info received! ")
        if not self.camera_info_received:
            self.camera_info = msg
            # TODO: decide whether to use K or P for projection
            self.camera_P = np.array(msg.P).reshape((3,4))
            self.camera_K = np.array(msg.K).reshape((3,3))
            self.at_camera_params = (self.camera_P[0,0], self.camera_P[1,1],
                                     self.camera_P[0,2], self.camera_P[1,2])

        self.camera_info_received = True

        return


    # def projection_matrix(self, intrinsic, homography):
    #     """
    #         Write here the compuatation for the projection matrix, namely the matrix
    #         that maps the camera reference frame to the AprilTag reference frame.
    #     """
    #
    #     # TODO:
    #     # Write your code here
    #     #
    #     pass
    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.camera_info_received:
                img_msg = deepcopy(self.img_msg)
                self.aug_image_and_pub(img_msg)
                # self.logdebug("published one augmented image...")
            else:
                self.logwarn("Image received, by initialization not finished")
            rate.sleep()



    def onShutdown(self):
        super(ARNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    camera_node = ARNode(node_name='augmented_reality_apriltag_node')
    camera_node.run()
    # Keep it spinning to keep the node alive
    rospy.spin()