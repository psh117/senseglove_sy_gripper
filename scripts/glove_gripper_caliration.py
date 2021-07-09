from __future__ import print_function, division
import rospy
import os, sys
import copy
import numpy as np
import keyboard
from sensor_msgs.msg import JointState

class GloveCalibration:
    def __init__(self):
        self.past_glove_joints = np.zeros(4)
        self.current_dxl_joints = np.zeros(4)

        self.joint_captures = list()
        self.capture_trigger = False
        self.num_captures = 20
        self.tau = 0.6

        self.calib_types = ['stretch',
                            'thumb_finger1',
                            'thumb_finger2',
                            'lateral_pinch']
        rospy.Subscriber("/senseglove/0/rh/joint_states", JointState, self.joint_callback, queue_size=1)

    def joint_callback(self, data):
        input_pose = data.position
        self.current_glove_joint = np.array([input_pose[16], input_pose[18], input_pose[2], input_pose[6]])
        self.filtered_glove_joint = self.current_glove_joint * self.tau + self.past_glove_joints * (1 - self.tau)
        self.past_glove_joints = self.filtered_glove_joint

        if self.capture_trigger:
            self.joint_captures.append(self.filtered_glove_joint)
            
            print ('captured {0} samples'.format(len(self.joint_captures)))

            if len(self.joint_captures) >= self.num_captures:
                self.capture_trigger = False

    def calibration(self):
        for calib_type in self.calib_types:
            print ('calibrating...', calib_type)

            self.joint_captures = []
            while rospy.is_shutdown() is False:
                if keyboard.is_pressed('c'):  # if key 'c' is pressed 
                    self.capture_trigger = True
                    print('capturing {0} samples! don\'t move'.format(self.num_captures))
                    
                    # wait for capturing
                    while rospy.is_shutdown() is False:
                        if self.capture_trigger is False:
                            break
                    
                    break
            sum_joints = np.zeros(4)    
            for joints in self.joint_captures:
                sum_joints += joints

            mean_joints = sum_joints / float(self.num_captures)
            print(mean_joints)

            rospy.set_param('/dyros_glove/calibration/' + calib_type, mean_joints.tolist())

        print('calibration done')


if __name__== '__main__':
    rospy.init_node('glove_gripper_calibration')
    gc = GloveCalibration()
    gc.calibration()
