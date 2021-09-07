#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import actionlib
import dynamixel_sdk as dxl                  # Uses DYNAMIXEL SDK library
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint

# Control table address
ADDR_XL330_TORQUE_ENABLE       	= 64                          # Control table address is different in Dynamixel model

ADDR_XL330_PRESENT_VELOCITY	= 112	
ADDR_XL330_GOAL_POSITION       	= 116

ADDR_XL330_PRESENT_POSITION	= 132
ADDR_XL330_OPERATING_MODE	= 11
ADDR_XL330_CURRENT_LIMIT	= 38

ADDR_XL330_GOAL_CURRENT		= 102


LEN_GOAL_POSITION		= 4
LEN_PRESENT_VELOCITY		= 4
LEN_PRESENT_POSITION		= 4

# Operating mode
CURRENT_CONTROL_MODE		= 0
POSITION_CONTROL_MODE		= 3
CURRENT_POSITION_CONTROL_MODE	= 5

# Protocol version
PROTOCOL_VERSION            = 2    

DXL_ID_LEFT = [11,12,13,14]
DXL_ID_RIGHT = [1,2,3,4]

BAUDRATE                    = 57600
# how to find ?
# if the device name is correct, red, blue, and green led would light
DEVICENAME_LEFT                   = "/dev/dxl_left".encode('utf-8')        # Check which port is being used on your controller
DEVICENAME_RIGHT                  = "/dev/dxl_right".encode('utf-8')        # Check which port is being used on your controller

device_names = {'left' : DEVICENAME_LEFT,
                'right': DEVICENAME_RIGHT}
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

dxl_ids = {'left' : DXL_ID_LEFT,
           'right': DXL_ID_RIGHT}

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque

NUM_DXL				= 4

pos = [0,0,0,0]
vel = [0,0,0,0]

class HandInterface:
    def __init__(self, location, dev):
        if location != 'left' and location != 'right':
            raise NameError('location is wrong! location: {0}'.format(location))
        
        self.last_input = np.zeros(6)
        self.current_dxl_joints = np.zeros(4)
        self.calib_poses = {}
        self.calib_types = ['stretch',
                            'three_fingers_pinch',
                            'thumb_flexion',
                            'finger1_finger2_flexion',
                            'lateral_pinch']

        if location == 'left':
            self.gripper_min = [3471, 700, 700, 700]
            self.gripper_pinch = [2580, 1500, 1450, 1650]
            self.gripper_max = [2700, 2400, 2400, 2400]

        elif location == 'right':
            self.gripper_min = [1689, 700, 700, 700]
            self.gripper_pinch = [2580, 1450, 1650, 1650]
            self.gripper_max = [2700, 2000, 2100, 2100]

        else: raise NameError('??{0}'.format(location))

        self.__calibration(location=location)
        self.__init_dxl(location=location, device=dev)
        
        self.tau = 0.6


        self.full_joint_names = ['thumb_brake', 'index_brake', 'middle_brake', 'ring_brake', 'pinky_brake', 
                                    'thumb_cmc', 'index_mcp', 'middle_mcp', 'ring_mcp', 'pinky_mcp']
        self.vib_names = ['thumb_cmc', 'index_mcp', 'middle_mcp', 'ring_mcp', 'pinky_mcp']

        senseglove_loc_str = {'left':'lh', 'right':'rh'}
        
        rospy.Subscriber("/senseglove/0/{0}/joint_states".format(senseglove_loc_str[location]), JointState, self.callback, queue_size=1)
        self.feedback_client = actionlib.SimpleActionClient('/senseglove/0/{0}/controller/trajectory/follow_joint_trajectory'.format(senseglove_loc_str[location]), FollowJointTrajectoryAction)
        self.feedback_client.wait_for_server()
        self.feedback_goal = FollowJointTrajectoryGoal()

        point = JointTrajectoryPoint()
        point.positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        point.time_from_start = rospy.Duration.from_sec(0.3)
        self.feedback_goal.trajectory.points.append(point)

        self.set_glove_feedback(self.full_joint_names, [0] * 10)
        rospy.Subscriber("/tocabi/handforce_{0}".format(location), Float64MultiArray, self.callback1, queue_size=1)


        #Preset dynamixel joint value of Gripper
        # Thumb: Lateral Pinch, T-1, T-1	
        # Thumb: Init, pinch, full flexion		
        # Index: Init, pinch, full flexion	    
        # Middle: Init, pinch, full flexion

    def set_glove_feedback(self, names, vals):
        self.feedback_goal.trajectory.joint_names = names
        self.feedback_goal.trajectory.points[0].positions = vals
        self.feedback_goal.trajectory.header.stamp = rospy.Time.now()
        self.feedback_client.send_goal(self.feedback_goal)
        self.feedback_client.wait_for_result()

        # how to use 
        # self.set_glove_feedback(['thumb_cmc'], [40]) # vibration 40 %
        # self.set_glove_feedback(['thumb_brake', 'thumb_cmc'], [20, 40]) # break 20 %, vibration 40 %
    
    def __init_dxl(self, location, device):
        self.dxl_id = dxl_ids[location]
        # self.portHandler = dxl.PortHandler(device_names[location])
        self.portHandler = dxl.PortHandler(device)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = dxl.GroupSyncRead(self.portHandler, self.packetHandler, ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)

        for i in self.dxl_id	 :
            self.groupSyncRead.addParam(i)

        # Open port
        try: self.portHandler.clearPort()
        except: pass
        try: self.portHandler.closePort()
        except: pass
        if self.portHandler.openPort(): print("Succeeded to open the port")
        else: print("Failed to open the port")
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE): print("Succeeded to change the baudrate")
        else: print("Failed to change the baudrate")

        for i in range(4):
            self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id[i], ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)

        for i in range(4):
            self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id[i], ADDR_XL330_OPERATING_MODE , CURRENT_POSITION_CONTROL_MODE) 
            self.packetHandler.write2ByteTxRx(self.portHandler, self.dxl_id[i], ADDR_XL330_CURRENT_LIMIT , 900) 

        # Torque on
        for i in range(4):
            self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id[i], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        
    def __calibration(self, location):
        for calib_type in self.calib_types:
            self.calib_poses[calib_type] = rospy.get_param('/dyros_glove/calibration/{0}/{1}'.format(location,calib_type))

        # TODO: 
        # Use self.calib_poses['stretch'], : numpy.array(), len() = 4
        #     self.calib_poses['thumb_finger2'], : numpy.array(), len() = 4
        #     self.calib_poses['thumb_finger3'], : numpy.array(), len() = 4
        #     self.calib_poses['lateral_pinch'] : numpy.array(), len() = 4

        # example ############
        stretch_pose = self.calib_poses['stretch']
        im_flex_pose = self.calib_poses['finger1_finger2_flexion']
        thumb_flex_pose = self.calib_poses['thumb_flexion']
        ti_pinch_pose = self.calib_poses['three_fingers_pinch']
        tm_pinch_pose = self.calib_poses['three_fingers_pinch']
        lat_pinch_pose = self.calib_poses['lateral_pinch'] 
        
        #0:Thumb AA, 1:Thumb MCP, 2:Index MCP, 3:Middle MCP (16:Thumb AA, 18:Thumb MCP, 2:Index MCP, 6:Middle MCP)

        #######################

        glove_min = [0, 0, 0, 0]
        glove_pinch = [0, 0, 0, 0]
        glove_max = [0, 0, 0, 0]
        # glove_min : full extension -> Using stretch_pose for all three fingers
        glove_min[0] = lat_pinch_pose[0]
        glove_min[1] = -stretch_pose[1]
        glove_min[2] = stretch_pose[2]
        glove_min[3] = stretch_pose[3]

        #Flexion Activation at pinch - Thumb Index Middle

        #Using ti_pinch_pose, tm_pinch_pose 
        glove_pinch[0] = (ti_pinch_pose[0]+tm_pinch_pose[0])/2
        glove_pinch[1] = -tm_pinch_pose[1]
        glove_pinch[2] = ti_pinch_pose[2]
        glove_pinch[3] = tm_pinch_pose[3]

        glove_max[0] = tm_pinch_pose[0]
        glove_max[1] = -thumb_flex_pose[1]
        glove_max[2] = im_flex_pose[2]
        glove_max[3] = im_flex_pose[3]

        #glove_AA = [lat_pinch_pose[0], ti_pinch_pose[0], tm_pinch_pose[0]]
        #glove_AA = [0.22, 0.71, 0.76]  #AA activations of Lateral Pinch, T-1 and T-2
        #glove_AA = [0.2, 0.6, 0.69]

        self.glove_min = glove_min
        self.glove_pinch = glove_pinch
        self.glove_max = glove_max

        #print('glove_min', glove_min)
        #print('glove_pinch', glove_pinch)
        #print('glove_max', glove_max)

    def callback(self, data):
        self.read_joint_position()
        input_pose = data.position

        gripper_min = self.gripper_min
        gripper_pinch = self.gripper_pinch
        gripper_max = self.gripper_max
        
        self.new_input = np.array([input_pose[16], -input_pose[18], input_pose[2], input_pose[6], input_pose[10], input_pose[14]])
        
        glove_current = self.new_input * self.tau + self.last_input * (1 - self.tau)
        #glove_current[1] = -glove_current[1] # Minus!
        
        # glove_current: 0 thumb aa / 1 thumb flex / 2 index flex / 3 middle flex / 4 ring flex / 5 pinky flex
        
        glove_min = self.glove_min
        glove_pinch = self.glove_pinch
        glove_max = self.glove_max

        gripper_desired = [0, 0, 0, 0]
        
        # Pinch mode: 1DOF control of 3 fingers when middle, ring, pinky are fully flexed
        #             flexion_rate = normalized index mcp of glove
        
        #threshold = 0.5
        
        # if glove_current[3] > threshold and glove_current[4] > threshold and glove_current[5] > threshold:
            
        #     flexion_rate = (glove_current[2] - glove_min[2]) / (glove_max[2] - glove_min[2])

        #     for i in range(4):
        #         gripper_desired[i] = gripper_min[i] + (gripper_max[i] - gripper_min[i]) * flexion_rate

        #     gripper_desired[0] = 2700
        
        
        
        # Normal mapping: Linear mapping except for pinching region
        #                 gripper_desired = gripper_pinch when pinch_start <= glove < pinch_end
        #else:
        glove_pinch_start = [0, 0, 0, 0]
        glove_pinch_end = [0, 0, 0, 0]

        #glove_pinch_start[0] = glove_pinch[0] - 0.1  # Thumb AA pinch range is not required
        #glove_pinch_end[0] = glove_pinch[0] + 0.1

        # glove_pinch_start[1] = glove_pinch[1] - 0.2   glove_pinch_lower * (glove_pinch[i] - glove_min[i])
        # glove_pinch_end[1] = glove_pinch[1] + 0.05
        
        # glove_pinch_start[2] = glove_pinch[2] - 0.2
        # glove_pinch_end[2] = glove_pinch[2] + 0.2
        
        # glove_pinch_start[3] = glove_pinch[3] - 0.2
        # glove_pinch_end[3] = glove_pinch[3] + 0.2

        glove_pinch_lower = 0.22
        glove_pinch_upper = 0.15

        for i in range(1,4):
            glove_pinch_start[i] = glove_pinch[i] - glove_pinch_lower * (glove_pinch[i] - glove_min[i])
            glove_pinch_end[i] = glove_pinch[i] + glove_pinch_upper * (glove_max[i] - glove_pinch[i])
            
        # Calculating Gripper desired pose
        # Thumb AA joint : linear interpolation using min and Max data    
        gripper_desired[0] = int(gripper_min[0] + (gripper_max[0] - gripper_min[0]) * (glove_current[0] - glove_min[0]) / (glove_max[0] - glove_min[0]))

        # Thumb  and Fingers flexion joints : 
        for i in range(1,4):
            if glove_current[i] < glove_pinch_start[i]:
                gripper_desired[i] = int(gripper_min[i] + (gripper_pinch[i] - gripper_min[i]) * (glove_current[i] - glove_min[i]) / (glove_pinch_start[i] - glove_min[i]))
            elif glove_current[i] < glove_pinch_end[i]:
                gripper_desired[i] = int(gripper_pinch[i])
            else:
                gripper_desired[i] = int(gripper_max[i] + (gripper_pinch[i] - gripper_max[i]) * (glove_current[i] - glove_max[i]) / (glove_pinch_end[i] - glove_max[i]))


        # Range of motion limit
        for i in range(1,4) :
            if gripper_desired[i] < gripper_min[i]:
                gripper_desired[i] = gripper_min[i]
            elif gripper_desired[i] > gripper_max[i]:
                gripper_desired[i] = gripper_max[i]


        
        ###############################
       
        self.groupSyncWrite.clearParam()
        for i in range(4):
            param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(gripper_desired[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(gripper_desired[i])), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(gripper_desired[i])), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(gripper_desired[i]))]
            self.groupSyncWrite.addParam(self.dxl_id[i], param_goal_position)

        dxl_comm_result = self.groupSyncWrite.txPacket()
        self.last_input = glove_current

    def callback1(self, data):
        sensor_data = data.data
        vib_data = []
        for d in sensor_data:
            vib_data.append( min (d * 40, 100))
            if vib_data[-1] < 50.0:
                vib_data[-1] = 0
        
        # print('vib_data :' , vib_data)
        self.set_glove_feedback(self.full_joint_names[0:3] + self.vib_names[0:3], [0,0,0] + vib_data[0:3])

    def read_joint_position(self) :
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        for i in range(4):
            self.current_dxl_joints[i] = self.groupSyncRead.getData(self.dxl_id[i], ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
            #print('Joint pos read')
        
        # print('current pos ' , pos)
        
if __name__== '__main__':
    rospy.init_node('gripper_test', anonymous=True)
    location = rospy.get_param('~location', 'unset')
    dev = rospy.get_param('~dev', 'unset')
    hi = HandInterface(location, dev)
    rospy.spin()
    
    # shutdown
    for i in range(4):
        hi.packetHandler.write1ByteTxRx(hi.portHandler, hi.dxl_id[i], ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
        