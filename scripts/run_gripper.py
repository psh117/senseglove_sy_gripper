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

DXL_ID = [1,2,3,4]

BAUDRATE                    = 57600
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')        # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque

NUM_DXL				= 4

init_pos =[2580, 700, 700, 700]
pos = [0,0,0,0]
vel = [0,0,0,0]
desired_pos = [0,0,0,0]

class HandInterface:
    def __init__(self):
        self.past_glove_joints = np.zeros(4)
        self.current_dxl_joints = np.zeros(4)
        self.calib_poses = {}
        self.calib_types = ['stretch',
                            'finger1_finger2_flexion',
                            'thumb_flexion',
                            'thumb_finger1',
                            'thumb_finger2',
                            'lateral_pinch']

        self.__calibration()
        self.__init_dxl()
        
        self.tau = 0.6


        self.full_joint_names = ['thumb_brake', 'index_brake', 'middle_brake', 'ring_brake', 'pinky_brake', 
                                    'thumb_cmc', 'index_mcp', 'middle_mcp', 'ring_mcp', 'pinky_mcp']
        self.vib_names = ['thumb_cmc', 'index_mcp', 'middle_mcp', 'ring_mcp', 'pinky_mcp']
        rospy.Subscriber("/senseglove/0/rh/joint_states", JointState, self.callback, queue_size=1)
        self.feedback_client = actionlib.SimpleActionClient('/senseglove/0/rh/controller/trajectory/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.feedback_client.wait_for_server()
        self.feedback_goal = FollowJointTrajectoryGoal()

        point = JointTrajectoryPoint()
        point.positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        point.time_from_start = rospy.Duration.from_sec(0.3)
        self.feedback_goal.trajectory.points.append(point)

        self.set_glove_feedback(self.full_joint_names, [0] * 10)
        rospy.Subscriber("/tocabi/handforce", Float64MultiArray, self.callback1, queue_size=1)

    def set_glove_feedback(self, names, vals):
        self.feedback_goal.trajectory.joint_names = names
        self.feedback_goal.trajectory.points[0].positions = vals
        self.feedback_goal.trajectory.header.stamp = rospy.Time.now()
        self.feedback_client.send_goal(self.feedback_goal)
        self.feedback_client.wait_for_result()

        # how to use 
        # self.set_glove_feedback(['thumb_cmc'], [40]) # vibration 40 %
        # self.set_glove_feedback(['thumb_brake', 'thumb_cmc'], [20, 40]) # break 20 %, vibration 40 %
    
    def __init_dxl(self):
        self.portHandler = dxl.PortHandler(DEVICENAME)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = dxl.GroupSyncRead(self.portHandler, self.packetHandler, ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)

        for i in DXL_ID	 :
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


        # Position mode
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[0], ADDR_XL330_OPERATING_MODE , POSITION_CONTROL_MODE) 
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[1], ADDR_XL330_OPERATING_MODE , POSITION_CONTROL_MODE) 
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[2], ADDR_XL330_OPERATING_MODE , POSITION_CONTROL_MODE) 
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[3], ADDR_XL330_OPERATING_MODE , POSITION_CONTROL_MODE) 

        # Torque on
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[0], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[1], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[2], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[3], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        
    def __calibration(self):
        for calib_type in self.calib_types:
            self.calib_poses[calib_type] = rospy.get_param('/dyros_glove/calibration/' + calib_type)

        # TODO: 
        # Use self.calib_poses['stretch'], : numpy.array(), len() = 4
        #     self.calib_poses['thumb_finger2'], : numpy.array(), len() = 4
        #     self.calib_poses['thumb_finger3'], : numpy.array(), len() = 4
        #     self.calib_poses['lateral_pinch'] : numpy.array(), len() = 4

        # example ############
        str_pos = self.calib_poses['stretch']
        f12_pos = self.calib_poses['finger1_finger2_flexion']
        tfe_pos = self.calib_poses['thumb_flexion']
        tf1_pos = self.calib_poses['thumb_finger1']
        tf2_pos = self.calib_poses['thumb_finger2']
        lap_pos = self.calib_poses['lateral_pinch'] 
        
        #0:Thumb AA, 1:Thumb MCP, 2:Index MCP, 3:Middle MCP (16:Thumb AA, 18:Thumb MCP, 2:Index MCP, 6:Middle MCP)

        v1 = str_pos[0] + str_pos[1]
        #######################
        
        
        
        
        
        
        #Preset dynamixel joint value of Gripper
        ps = np.array([[1689, init_pos[0], 2700], [init_pos[1], 1650 - init_pos[1] , 2400 - init_pos[1]], [init_pos[2], 1800 - init_pos[2], 2400 - init_pos[2]], [init_pos[3], 1800 - init_pos[3], 2400 - init_pos[3]]])
        # Thumb: Lateral Pinch, T-1, T-1	Thumb: Init, pinch, full flexion		Index: Init, pinch, full flexion	    Middle: Init, pinch, full flexion




        a0 = [0, 0, 0, 0]

        # a0 : full extension -> Using str_pos for all three fingers
        a0[1] = -str_pos[1]  # Minus!!!
        a0[2] = str_pos[2]
        a0[3] = str_pos[3]

        #Flexion Activation at pinch - Thumb Index Middle
        a = [0, 0, 0, 0]			
        a[0] = 0					#Not used

        #Using tf1_pos, tf2_pos 
        a[1] = -(tf1_pos[1]+tf2_pos[1])/2   # MINUS Value!!!! 
        a[2] = tf1_pos[2]
        a[3] = tf2_pos[3]



        a_max = [0, 0, 0, 0]
        a_max[1] = -tfe_pos[1] - a0[1] # Minus!!!
        a_max[2] = f12_pos[2] - a0[2]
        a_max[3] = f12_pos[3] - a0[3]

        a_thumbAA = [lap_pos[0], tf1_pos[0], tf2_pos[0]]
        #a_thumbAA = [0.22, 0.71, 0.76]  #AA activations of Lateral Pinch, T-1 and T-2
        #a_thumbAA = [0.2, 0.6, 0.69]

        #Thumb FE
        f1 = [0, 0, 0]
        f1[0] = -(ps[1,2]-ps[1,1])/((ps[1,2]/a_max[1]) - (ps[1,1] / (a[1] - a0[1])))  	# x_0
        f1[1] = (ps[1,2]/a_max[1]) * (f1[0] + a_max[1]) * f1[0]  	# k
        f1[2] = ps[1,2] + (f1[1] / (a_max[1] + f1[0]))	# y_0


        #Index FE
        f2 = [0, 0, 0]
        # F : Y= -k/(x + x_0) + y_0
        # F : Y = -f2[1]/(data + f2[0]) + f2[2]
        #f2[0] = -(ps[2,2]-ps[2,1])/((ps[2,2]/3) - ((ps[2,2]-ps[2,1]) / (a[2]-a0[2])))  	# x_0
        #f2[1] = (ps[2,2]/3) * (f2[0] + 3) * f2[0]  	# k
        #f2[2] = ps[2,2] + (f2[1] / (3 + f2[0]))	# y_0

        f2[0] = -(ps[2,2]-ps[2,1])/((ps[2,2]/a_max[2]) - (ps[2,1] / (a[2]-a0[2])))  	# x_0
        f2[1] = (ps[2,2]/a_max[2] ) * (f2[0] + a_max[2] ) * f2[0]  	# k
        f2[2] = ps[2,2] + (f2[1] / (a_max[2]  + f2[0]))	# y_0


        #Middle FE
        f3 = [0, 0, 0]
        f3[0] = -(ps[3,2]-ps[3,1])/((ps[3,2]/a_max[3]) - (ps[3,1] / (a[3] - a0[3])))  	# x_0
        f3[1] = (ps[3,2]/a_max[3]) * (f3[0] + a_max[3]) * f3[0]  	# k
        f3[2] = ps[3,2] + (f3[1] / (a_max[3] + f3[0]))	# y_0

        #Thumb AA
        q = np.array([[0], [891], [1011]]) 

        M_AA = np.array([[a_thumbAA[0]*a_thumbAA[0], a_thumbAA[0], 1] , [a_thumbAA[1]*a_thumbAA[1], a_thumbAA[1], 1], [a_thumbAA[2]*a_thumbAA[2], a_thumbAA[2], 1]])
        M_AA_inv = np.linalg.inv(M_AA)
        f0 = np.dot(M_AA_inv, q)

        self.a0 = a0
        self.a = a
        self.a_max = a_max
        self.f0 = f0
        self.f1 = f1
        self.f2 = f2
        self.f3 = f3
        #f0[0,0]
        #f0[0,1]
        #f0[0,2]
        # F = f0[0,0]*x^2 + f0[0,1]*x + f0[0,2]

    def callback(self, data):
        self.read_joint_position()
        input_pose = data.position
        
        self.current_glove_joint = np.array([input_pose[16], input_pose[18], input_pose[2], input_pose[6]])
        self.filtered_glove_joint = self.current_glove_joint * self.tau + self.past_glove_joints * (1 - self.tau)

        #Thumb AA
        q = self.filtered_glove_joint

        f0 = self.f0
        f1 = self.f1
        f2 = self.f2
        f3 = self.f3
        a0 = self.a0

        desired_pos[0] = 1689 + int(f0[0,0]*q[0]*q[0] + f0[1,0]*q[0] + f0[2,0])
        desired_pos[1] = init_pos[1] + int( -f1[1]/((-q[1] - a0[1]) + f1[0]) + f1[2] )   #Minus data !!!
        desired_pos[2] = init_pos[2] + int( -f2[1]/((q[2] - a0[2]) + f2[0]) + f2[2] )
        desired_pos[3] = init_pos[3] + int( -f3[1]/((q[3] - a0[3]) + f3[0]) + f3[2] ) 

        if desired_pos[0] < 1000 :
            desired_pos[0] = 1689

        for i in range(1,4) :
            if desired_pos[i] < 600 :
                desired_pos[i] = 600

        # print('desired_position', desired_pos)
        #print('current_position', self.current_dxl_joints)

        ###############################


        self.groupSyncWrite.clearParam()
        for i in range(4):
            param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(desired_pos[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(desired_pos[i])), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(desired_pos[i])), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(desired_pos[i]))]
            self.groupSyncWrite.addParam(DXL_ID[i], param_goal_position)

        dxl_comm_result = self.groupSyncWrite.txPacket()
        self.past_glove_joints = self.filtered_glove_joint

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
        for idx in DXL_ID :
            self.current_dxl_joints[idx-1] = self.groupSyncRead.getData(idx, ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
            #print('Joint pos read')
        
        # print('current pos ' , pos)
        
if __name__== '__main__':
    rospy.init_node('gripper_test')
    hi = HandInterface()
    rospy.spin()