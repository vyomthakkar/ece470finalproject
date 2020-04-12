import sim
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm
import modern_robotics as mr


# Get distances measurements from each joint center to base frame (useful for forward kinematics)
def get_joint():
	X = []
	Y = []
	Z = []
	result,vector=sim.simxGetObjectPosition(clientID, joint_one_handle,base_handle,sim.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=sim.simxGetObjectPosition(clientID, joint_two_handle,base_handle,sim.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=sim.simxGetObjectPosition(clientID, joint_three_handle,base_handle,sim.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=sim.simxGetObjectPosition(clientID, joint_four_handle,base_handle,sim.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=sim.simxGetObjectPosition(clientID, joint_five_handle,base_handle,sim.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=sim.simxGetObjectPosition(clientID, joint_six_handle,base_handle,sim.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	result,vector=sim.simxGetObjectPosition(clientID, end_handle,base_handle,sim.simx_opmode_blocking)
	X.append(vector[0])
	Y.append(vector[1])
	Z.append(vector[2])
	X = np.round(X, decimals = 3)
	Y = np.round(Y, decimals = 3)
	Z = np.round(Z, decimals = 3)
	return X,Y,Z

# Function that used to move joints
def SetJointPosition(theta):
	sim.simxSetJointTargetPosition(clientID, joint_one_handle, theta[0], sim.simx_opmode_oneshot)
	time.sleep(0.5)
	sim.simxSetJointTargetPosition(clientID, joint_two_handle, theta[1], sim.simx_opmode_oneshot)
	time.sleep(0.5)
	sim.simxSetJointTargetPosition(clientID, joint_three_handle, theta[2], sim.simx_opmode_oneshot)
	time.sleep(0.5)
	sim.simxSetJointTargetPosition(clientID, joint_four_handle, theta[3], sim.simx_opmode_oneshot)
	time.sleep(0.5)
	sim.simxSetJointTargetPosition(clientID, joint_five_handle, theta[4], sim.simx_opmode_oneshot)
	time.sleep(0.5)
	sim.simxSetJointTargetPosition(clientID, joint_six_handle, theta[5], sim.simx_opmode_oneshot)
	time.sleep(0.5)

# Function that used to read joint angles
def GetJointAngle():
	result, theta1 = sim.simxGetJointPosition(clientID, joint_one_handle, sim.simx_opmode_blocking)
	if result != sim.simx_return_ok:
		raise Exception('could not get 1 joint variable')
	result, theta2 = sim.simxGetJointPosition(clientID, joint_two_handle, sim.simx_opmode_blocking)
	if result != sim.simx_return_ok:
		raise Exception('could not get 2 joint variable')
	result, theta3 = sim.simxGetJointPosition(clientID, joint_three_handle, sim.simx_opmode_blocking)
	if result != sim.simx_return_ok:
		raise Exception('could not get 3 joint variable')
	result, theta4 = sim.simxGetJointPosition(clientID, joint_four_handle, sim.simx_opmode_blocking)
	if result != sim.simx_return_ok:
		raise Exception('could not get 4 joint variable')
	result, theta5 = sim.simxGetJointPosition(clientID, joint_five_handle, sim.simx_opmode_blocking)
	if result != sim.simx_return_ok:
		raise Exception('could not get 5 joint variable')
	result, theta6 = sim.simxGetJointPosition(clientID, joint_six_handle, sim.simx_opmode_blocking)
	if result != sim.simx_return_ok:
		raise Exception('could not get 6 joint variable')
	theta = np.array([[theta1],[theta2],[theta3],[theta4],[theta5],[theta6]])
	return theta



# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

# Close all open connections (Clear bad cache)
sim.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  =========================================== #

'''
# Print object name list
result,joint_name,intData,floatData,stringData = sim.simxGetObjectGroupData(clientID,sim.sim_appobj_object_type,0,sim.simx_opmode_blocking)
print(stringData)
'''

# Get "handle" to the base of robot
result, base_handle = sim.simxGetObjectHandle(clientID, 'UR3_link1_visible', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for base frame')

# Get "handle" to the all joints of robot
result, joint_one_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint1', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for first joint')
result, joint_two_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint2', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for second joint')
result, joint_three_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint3', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for third joint')
result, joint_four_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint4', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for fourth joint')
result, joint_five_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint5', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for fifth joint')
result, joint_six_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint6', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')

# Get "handle" to the end-effector of robot
result, end_handle = sim.simxGetObjectHandle(clientID, 'UR3_link7_visible', sim.simx_opmode_blocking)
if result != sim.simx_return_ok:
	raise Exception('could not get object handle for end effector')
# ==================================================================================================== #

# Start simulation
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

# ******************************** Your robot control code goes here  ******************************** #
rotation_string = input("PLEASE input 9 floats for the rotation matrix for the UR3 end effector space separated in row major order\n")
rotation_string_split = rotation_string.split(" ")
trans_string = input("PLEASE input 3 floats for the translation matrix for the UR3 end effector space separated in row major order\n")
trans_string_split = trans_string.split(" ")

pose_inv_kin = np.zeros((4, 4))
for i, val in enumerate(rotation_string_split):
    pose_inv_kin[int(i/3), int(i%3)] = val
for i, val in enumerate(trans_string_split):
    pose_inv_kin[i, 3] = val
pose_inv_kin[3, 3] = 1

# print(repr(pose_inv_kin))

M = np.array ([[ 0, -1,  0,  0.39   ],
               [ 0,  0, -1,  0.401  ],
               [ 1,  0,  0,  0.2155 ],
               [ 0,  0,  0,  1      ]])
    
# Rotations (omegas) of each joint with respect to base frame in 0 position
omegas = np.array([[ 0, 0, 1 ],
                   [ 0, 1, 0 ],
                   [ 0, 1, 0 ],
                   [ 0, 1, 0 ],
                   [ 1, 0, 0 ],
                   [ 0, 1, 0 ]])

# Points on each joint's screw axis
Q = np.array([[ -0.15, 0.15 , 0.01  ],
              [ -0.15, 0.27 , 0.162 ],
              [ 0.094, 0.27 , 0.162 ],
              [ 0.307, 0.177, 0.162 ],
              [ 0.307, 0.26 , 0.162 ],
              [ 0.39 , 0.26 , 0.162 ]])

# Linear velocity values
v = np.array([np.array(np.cross(-omegas[0], Q[0])),
              np.array(np.cross(-omegas[1], Q[1])),
              np.array(np.cross(-omegas[2], Q[2])),
              np.array(np.cross(-omegas[3], Q[3])),
              np.array(np.cross(-omegas[4], Q[4])),
              np.array(np.cross(-omegas[5], Q[5]))])

# Screw axis vectors
S1 = np.array([omegas[0], v[0]]).reshape((6,))
S2 = np.array([omegas[1], v[1]]).reshape((6,))
S3 = np.array([omegas[2], v[2]]).reshape((6,))
S4 = np.array([omegas[3], v[3]]).reshape((6,))
S5 = np.array([omegas[4], v[4]]).reshape((6,))
S6 = np.array([omegas[5], v[5]]).reshape((6,))

S = np.zeros((6,6))
S[:,0] = S1
S[:,1] = S2
S[:,2] = S3
S[:,3] = S4
S[:,4] = S5
S[:,5] = S6

thetalist0 = np.zeros((S.shape[1]))
thetalist,success = mr.IKinSpace(S,M,pose_inv_kin,thetalist0,0.01,0.01)
#print("thetas found: \n", repr(thetalist.reshape(S.shape[1],1)))

thetalist.reshape((6,))
print(thetalist*180/np.pi)

SetJointPosition(thetalist)

time.sleep(5)

# **************************************************************************************************** #

# Stop simulation
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
sim.simxGetPingTime(clientID)
# Close the connection to V-REP
sim.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")

# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #
