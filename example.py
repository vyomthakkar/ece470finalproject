import sim
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm


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
time.sleep(1)

zero_state_trans = sim.simxGetJointMatrix(clientID, joint_six_handle, sim.simx_opmode_blocking)
M = np.array([[zero_state_trans[1][0], zero_state_trans[1][1], zero_state_trans[1][2], zero_state_trans[1][3]],
              [zero_state_trans[1][4], zero_state_trans[1][5], zero_state_trans[1][6], zero_state_trans[1][7]],
              [zero_state_trans[1][8], zero_state_trans[1][9], zero_state_trans[1][10], zero_state_trans[1][11]],
              [0,0,0,1]])
#print(repr(M),"\n")

qx, qy, qz = get_joint()
#print(repr(qx),"\n")
#print(repr(qy),"\n")
#print(repr(qz),"\n")

omegas = np.array([[0,0,1],
                   [-1,0,0],
                   [-1,0,0],
                   [1,0,0],
                   [0,0,-1],
                   [1,0,0]])
#print(repr(omegas),"\n")
    
v = np.array([np.array(np.cross(-omegas[0],np.array([qx[0],qy[0],qz[0]]))),
              np.array(np.cross(-omegas[1],np.array([qx[0],qy[0],qz[0]]))),
              np.array(np.cross(-omegas[2],np.array([qx[0],qy[0],qz[0]]))),
              np.array(np.cross(-omegas[3],np.array([qx[0],qy[0],qz[0]]))),
              np.array(np.cross(-omegas[4],np.array([qx[0],qy[0],qz[0]]))),
              np.array(np.cross(-omegas[5],np.array([qx[0],qy[0],qz[0]])))])
#print(repr(v),"\n")

s_bracket_1 = np.array([[0,-omegas[0][2],omegas[0][1],v[0][0]],[omegas[0][2],0,-omegas[0][0],v[0][1]],[-omegas[0][1],omegas[0][0],0,v[0][2]],[0,0,0,0]])
s_bracket_2 = np.array([[0,-omegas[1][2],omegas[1][1],v[1][0]],[omegas[1][2],0,-omegas[1][0],v[1][1]],[-omegas[1][1],omegas[1][0],0,v[1][2]],[0,0,0,0]])
s_bracket_3 = np.array([[0,-omegas[2][2],omegas[2][1],v[2][0]],[omegas[2][2],0,-omegas[2][0],v[2][1]],[-omegas[2][1],omegas[2][0],0,v[2][2]],[0,0,0,0]])
s_bracket_4 = np.array([[0,-omegas[3][2],omegas[3][1],v[3][0]],[omegas[3][2],0,-omegas[3][0],v[3][1]],[-omegas[3][1],omegas[3][0],0,v[3][2]],[0,0,0,0]])
s_bracket_5 = np.array([[0,-omegas[4][2],omegas[4][1],v[4][0]],[omegas[4][2],0,-omegas[4][0],v[4][1]],[-omegas[4][1],omegas[4][0],0,v[4][2]],[0,0,0,0]])
s_bracket_6 = np.array([[0,-omegas[5][2],omegas[5][1],v[5][0]],[omegas[5][2],0,-omegas[5][0],v[5][1]],[-omegas[5][1],omegas[5][0],0,v[5][2]],[0,0,0,0]])

thetas_string = input("PLEASE input 6 floats for the joint angles for the UR3\n")
thetas_string_split = thetas_string.split(" ")
out_thetas = [float(thetas_string_split[i])*np.pi/180 for i in range(len(thetas_string_split))]

s_bracket_1_theta = np.dot(s_bracket_1, out_thetas[0])
s_bracket_2_theta = np.dot(s_bracket_2, out_thetas[1])
s_bracket_3_theta = np.dot(s_bracket_3, out_thetas[2])
s_bracket_4_theta = np.dot(s_bracket_4, out_thetas[3])
s_bracket_5_theta = np.dot(s_bracket_5, out_thetas[4])
s_bracket_6_theta = np.dot(s_bracket_6, out_thetas[5])

exp_1 = expm(s_bracket_1_theta)
exp_2 = expm(s_bracket_2_theta)
exp_3 = expm(s_bracket_3_theta)
exp_4 = expm(s_bracket_4_theta)
exp_5 = expm(s_bracket_5_theta)
exp_6 = expm(s_bracket_6_theta)

out_T = np.dot(exp_1, np.dot(exp_2, np.dot(exp_3, np.dot(exp_4, np.dot(exp_5, np.dot(exp_6, M))))))
print(repr(out_T),"\n")

time.sleep(3)
SetJointPosition(out_thetas)

#Goal_joint_angles = np.array([[np.pi,-0.5*np.pi,0,0,0.5*np.pi,0],
#                              [0,0,0,0,0,0],
#                              [0.5*np.pi,-0.5*np.pi,0,0,0.5*np.pi,0],
#                              [0,0,0,0,0,0],
#                              [np.pi,-0.5*np.pi,0,0,0.5*np.pi,0],
#                              [0,0,0,0,0,0],
#                              [-0.5*np.pi,-0.5*np.pi,0,0,0.5*np.pi,0],
#                              [0,0,0,0,0,0]])
				
#for i in range(len(Goal_joint_angles)):
#    SetJointPosition(Goal_joint_angles[i])

# Wait two seconds
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