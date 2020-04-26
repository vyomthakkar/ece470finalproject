import sim
import time
import numpy as np
import matplotlib.pyplot as mpl

def gripper(clientID, closing, j1, j2):
    r, p1 = sim.simxGetJointPosition(clientID, j1, sim.simx_opmode_blocking)
    r, p2 = sim.simxGetJointPosition(clientID, j2, sim.simx_opmode_blocking)

    if closing:
        if p1 < (p2 - 0.008):
            sim.simxSetJointTargetVelocity(clientID, j1, -0.01, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, j2, -0.04, sim.simx_opmode_blocking)
        else:
            sim.simxSetJointTargetVelocity(clientID, j1, -0.04, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, j2, -0.04, sim.simx_opmode_blocking)
    else:
        if p1 < p2:
            sim.simxSetJointTargetVelocity(clientID, j1, 0.04, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, j2, 0.02, sim.simx_opmode_blocking)
        else:
            sim.simxSetJointTargetVelocity(clientID, j1, 0.02, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, j2, 0.04, sim.simx_opmode_blocking)

def moveL(clientID, target, pos, speed):
    r, p = sim.simxGetObjectPosition(clientID, target,-1,sim.simx_opmode_blocking)
    r, o = sim.simxGetObjectOrientation(clientID, target,-1,sim.simx_opmode_blocking)

    for i in range(3):
        if np.abs(pos[i+3] - o[i]) > np.pi and o[i] < 0:
            o[i] = o[i] + 2 * np.pi
        elif np.abs(pos[i+3] - o[i]) > np.pi and o[i] > 0:
            o[i] = o[i] - 2 * np.pi

    old_pos = np.concatenate((p, o))
    delta_pos = pos - old_pos
    distance = np.linalg.norm(delta_pos)
    samples_number = int(np.floor(distance * 50))
    # print(samples_number)

    for i in range(samples_number):
        intermediate_pos = old_pos + (delta_pos / samples_number)
        start_time = time.time()
        while (time.time() - start_time < distance / (speed * samples_number)):
            continue
        # print("pos to set: ", intermediate_pos)
        sim.simxSetObjectPosition(clientID, target, -1, intermediate_pos[0:3], sim.simx_opmode_blocking)
        sim.simxSetObjectOrientation(clientID, target, -1, intermediate_pos[3:6], sim.simx_opmode_blocking)
        old_pos = intermediate_pos

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

# GET OBJECT HANDLES
res, j1 = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active1', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
	raise Exception('could not get object handle for active 1')

res, j2 = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active2', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
	raise Exception('could not get object handle for active 2')

res, ur3_target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
	raise Exception('could not get object handle for target')
    
res, proximity_sensor = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
	raise Exception('could not get object handle for proximity sensor')

res, connector = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_attachPoint', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
	raise Exception('could not get object handle for attach point')
    
res, vision_sensor = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
if res != sim.simx_return_ok:
	raise Exception('could not get object handle for vision sensor')

# Waypoints
fposition1 = [-0.475, 0.15, 1.25, 0, 0, 0] # resting position (home)
fposition2 = [-0.15, 0.5, 1, 0, -np.pi/2, 0] # location above conveyor belt
fposition3 = [-0.15, 0.5, 0.65, 0, -np.pi/2, 0] # location of block on conveyor belt
fposition4 = [-0.675, -0.025, 1, 0, -np.pi/2, 0] # red blocks location
fposition5 = [0.26, -0.073, 1, 0, -np.pi/2, 0] # green blocks location

# Move to home
gripper(clientID, 0, j1, j2)
moveL(clientID, ur3_target, fposition1, 2)

# Wait for blocks on conveyor belt
while(1):
    # Check proximity sensor
    res, detectState, detectedPoint, detObjHandle, detVec = sim.simxReadProximitySensor(clientID, proximity_sensor, sim.simx_opmode_blocking)
    
    # If block detected
    if(detectState):
        moveL(clientID, ur3_target, fposition2, 2) # move arm above conveyor belt
        moveL(clientID, ur3_target, fposition3, 2) # move arm onto block
        gripper(clientID, 1, j1, j2) # close gripper
        sim.simxSetObjectParent(clientID, detObjHandle, connector, False, sim.simx_opmode_blocking) # fake grabbing block
        
        # Grab image from vision sensor
        res, resolution, vision_image = sim.simxGetVisionSensorImage(clientID, vision_sensor, 0, sim.simx_opmode_blocking)
        sensor_image = np.array(vision_image, dtype = np.uint8)
        sensor_image.resize([resolution[0], resolution[1], 3])
        time.sleep(1)
        moveL(clientID, ur3_target, fposition2, 2) # move arm above conveyor belt
        time.sleep(1)
        
        # RED
        if (sensor_image[0][0][0] >= 250):
            moveL(clientID, ur3_target, fposition4, 2) #move to red block section
        
        #GREEN
        elif (sensor_image[0][0][1] >= 250):
            moveL(clientID, ur3_target, fposition5, 2) #move to green block section
        
        time.sleep(1)
        sim.simxSetObjectParent(clientID, detObjHandle, -1, False, sim.simx_opmode_blocking) # drop block
        gripper(clientID, 0, j1, j2)

time.sleep(2)

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