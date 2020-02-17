# ece470finalproject

Robot Motion (WIP)

Using the provided 'example.py' file from the class, we have been able to program a UR3 robot arm to move to three different waypoints with the remote python api.

In order to run this demo:
- Download and install CoppeliaSim if you haven't already.
- Download and install Anaconda3 (which comes with the Spyder IDE) if you haven't already.
- Open the "ece470_sim.ttt" scene.
- Open "example.py" in the Spyder IDE.
- In the LUA command box within CoppeliaSim, run "simExtRemoteApiStart (19999)".
- In Spyder, run the "example.py" code.
- Observe in the CoppeliaSim window that the arm moves.

Camera Vision (WIP)

*insert stuff here*

Proximity Sensor (WIP)

Currently the proximity sensor is not fully set up. However, the CoppeliaSim scene "trash-collection.ttt" contains the work that we have so far.
This work includes a UR3 arm with a non-functional proximity sensor, as well as a laptop placed above the sensor's range. Ideally, the laptop
will fall through and generate a reading from the proximity sensor, to help demonstrate functionality. As of now, the laptop falls, and we have
a graph of how far the laptop is from the robot during this fall, but we have not yet linked this to python code.

In order to run this demo:
- Download and install CoppeliaSim if you haven't already.
- Open the "trash-collection.ttt" scene
- At the top bar click Simulation -> Start simulation
- Observe the results on the graph within CoppeliaSim.
- Click Simulation -> Stop simulation