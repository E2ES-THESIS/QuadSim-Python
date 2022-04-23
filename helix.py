from TrajGen import trajGenerator, Helix_waypoints, Circle_waypoints
from Quadrotor import QuadSim
import controller

waypoints = Helix_waypoints(10)

#Generate trajectory through waypoints
traj = trajGenerator(waypoints,max_vel = 1,gamma = 1e6)

#initialise simulation with given controller and trajectory
Tmax = traj.TS[-1]
des_state = traj.get_des_state
sim = QuadSim(controller,des_state,Tmax)
print(waypoints)
#run simulation
sim.run()
