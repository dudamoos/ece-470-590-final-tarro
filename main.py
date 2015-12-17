#!/usr/bin/python -OO

import numpy as np
import hubo_ach as ha
import ach

import fk
import fk_chain
import ik
#import path

# assignment:
#  robot on ground at (x,y) = (0, 0), red box located at (x,y,z) = (1.5, 0, 1)
#  make robot walk up to red box and touch it with its right hand and remain standing

# path planner plans ((hx, hy, hz), (lfx, lfy, lfz), (rfx, rfy, rfz))
# control loop sends each path point to IK
# IK solves for desired point from current pose
# robot sends current pose to IK and control loop

chan_state = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
chan_ref = ach.Channel(ha.HUBO_CHAN_REF_NAME)
state = ha.HUBO_STATE()
ref = ha.HUBO_REF()

def sim_sleep(until):
	while (state.time < until):
		[status, framesize] = chan_state.get(state, wait=True, last=True)

def arm_lift():
	[status, framesize] = chan_state.get(state, wait=False, last=True)
	time_init = state.time
	time_last = time_init + 1
	while True:
		[status, framesize] = chan_state.get(state, wait=False, last=True)
		time_cur = state.time
		if (time_cur >= time_last): break
		theta = (np.pi / 6) * (1 - np.cos(np.pi * (time_cur - time_init)))
		ref.ref[ha.RSR] = -theta
		ref.ref[ha.LSR] = theta
		chan_ref.put(ref)
		[status, framesize] = chan_state.get(state, wait=False, last=True)
		sim_sleep(time_cur + 0.1)
	ref.ref[ha.RSR] = -np.pi / 3
	ref.ref[ha.LSR] = np.pi / 3
	chan_ref.put(ref)

def get_fk_h(theta):
	return fk.get_fk(theta, fk_chain.KRH)

#REF_INTERVAL = 0.1
REF_INTERVAL = 0.01
def ik_control(theta):
	global cur_time
	
	ref.ref[ha.RSP] = theta[0]
	ref.ref[ha.RSR] = theta[1]
	#ref.ref[ha.RSY] = theta[2]
	#if (theta[2] > 0): theta[2] = 0
	if (theta[2] > 0): theta[2] = -theta[2]
	ref.ref[ha.REB] = theta[2]
	#ref.ref[ha.RWY] = theta[3]
	ref.ref[ha.RWP] = theta[3]
	#ref.ref[ha.RWR] = theta[4]
	chan_ref.put(ref)
	
	[status, framesize] = chan_state.get(state, wait=False, last=True)
	sim_sleep(cur_time + REF_INTERVAL) # updates state while sleeping
	cur_time = state.time

theta = [0, -np.pi / 3, 0, 0]
#max_err = 0.01 # 1% of max radius the 2d arm can reach
max_err = 0.035 # 5% of max radius each 3d arm can reach
max_err = 0.15
theta_step = 0.01
#e_step = 0.003 # 1/2 of max_err
e_step = 0.01

goal = np.array([0.6, 0.0, 0.0]) # approximately pointing at the box
#goal_arm = goal - fk_chain.K_RSP[0] # ensure in reachable range
#goal_arm = 0.65 * goal_arm / np.linalg.norm(goal_arm)
#goal = goal_arm + fk_chain.K_RSP[0] # reachable goal
print "Goal position:", goal # approximately [ 0.60350985  0.00140394  0.        ]
#goal = np.array([-0.65, -0.1, -0.3]) # backwards is forwards?
# this point is approximately the result of theta = [-1.4, 0.2, 0, 0, 0, 0, 0]

arm_lift()

[status, framesize] = chan_state.get(state, wait=False, last=True)
cur_time = state.time
theta_end = ik.ik_solve(theta, goal, max_err, theta_step, e_step, get_fk_h, ik_control)
print "Final theta:", theta_end
print "Final pose:", get_fk_h(theta_end)
#ik_control(theta_end)

#def execute_path(path, fk, max_err):
#	index = 0
#	while True:
#		cur_time = get_time()
#		next_time = time + 0.1
#		cur_pose = get_pose()
#		
#		if (get_dist(fk(cur_pose), fk(path[index])) < max_err):
#			index += 1
#			if (index >= len(path)): break
#			send_to_ik(cur_pose, path[index])
#		
#		cur_time = get_time()
#		sim_sleep(next_time - cur_time)

