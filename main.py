#!/usr/bin/python -OO

import numpy as np
import hubo_ach as ha
import ach

import fk
import fk_chain
import ik

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

REF_INTERVAL = 0.1
#REF_INTERVAL = 0.01
def ik_control(theta):
	global cur_time
	
	ref.ref[ha.RSP] = theta[0]
	ref.ref[ha.RSR] = theta[1]
	if (theta[2] > 0): theta[2] = -theta[2]
	ref.ref[ha.REB] = theta[2]
	ref.ref[ha.RWP] = theta[3]
	chan_ref.put(ref)
	
	[status, framesize] = chan_state.get(state, wait=False, last=True)
	sim_sleep(cur_time + REF_INTERVAL) # updates state while sleeping
	cur_time = state.time

theta = [0, -np.pi / 3, 0, 0]
max_err = 0.15
theta_step = 0.01
e_step = 0.01

goal = np.array([0.6, 0.0, 0.0])
print "Goal position:", goal

arm_lift()

[status, framesize] = chan_state.get(state, wait=False, last=True)
cur_time = state.time
theta_end = ik.ik_solve(theta, goal, max_err, theta_step, e_step, get_fk_h, ik_control)
print "Final theta:", theta_end
print "Final pose:", get_fk_h(theta_end)

