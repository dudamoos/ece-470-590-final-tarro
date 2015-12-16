#!/usr/bin/python

import numpy as np

# IK functions

def get_dist(p1, p2):
	p1 = np.array(p1, copy=False)
	p2 = np.array(p2, copy=False)
	return np.linalg.norm(p1 - p2)

def get_jacobian(fk, d_theta, theta):
	pose = fk(theta)
	J = np.zeros((len(pose), len(theta)))
	for j in range(len(theta)): # robot DOF
		theta_new = np.array(theta)
		theta_new[j] = max(min(theta_new[j] + d_theta, np.pi), -np.pi)
		d_e = fk(theta_new) - pose
		J[:,j] = d_e / d_theta
	return J

# simple path planning - straight line towards goal
# do more complex planning by IK solving for points along the intended path
def get_next_point_delta(cur, goal, step, snap_region):
	cur = np.array(cur, copy=False)
	goal = np.array(goal, copy=False)
	m = goal - cur
	m_unit = m / np.linalg.norm(m)
	n = step * m_unit
	for i in range(len(m)):
		if (m[i] < snap_region): n[i] = m[i]
	print "IK moving by delta", n
	return n

def null_move(theta): pass

def ik_solve(theta, goal, max_err, theta_step, e_step, fk, move=null_move):
	global th
	global pos
	global iter_count
	
	th = theta = np.array(theta, copy=False)
	goal = np.array(goal, copy=False)
	pos = e = fk(theta)
	iter_count = 0
	while (get_dist(e, goal) > max_err):
		J = get_jacobian(fk, theta_step, theta)
		Jp = np.linalg.pinv(J)
		d_e = get_next_point_delta(e, goal, e_step, max_err)
		d_theta = Jp.dot(np.transpose([d_e]))
		theta = (theta + np.transpose(d_theta))[0]
		#theta = np.max([theta, -(np.pi/2) * np.ones(len(theta))], axis=0)
		#theta = np.min([theta,  (np.pi/2) * np.ones(len(theta))], axis=0)
		th = theta = np.mod(theta + np.pi, 2*np.pi) - np.pi # normalize angles
		pos = e = fk(theta)
		print "IK now at", e, "->", theta
		move(theta)
		iter_count += 1
	return theta # np.mod(theta + np.pi, 2*np.pi) - np.pi # normalize angles

