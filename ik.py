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
		theta_new[j] += d_theta
		d_e = fk(theta_new) - pose
		J[:,j] = d_e / d_theta
	return J

# simple path planning - straight line towards goal
# do more complex planning by IK solving for points along the intended path
def get_next_point_delta(cur, goal, step):
	cur = np.array(cur, copy=False)
	goal = np.array(goal, copy=False)
	m = goal - cur
	m_unit = m / np.linalg.norm(m)
	return step * m_unit

def ik_solve(theta, goal, max_err, theta_step, e_step, fk):
	theta = np.array(theta, copy=False)
	goal = np.array(goal, copy=False)
	e = fk(theta)
	while (get_dist(e, goal) > max_err):
		J = get_jacobian(fk, theta_step, theta)
		Jp = np.linalg.pinv(J)
		d_e = get_next_point_delta(e, goal, e_step)
		d_theta = Jp.dot(np.transpose([d_e]))
		theta = (theta + np.transpose(d_theta))[0]
		e = fk(theta)
	return np.mod(theta + pi, 2*pi) - pi # normalize angles

