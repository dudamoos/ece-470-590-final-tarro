#!/usr/bin/python

import numpy as np
import fk_chain
import time

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
def get_next_point_delta(cur, goal, step):
	cur = np.array(cur, copy=False)
	goal = np.array(goal, copy=False)
	m = goal - cur
	m_unit = m / np.linalg.norm(m)
	n = step * m_unit
	# keep in reachable space
	next = cur + n
	if (np.linalg.norm(next) >= 0.75):
		print "\033[91mWent to far!\033[0m", next, "->", np.linalg.norm(next)
		next = 0.7 * next / np.linalg.norm(next) # enforce reachability
		n = next - cur
	elif (np.linalg.norm(next) <= 0.3):
		print "\033[92mHit too close!\033[0m", next, "->", np.linalg.norm(next)
		next = 0.35 * next / np.linalg.norm(next) # enforce reachability
		n = next - cur
	if (np.any(abs(n) > 0.02)):
		print "\033[93mCapping delta!\033[0m", n,
		n = np.max([n, -0.02 * np.ones(len(n))], axis=0)
		n = np.min([n,  0.02 * np.ones(len(n))], axis=0)
		print "->", n
	print "IK moving to", next, "by delta", n, " (dist:", m, ")"
	return n

def null_move(theta): pass

def ik_solve(theta, goal, max_err, theta_step, e_step, fk, move=null_move):
	theta = np.array(theta, copy=False)
	goal = np.array(goal, copy=False)
	e = fk(theta)
	while (get_dist(e, goal) > max_err):
		print "IK distance", get_dist(e, goal)
		J = get_jacobian(fk, theta_step, theta)
		Jp = np.linalg.pinv(J)
		d_e = get_next_point_delta(e, goal, e_step)
		d_theta = Jp.dot(np.transpose([d_e]))
		theta = (theta + np.transpose(d_theta))[0]
		theta = np.mod(theta + np.pi, 2*np.pi) - np.pi # normalize angles
		move(theta)
		e = fk(theta)
		print "IK now at", e, "->", theta
	return theta

