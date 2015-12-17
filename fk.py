import numpy as np
from numpy import sin, cos

# FK functions

def getR(theta):
	"""
	Calculates the composite rotation matrix for the rotation described by
	theta. Each axis rotation becomes a rotation matrix and these are multiplied
	to get the final rotation matrix.

	@param theta - 3x1 vector of rotations about each axis
	@return 3x3 composite rotation matrix
	"""
	Rx = np.array([[  1            ,  0            ,  0             ],
	               [  0            ,  cos(theta[0]), -sin(theta[0]) ],
	               [  0            ,  sin(theta[0]),  cos(theta[0]) ]])
	
	Ry = np.array([[  cos(theta[1]),  0            ,  sin(theta[1]) ],
	               [  0            ,  1            ,  0             ],
	               [ -sin(theta[1]),  0            ,  cos(theta[0]) ]])
	
	Rz = np.array([[  cos(theta[2]), -sin(theta[2]),  0             ],
	               [  sin(theta[2]),  cos(theta[2]),  0             ],
	               [  0            ,  0            ,  1             ]])
	
	return Rx.dot(Ry).dot(Rz)

def getT(theta, pick_lr):
	"""
	@param theta - vector of joint angles in arm
	@param pick_lr - kinematic chain for chosen limb
	@return 4x4 transformation matrix for end effector relative to base
	"""
	assert len(theta) == (len(pick_lr) - 1), "joints don't match kinematic model"
	all_T = []
	if (type(theta) == list): theta = theta[:] # copy the list
	if (type(theta) == np.ndarray): theta = theta.tolist()
	theta.append(0) # append dummy zero for end effector

	for entry in zip(theta, pick_lr): # (theta[i], Kin[i] { P, THETA_rot })
		T = np.zeros((4, 4))
		T[3, 3] = 1
		T[0:3:1, 3] = entry[1][0]
		T[0:3:1, 0:3:1] = getR(entry[0] * np.array(entry[1][1], copy=False))
		all_T.append(T)
	
	return reduce(np.dot, all_T) # dot(x, y) does matrix multiplication too

def get_pos(T, coords_to_get="x y z"):
	"""
	@param T - the transformation matrix for the entire set of limbs you
	           are modelling
	@param coords_to_get - a string containing the names of the coords you
	                       want the function to return values for
	@return the requested coords of the end effector modelled by
	"""
	P = T[0:3:1, 3]
	R = T[0:3:1, 0:3:1]
	ret_coords = []

	coords_to_get = coords_to_get.strip().lower()
	if (coords_to_get == ""): coords_to_get = "x y z thx thy thz"
	for coord in coords_to_get.split():
		if coord == "x": ret_coords.append(P[0])
		elif coord == "y": ret_coords.append(P[1])
		elif coord == "z": ret_coords.append(P[2])
		elif coord == "thx": ret_coords.append(np.arctan2(R[2, 1], R[2, 2]))
		elif coord == "thy":
			ret_coords.append(atan2(R[2, 0], R[2,1]**2 + R[2, 2]**2))
		elif coord == "thz": ret_coords.append(np.arctan2(R[1, 0], R[0, 0]))
	return np.array(ret_coords)

def get_fk(theta, pick_lr):
	"""
	@param theta - vector of joint angles in arm
	@param pick_lr - kinematic chain for chosen limb
	@return position in space of the end effector when joints are as theta
	"""
	return get_pos(getT(theta, pick_lr), "x y z")

