# standard libraries
import math
import numpy as np
import scipy.integrate
import time

# custom libraries
from vectors import *
from cost import *

def epsilonTarget(x, y, z, target, epsilon):

	distVect = [target[0]-x,target[1]-y,target[2]-z]
	dist = np.linalg.norm(distVect)
	if(dist < epsilon):
		return True
	else:
		return False

target = np.asfarray([0.0, 0.6, 0.4])

def trajectory(x, y, z, px, py, pz, pxdot, pydot, pzdot):

	start = time.time()

	# N: Event horizon of choice.
	N = 20
	
	# t_s = Sampling time--step through which time is discretized.
	t_s = 0.05

	# M: control horizon
	M = 5

	# linear discretization - (K-1) trajectories will be planned
	K = 9

	# quadcopter velocity
	v_s = 0.5

	# distance tolerance for how close to target
	epsilon = 0.05

	# projectile prediction lists
	pPredict = np.asfarray([[0 for i in range(6)] for j in range(N)])

	# diff eq estimator
	pDE = scipy.integrate.ode(pDot).set_integrator('vode', method='bdf')

	# xi_0: Current vehicle coordinates.
	xi_0 = [x, y, z]
	# p_0: Current projectile coordinates.
	p_0 = [px, py, pz, pxdot, pydot, pzdot]

	x_0 = x
	y_0 = y
	z_0 = 0.4

	# safety net
	R_n = 1.0e-04
	R_d = 1.0e-03
	R_v = 6.0e-02
	R_p = 6.0e-02
	R_col = 2 * (R_n + R_d + R_v + R_p)
	collision = False
	
	# xiPredict: prediction of quadcopter (stays at location)
	# pPredict: projectile state vectors for prediction
	xiPredict = np.asfarray(xi_0)
	pPredict[0][:] = np.asfarray(p_0)

	pDE.set_initial_value(p_0, 0)

    # predict projectile coordinate 
	for j in range(0, N):
		pPredict[j][:] = pDE.integrate(pDE.t + t_s)
		#print(pDE.t)

	

	# collision detection
	for i in range(0, N-1):
		dist_ij = line2line(xiPredict, xiPredict, pPredict[i][0:3], pPredict[i+1][0:3])
		if (dist_ij <= R_col):
			collision = True
			print("collision detected at " + str(i) + " time stamp")
			break


	
	projFile = open("pPredData.txt", 'w')
	np.savetxt('pPredData.txt', pPredict[:,0:3])
	projFile.close()

	"""
	quadFile = open("qData.txt", 'w')
	np.savetxt("qData.txt", xi_0)
	quadFile.close()
	"""

	# trajectory planning
	theta = np.linspace(0, 2 * math.pi, K)	
	U = np.zeros((K, N, 3))

	
	for k in range(0, K):
		# each trajectory starts at quad coordinates

		U[k][0][0:3] = xi_0


		for j in range(1, N):
			U[k][j][0] = x_0 + j * t_s * v_s * math.cos(theta[k])
			U[k][j][1] = y_0 + j * t_s * v_s * math.sin(theta[k])
			U[k][j][2] = z_0




	#trajFile = open("trajdata.txt", 'w')
	#trajFile.write()
	

	# close enough to target and no impending collision 
	if(not collision and epsilonTarget(x_0, y_0, z_0, target.tolist(),epsilon)):
		
		u_opt = target.tolist()

	# impeding collision or not close enough to target
	else:
		Cost_min = 1.0e10
		u_opt = np.asfarray([[0 for j in range(3)] for t in range(N)])
		safeList = []

		# In this for-loop, input trajectories are analyzed for safety with the same collision detection process as above. Unsafe trajectories are discarded.
		for k in range(0, K):
			unSafe = False
			u_k = U[k][:][:]

			
			trajFile = open("trajdata" + str(k) + ".txt", 'w')
			np.savetxt("trajdata" + str(k) + ".txt", u_k.tolist())
			trajFile.close()
			
			# to N-1 because we are doing lines between successive points
			for j in range(0, N-1):

				#print(u_k[j][0:3])
				dist_ij = line2line(u_k[j][0:3], u_k[j + 1][0:3], pPredict[j][0:3], pPredict[j + 1][0:3])
				if (dist_ij) <= R_col:
					print("collision detected in trajectory " + str(k) + " at time " + str(j+1))
					print("distance: " + str(dist_ij))
					unSafe = True
					break

			if(unSafe == False):
				safeList.append(u_k)



		# Here, safe trajectories are optimized regarding minimal distance to the target.
		if (len(safeList) == 0):
			# 
			#	problem if there is nothing in the safeList
			#
			u_opt = target.tolist()
			print("NOTHING IN SAFE LIST")
		else:

			for u in safeList:
				# calculate total cost of trajectory and minimize
				Cost_k = costSum(u, target, N)

				if (Cost_k < Cost_min):
					Cost_min = Cost_k
					u_opt = u[:][M]

	# boundary of motion capture system
	if(u_opt[0] > 1.6):
	    u_opt[0] = 1.6
	elif(u_opt[0] < -1.6):
	    u_opt[0] = -1.6

	if(u_opt[1] > 1.6):
	    u_opt[1] = 1.6
	elif(u_opt[1] < -0.4):
	    u_opt[1] = -0.4

	#print("Optimal: " + str (u_opt))

	ret = list(u_opt)


	for i in range(0,6):
		ret.append(pPredict[M-1,i].tolist())	    

	#print(time.time() - start)

	return ret

