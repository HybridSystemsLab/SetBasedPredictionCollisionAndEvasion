# standard libraries
import trajectoryplanner
import numpy as np
import time
import math
import scipy.integrate
from random import *


# custom libraries
from vectors import *
from cost import *


def propagate(destCoord, curCoord, distTraveled):
	xDistToCommand = destCoord[0] - curCoord[0]
	yDistToCommand = destCoord[1] - curCoord[1]

	angle = np.arctan2(yDistToCommand,xDistToCommand)

	xTraveled = distTraveled * np.cos(angle)
	yTraveled = distTraveled * np.sin(angle)

	quadCoord = [0,0,0]
	
	quadCoord[0] = xTraveled + curCoord[0]
	quadCoord[1] = yTraveled + curCoord[1]
	quadCoord[2] = destCoord[2]

	return quadCoord


if __name__ == '__main__':

	# matlab files
	quadfile = open("quadData.txt", 'w')
	quadfile.close()
	projfile = open("projData.txt", 'w')
	projfile.close()
	timefile = open("timeData.txt", 'w')
	timefile.close()
	tDistFile = open("targetDistance.txt", 'w')
	tDistFile.close()
	pDistFile = open("projDistance.txt", 'w')
	pDistFile.close()

	######################
	# initial conditions #
	######################

	target = [0.0, 0.6, 0.4]

	# PAPER sim
	quadCoord = [0.8, -0.4, 0.4]
	projCoord = [-0.5, 1.0, 0.4]	
	projVel = [0.27, -0.24, 11]

	# avoids and converges
	#quadCoord = [-0.5, 0.1, 0.4]
	#projCoord = [0.6, 1.2, 0.4]
	#projVel = [-1.0, -1.0, 4.5]

	# avoids and converges
	#quadCoord = [-0.6, 0.2, 0.4]
	#projCoord = [0.6, 1.2, -0.6]
	#projVel = [-1.0, -1.0, 5.5]

	# avoids and converges
	#quadCoord = [-0.4, 0.3, 0.4]
	#projCoord = [0.4, 1.2, 0.0]
	#projVel = [-1.1, -1.2, 3.5]
	
	# projectile passes safely over
	#quadCoord = [0.8, -0.4, 0.4]
	#projCoord = [-0.6, -1.2, 0.4]	
	#projVel = [0.8, 0.8, 8.5]

	# narrow avoid and converges
	#quadCoord = [0.9, -0.4, 0.4]
	#projCoord = [-0.5, 1.0, 0.4]	
	#projVel = [0.50, -0.48, 7.5]

	# nice avoid at the end and converges
	#quadCoord = [0.6, -0.4, 0.4]
	#projCoord = [-0.47, 0.98, 0.4]	
	#projVel = [0.27, -0.24, 9.82]

	# low vert velocity
	#quadCoord = [0.8, -0.4, 0.4]
	#projCoord = [-0.47, 0.98, -4.2]	
	#projVel = [0.27, -0.24, 12]

	# avoids near target at the end
	#quadCoord = [1, 1, 0.4]
	#projCoord = [-0.5, 1.0, 0.4]	
	#projVel = [0.27, -0.24, 10]

	# avoid from side projectile 
	#quadCoord = [-1.2, 0.4, 0.4]
	#projCoord = [-0.5, 0.0, 0.4]	
	#projVel = [-0.1, 0.3, 10]

	# narrow avoid and converges
	#quadCoord = [0.8, -0.4, 0.4]
	#projCoord = [-0.5, 1.0, 0.0]	
	#projVel = [0.50, -0.48, 8.5]

	# avoid from beginning
	#quadCoord = [0.8, -0.4, 0.4]
	#projCoord = [-0.5, 1.0, 3.2]
	#projVel = [1.50, -1.58, 0.5]

	# avoid from beginning
	#quadCoord = [0.8, -0.4, 0.4]
	#projCoord = [-0.5, 1.0, 6.2]
	#projVel = [0.70, -0.74, 0.5]

	# avoid from beginning
	#quadCoord = [0.8, -0.4, 0.4]
	#projCoord = [0.5, -1.0, 5.9]
	#projVel = [0.0, 0.84, 0.5]

	# quad at target 
	#quadCoord = [0.0, 0.6, 0.4]
	#projCoord = [-1.0, 0.0, 0.4]	
	#projVel = [0.95, 0.65, 5.0]

	# at target
	#quadCoord = [0.0, 0.6, 0.4]
	#projCoord = [1.0, 0.0, 0.4]	
	#projVel = [-0.95, 0.75, 5.0]

	# at target
	#quadCoord = [0.0, 0.6, 0.4]
	#projCoord = [1.0, 0.5, 0.4]	
	#projVel = [-0.95, 0.1, 4.0]

	# at target
	#quadCoord = [0.0, 0.6, 0.4]
	#projCoord = [0.2, 0.4, 0.76]	
	#projVel = [-0.1, 0.2, 7.0]

	# simulation
	speed = 0.5 #m/s
	curTime = 0 

	# proportional constant
	p = 1

	#print("Time: " + str(curTime))
	print("Quad: " + str(quadCoord))

	targetDist = np.linalg.norm(np.subtract(quadCoord,[0.0, 0.6, 0.4]))
	projDist = np.linalg.norm(np.subtract(quadCoord,projCoord))

	


	

	# files to matlab
	quadfile = open("quadData.txt", 'a')
	qCoordStr = ' '.join(map(str, quadCoord))
	quadfile.write(qCoordStr)
	quadfile.write("\n")
	quadfile.close()

	projfile = open("projData.txt", 'a')
	pCoordStr = ' '.join(map(str, projCoord))
	projfile.write(pCoordStr)
	projfile.write("\n")
	projfile.close()

	timefile = open("timeData.txt", 'a')
	timefile.write(str(curTime))
	timefile.write("\n")
	timefile.close()

	tDistFile = open("targetDistance.txt", 'a')
	tDistFile.write(str(targetDist))
	tDistFile.write('\n')
	tDistFile.close()

	pDistFile = open("projDistance.txt", 'a')
	pDistFile.write(str(projDist))
	pDistFile.write('\n')
	pDistFile.close()


	# iterate to simulate dynamic environment
	for i in range(0,70):
		stamp = time.time()

		# optimization
		u_opt = trajectoryplanner.trajectory(quadCoord[0],quadCoord[1],quadCoord[2],projCoord[0],projCoord[1],projCoord[2],projVel[0],projVel[1],projVel[2])
		
		# diff eq estimator
		pDE = scipy.integrate.ode(pDot).set_integrator('vode', method='bdf')
		state = np.append(projCoord,projVel)
		pDE.set_initial_value(state, 0)

		# computation time
		timeElapsed = time.time() - stamp
		


		# equalize time step to optimization run time
		if(timeElapsed < 0.05):

			extraTime = 0.05 - timeElapsed
			timeElapsed = timeElapsed + extraTime

		curTime = curTime+timeElapsed

		# calculate quads new position
		distTraveled = speed*timeElapsed

		# hacky
		#if(np.linalg.norm(np.subtract(quadCoord,target)) < 0.02):
		#	quadCoord = target
		#	u_opt = target
		#else:
		#	quadCoord = propagate(u_opt, quadCoord, distTraveled)
		
		quadCoord = propagate(u_opt, quadCoord, distTraveled)
		

		# calculates projectiles new position
		projState = pDE.integrate(pDE.t + timeElapsed)
		
		projCoord = projState[0:3]
		projVel = projState[3:6]


		targetDist = np.linalg.norm(np.subtract(quadCoord,[0.0, 0.6, 0.4]))
		projDist = np.linalg.norm(np.subtract(quadCoord,projCoord))

		
	
		print("Time: " + str(i))
		print("Dest: " + str(u_opt[0:3]))
		print("Quad: " + str(quadCoord))
		

		# files to matlab
		quadfile = open("quadData.txt", 'a')
		qCoordStr = ' '.join(map(str, quadCoord))
		quadfile.write(qCoordStr)
		quadfile.write("\n")
		quadfile.close()

		projfile = open("projData.txt", 'a')
		pCoordStr = ' '.join(map(str, projCoord))
		projfile.write(pCoordStr)
		projfile.write("\n")
		projfile.close()

		timefile = open("timeData.txt", 'a')
		timefile.write(str(curTime))
		timefile.write("\n")
		timefile.close()

		tDistFile = open("targetDistance.txt", 'a')
		tDistFile.write(str(targetDist))
		tDistFile.write('\n')
		tDistFile.close()

		pDistFile = open("projDistance.txt", 'a')
		pDistFile.write(str(projDist))
		pDistFile.write('\n')
		pDistFile.close()


			
		

		

