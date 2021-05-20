import numpy as np 
import argparse
import math

def carFollowingFvdm(N, l, init_vel, init_acc, tot_t, stp_t):
	sim_stp = int(tot_t/stp_t)
	v_max = 14.6 #max velocity, measured in m/s
	l_safe = 5 #max safety distance
	v_ini = init_vel #initial velocity of the first vehicle in platoon, measured in m/s
	a_ini = init_acc #initial acceleration of the first vehicle, measured in m/s^2
	#position = np.zeros((int(N),sim_stp)) #given position vlaues
	position = [k.strip().split(',    ') for k in open("/Volumes/Fadwas_disk/Alex_research_group/optimal_control/samples_inputs2.dat").readlines()]
	position = position[1:]
	print('position array is'+str(position))
	#headway = np.zeros((N,sim_stp))#headway distance is calculated based on the positions of vehicles
	v_opt = np.zeros((N,sim_stp)) #optimal velocity array
	v = np.zeros((N,sim_stp)) #actual velocity array
	a = np.zeros((N,sim_stp)) #acceleration array
	print('No. of vehicles is'+str(N))
	print('No. of sim steps is'+str(sim_stp))

	for j in range(0, N): #loop for N vehicles
		if j == 0:	
			for i in range(0, sim_stp): #loop around simulation steps for the first vehicle
				if i == 0: 
					print('first time')
					v[j][i] = v_ini
					a[j][i] = a_ini
				elif v[j][i-1]+stp_t*a[j][i-1] >= 0 :#velocity mustn't be negative (the vehicle mustn't be reversing) 
				    v[j][i] = v[j][i-1]+stp_t*a[j][i-1]
				    #headway = position[j][i] - position[j][i-1]
				    s = 1/1+math.exp(l_safe)#since we don't have a preceding vehicle, there is no headway
				    v_opt[j][i] = v_max*(s-l_safe)#since we don't have a preceding vehicle, there is no a leading vehicle speed
				    a[j][i] = 0.41*(v_opt[j][i] - v[j][i]) # since we don't have a preceding vehicle, there is no velocity difference between the curent and the leading vehicle

				else:
					v[j][i] = 0
					a[j][i] = 0
				print('speed of first vehicle'+str(v[j][i]))
		else:
			for i in range(0, sim_stp):

				if i == 0: 
					v[j][i] = v_ini
					a[j][i] = 0

				elif v[j][i-1]+stp_t*a[j][i-1] >= 0 :#velocity mustn't be negative (the vehicle mustn't be reversing) 
				    v[j][i] = v[j][i-1]+stp_t*a[j][i-1]
				    print('i is'+str(i)+'j is'+str(j))
				    headway = float(position[j-1][i]) - float(position[j][i])
				    s = 1/1+math.exp(l_safe-0.7*headway)
				    v_opt[j][i] = v_max*(s-l_safe)+(1-s)*v[j-1][i]
				    a[j][i] = 0.41*(v_opt[j][i] - v[j][i]) + 0.5*(v[j-1][i] - v[j][i])
				else:
					v[j][i] = 0
					a[j][i] = 0
				print('val of following vehicle'+str(j)+'is'+str(v[j][i]))
def main():
	ap = argparse.ArgumentParser()
	# Add the arguments to the parser
	ap.add_argument("-N", "--N_vehicles", required=True,
	   help="")
	ap.add_argument("-l", "--vehicle_length", required=True,
	   help="")
	ap.add_argument("-init_vel", "--init_vel", required=True,
	   help="")
	ap.add_argument("-init_acc", "--init_acc", required=True,
	   help="")
	ap.add_argument("-tot_t", "--tot_t", required=True,
	   help="")
	ap.add_argument("-stp_t", "--stp_t", required=True,
	   help="")
 
	args = vars(ap.parse_args())
	carFollowingFvdm(int(args['N_vehicles']), float(args['vehicle_length']), float(args['init_vel']), float(args['init_acc']), float(args['tot_t']), float(args['stp_t']))

if __name__ == "__main__":
    main()


	

#######notes######
#1: the current velocity v[j][i] can also be calculate as a normal instant velocity: dx/dt
#2: the velocity and acceleration in the first simulation step for all vehicles must be pre-defined.
#3: what is the value of the headway (delta_x) of the fisrt vehicle??
#4: What is the value of delta_v of the first vehicle??
#5: we should consider the speed limit and adpat this algo to decrease the speed in case it exceeds the speed limit.
#6: shall we calculate the time headway besides the space headway 



