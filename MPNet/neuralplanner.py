import argparse
import torch
import torch.nn as nn
import numpy as np
import os
import pickle
from data_loader import load_test_dataset 
from model import MLP 
from torch.autograd import Variable 
import math
import time

size=5.0

# Load trained model for path generation
mlp = MLP(32, 2) # simple @D
mlp.load_state_dict(torch.load('models/mlp_100_4000_PReLU_ae_dd150.pkl'))

if torch.cuda.is_available():
	mlp.cuda()

#load test dataset
obc,obstacles, paths, path_lengths= load_test_dataset() 

def IsInCollision(x,idx):
	s=np.zeros(2,dtype=np.float32)
	s[0]=x[0]
	s[1]=x[1]
	for i in range(0,7):
		cf=True
		for j in range(0,2):
			if abs(obc[idx][i][j] - s[j]) > size/2.0:
				cf=False
				break
		if cf==True:						
			return True
	return False


def steerTo (start, end, idx):

	DISCRETIZATION_STEP=0.01
	dists=np.zeros(2,dtype=np.float32)
	for i in range(0,2): 
		dists[i] = end[i] - start[i]

	distTotal = 0.0
	for i in range(0,2): 
		distTotal =distTotal+ dists[i]*dists[i]

	distTotal = math.sqrt(distTotal)
	if distTotal>0:
		incrementTotal = distTotal/DISCRETIZATION_STEP
		for i in range(0,2): 
			dists[i] =dists[i]/incrementTotal



		numSegments = int(math.floor(incrementTotal))

		stateCurr = np.zeros(2,dtype=np.float32)
		for i in range(0,2): 
			stateCurr[i] = start[i]
		for i in range(0,numSegments):

			if IsInCollision(stateCurr,idx):
				return 0

			for j in range(0,2):
				stateCurr[j] = stateCurr[j]+dists[j]


		if IsInCollision(end,idx):
			return 0


	return 1

# checks the feasibility of entire path including the path edges
def feasibility_check(path,idx):

	for i in range(0,len(path)-1):
		ind=steerTo(path[i],path[i+1],idx)
		if ind==0:
			return 0
	return 1


# checks the feasibility of path nodes only
def collision_check(path,idx):

	for i in range(0,len(path)):
		if IsInCollision(path[i],idx):
			return 0
	return 1

def to_var(x, volatile=False):
	if torch.cuda.is_available():
		x = x.cuda()
	return Variable(x, volatile=volatile)

def get_input(i,dataset,targets,seq,bs):
	bi=np.zeros((bs,18),dtype=np.float32)
	bt=np.zeros((bs,2),dtype=np.float32)
	k=0	
	for b in range(i,i+bs):
		bi[k]=dataset[seq[i]].flatten()
		bt[k]=targets[seq[i]].flatten()
		k=k+1
	return torch.from_numpy(bi),torch.from_numpy(bt)



def is_reaching_target(start1,start2):
	s1=np.zeros(2,dtype=np.float32)
	s1[0]=start1[0]
	s1[1]=start1[1]

	s2=np.zeros(2,dtype=np.float32)
	s2[0]=start2[0]
	s2[1]=start2[1]


	for i in range(0,2):
		if abs(s1[i]-s2[i]) > 1.0: 
			return False
	return True

#lazy vertex contraction 
def lvc(path,idx):

	for i in range(0,len(path)-1):
		for j in range(len(path)-1,i+1,-1):
			ind=0
			ind=steerTo(path[i],path[j],idx)
			if ind==1:
				pc=[]
				for k in range(0,i+1):
					pc.append(path[k])
				for k in range(j,len(path)):
					pc.append(path[k])

				return lvc(pc,idx)
				
	return path

def re_iterate_path2(p,g,idx,obs):
	step=0
	path=[]
	path.append(p[0])
	for i in range(1,len(p)-1):
		if not IsInCollision(p[i],idx):
			path.append(p[i])
	path.append(g)			
	new_path=[]
	for i in range(0,len(path)-1):
		target_reached=False

	 
		st=path[i]
		gl=path[i+1]
		steer=steerTo(st, gl, idx)
		if steer==1:
			new_path.append(st)
			new_path.append(gl)
		else:
			itr=0
			target_reached=False
			while (not target_reached) and itr<50 :
				new_path.append(st)
				itr=itr+1
				ip=torch.cat((obs,st,gl))
				ip=to_var(ip)
				st=mlp(ip)
				st=st.data.cpu()		
				target_reached=is_reaching_target(st,gl)
			if target_reached==False:
				return 0

	#new_path.append(g)
	return new_path

def replan_path(p,g,idx,obs):
	step=0
	path=[]
	path.append(p[0])
	for i in range(1,len(p)-1):
		if not IsInCollision(p[i],idx):
			path.append(p[i])
	path.append(g)			
	new_path=[]
	for i in range(0,len(path)-1):
		target_reached=False

	 
		st=path[i]
		gl=path[i+1]
		steer=steerTo(st, gl, idx)
		if steer==1:
			new_path.append(st)
			new_path.append(gl)
		else:
			itr=0
			pA=[]
			pA.append(st)
			pB=[]
			pB.append(gl)
			target_reached=0
			tree=0
			while target_reached==0 and itr<50 :
				itr=itr+1
				if tree==0:
					ip1=torch.cat((obs,st,gl))
					ip1=to_var(ip1)
					st=mlp(ip1)
					st=st.data.cpu()
					pA.append(st)
					tree=1
				else:
					ip2=torch.cat((obs,gl,st))
					ip2=to_var(ip2)
					gl=mlp(ip2)
					gl=gl.data.cpu()
					pB.append(gl)
					tree=0		
				target_reached=steerTo(st, gl, idx)
			if target_reached==0:
				return 0
			else:
				for p1 in range(0,len(pA)):
					new_path.append(pA[p1])
				for p2 in range(len(pB)-1,-1,-1):
					new_path.append(pB[p2])

	return new_path	
    
def main(args):
	# Create model directory
	if not os.path.exists(args.model_path):
		os.makedirs(args.model_path)
	

	
	tp=0
	fp=0
	tot=[]
	for i in range(0,1):
		et=[]
		for j in range(0,2):
			print ("step: i="+str(i)+" j="+str(j))
			p1_ind=0
			p2_ind=0
			p_ind=0	
			if path_lengths[i][j]>0:								
				start=np.zeros(2,dtype=np.float32)
				goal=np.zeros(2,dtype=np.float32)
				for l in range(0,2):
					start[l]=paths[i][j][0][l]
				
				for l in range(0,2):
					goal[l]=paths[i][j][path_lengths[i][j]-1][l]
				#start and goal for bidirectional generation
				## starting point
				start1=torch.from_numpy(start)
				goal2=torch.from_numpy(start)
				##goal point
				goal1=torch.from_numpy(goal)
				start2=torch.from_numpy(goal)
				##obstacles
				obs=obstacles[i]
				obs=torch.from_numpy(obs)
				##generated paths
				path1=[] 
				path1.append(start1)
				path2=[]
				path2.append(start2)
				path=[]
				target_reached=0
				step=0	
				path=[] # stores end2end path by concatenating path1 and path2
				tree=0	
				tic = time.clock()	
				while target_reached==0 and step<80 :
					step=step+1
					if tree==0:
						inp1=torch.cat((obs,start1,start2))
						inp1=to_var(inp1)
						start1=mlp(inp1)
						start1=start1.data.cpu()
						path1.append(start1)
						tree=1
					else:
						inp2=torch.cat((obs,start2,start1))
						inp2=to_var(inp2)
						start2=mlp(inp2)
						start2=start2.data.cpu()
						path2.append(start2)
						tree=0
					target_reached=steerTo(start1,start2,i);
				tp=tp+1

				if target_reached==1:
					for p1 in range(0,len(path1)):
						path.append(path1[p1])
					for p2 in range(len(path2)-1,-1,-1):
						path.append(path2[p2])
													
					
					path=lvc(path,i)
					indicator=feasibility_check(path,i)
					if indicator==1:
						toc = time.clock()
						t=toc-tic
						et.append(t)
						fp=fp+1
						print ("path[0]:")
						for p in range(0,len(path)):
							print (path[p][0])
						print ("path[1]:")
						for p in range(0,len(path)):
							print (path[p][1])
						print ("Actual path[0]:")
						for p in range(0,path_lengths[i][j]):
							print (paths[i][j][p][0])
						print ("Actual path[1]:")
						for p in range(0,path_lengths[i][j]):
							print (paths[i][j][p][1])
					else:
						sp=0
						indicator=0
						while indicator==0 and sp<10 and path !=0:
							sp=sp+1
							g=np.zeros(2,dtype=np.float32)
							g=torch.from_numpy(paths[i][j][path_lengths[i][j]-1])
							path=replan_path(path,g,i,obs) #replanning at coarse level
							if path !=0:
								path=lvc(path,i)
								indicator=feasibility_check(path,i)
					
							if indicator==1:
								toc = time.clock()
								t=toc-tic
								et.append(t)
								fp=fp+1
								if len(path)<20:
									print ("new_path[0]:")
									for p in range(0,len(path)):
										print (path[p][0])
									print ("new_path[1]:")
									for p in range(0,len(path)):
										print (path[p][1])
									print ("Actual path[0]:")
									for p in range(0,path_lengths[i][j]):
										print (paths[i][j][p][0])
									print ("Actual path[1]:")
									for p in range(0,path_lengths[i][j]):
										print (paths[i][j][p][1])
								else:
									print "path found, dont worry"	

				
		tot.append(et)					
	pickle.dump(tot, open("time_s2D_unseen_mlp.p", "wb" ))	


	print ("total paths")
	print (tp)
	print ("feasible paths")
	print (fp)

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--model_path', type=str, default='./models/',help='path for saving trained models')
	parser.add_argument('--no_env', type=int, default=50,help='directory for obstacle images')
	parser.add_argument('--no_motion_paths', type=int,default=2000,help='number of optimal paths in each environment')
	parser.add_argument('--log_step', type=int , default=10,help='step size for prining log info')
	parser.add_argument('--save_step', type=int , default=1000,help='step size for saving trained models')

	# Model parameters
	parser.add_argument('--input_size', type=int , default=68, help='dimension of the input vector')
	parser.add_argument('--output_size', type=int , default=2, help='dimension of the input vector')
	parser.add_argument('--hidden_size', type=int , default=256, help='dimension of lstm hidden states')
	parser.add_argument('--num_layers', type=int , default=4, help='number of layers in lstm')

	parser.add_argument('--num_epochs', type=int, default=100)
	parser.add_argument('--batch_size', type=int, default=28)
	parser.add_argument('--learning_rate', type=float, default=0.001)
	args = parser.parse_args()
	print(args)
	main(args)


