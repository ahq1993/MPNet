import argparse
import torch
import torch.nn as nn
import numpy as np
import os
import pickle
from data_loader import load_dataset 
from model import MLP 
from torch.autograd import Variable 
import math

def to_var(x, volatile=False):
	if torch.cuda.is_available():
		x = x.cuda()
	return Variable(x, volatile=volatile)

def get_input(i,data,targets,bs):

	if i+bs<len(data):
		bi=data[i:i+bs]
		bt=targets[i:i+bs]	
	else:
		bi=data[i:]
		bt=targets[i:]
		
	return torch.from_numpy(bi),torch.from_numpy(bt)


    
def main(args):
	# Create model directory
	if not os.path.exists(args.model_path):
		os.makedirs(args.model_path)
    
    
	# Build data loader
	dataset,targets= load_dataset() 
	
	# Build the models
	mlp = MLP(args.input_size, args.output_size)
    
	if torch.cuda.is_available():
		mlp.cuda()

	# Loss and Optimizer
	criterion = nn.MSELoss()
	optimizer = torch.optim.Adagrad(mlp.parameters()) 
    
	# Train the Models
	total_loss=[]
	print len(dataset)
	print len(targets)
	sm=100 # start saving models after 100 epochs
	for epoch in range(args.num_epochs):
		print "epoch" + str(epoch)
		avg_loss=0
		for i in range (0,len(dataset),args.batch_size):
			# Forward, Backward and Optimize
			mlp.zero_grad()			
			bi,bt= get_input(i,dataset,targets,args.batch_size)
			bi=to_var(bi)
			bt=to_var(bt)
			bo = mlp(bi)
			loss = criterion(bo,bt)
			avg_loss=avg_loss+loss.data[0]
			loss.backward()
			optimizer.step()
		print "--average loss:"
		print avg_loss/(len(dataset)/args.batch_size)
		total_loss.append(avg_loss/(len(dataset)/args.batch_size))
		# Save the models
		if epoch==sm:
			model_path='mlp_100_4000_PReLU_ae_dd'+str(sm)+'.pkl'
			torch.save(mlp.state_dict(),os.path.join(args.model_path,model_path))
			sm=sm+50 # save model after every 50 epochs from 100 epoch ownwards
	torch.save(total_loss,'total_loss.dat')
	model_path='mlp_100_4000_PReLU_ae_dd_final.pkl'
	torch.save(mlp.state_dict(),os.path.join(args.model_path,model_path))
if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--model_path', type=str, default='./models/',help='path for saving trained models')
	parser.add_argument('--no_env', type=int, default=50,help='directory for obstacle images')
	parser.add_argument('--no_motion_paths', type=int,default=2000,help='number of optimal paths in each environment')
	parser.add_argument('--log_step', type=int , default=10,help='step size for prining log info')
	parser.add_argument('--save_step', type=int , default=1000,help='step size for saving trained models')

	# Model parameters
	parser.add_argument('--input_size', type=int , default=32, help='dimension of the input vector')
	parser.add_argument('--output_size', type=int , default=2, help='dimension of the input vector')
	parser.add_argument('--hidden_size', type=int , default=256, help='dimension of lstm hidden states')
	parser.add_argument('--num_layers', type=int , default=4, help='number of layers in lstm')

	parser.add_argument('--num_epochs', type=int, default=500)
	parser.add_argument('--batch_size', type=int, default=100)
	parser.add_argument('--learning_rate', type=float, default=0.0001)
	args = parser.parse_args()
	print(args)
	main(args)



