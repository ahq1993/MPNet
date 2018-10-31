import argparse
import os
import torch
import torchvision
from torch import nn
from torch.autograd import Variable
from data_loader import load_dataset


class Encoder(nn.Module):
	def __init__(self):
		super(Encoder, self).__init__()
		self.encoder = nn.Sequential(nn.Linear(2800, 512),nn.PReLU(),nn.Linear(512, 256),nn.PReLU(),nn.Linear(256, 128),nn.PReLU(),nn.Linear(128, 28))
			
	def forward(self, x):
		x = self.encoder(x)
		return x

class Decoder(nn.Module):
	def __init__(self):
		super(Decoder, self).__init__()
		self.decoder = nn.Sequential(nn.Linear(28, 128),nn.PReLU(),nn.Linear(128, 256),nn.PReLU(),nn.Linear(256, 512),nn.PReLU(),nn.Linear(512, 2800))
	def forward(self, x):
		x = self.decoder(x)
		return x



mse_loss = nn.MSELoss()
lam=1e-3
def loss_function(W, x, recons_x, h):
	mse = mse_loss(recons_x, x)
	"""
	W is shape of N_hidden x N. So, we do not need to transpose it as opposed to http://wiseodd.github.io/techblog/2016/12/05/contractive-autoencoder/
	"""
	dh = h*(1-h) # N_batch x N_hidden
	contractive_loss = torch.sum(Variable(W)**2, dim=1).sum().mul_(lam)
	return mse + contractive_loss


def main(args):	
	
	if not os.path.exists(args.model_path):
		os.makedirs(args.model_path)


	obs = load_dataset()

	encoder = Encoder()
	decoder = Decoder()
	if torch.cuda.is_available():
		encoder.cuda()
		decoder.cuda()

	
	params = list(encoder.parameters())+list(decoder.parameters())
	optimizer = torch.optim.Adagrad(params)
	total_loss=[]
	for epoch in range(args.num_epochs):
		print "epoch" + str(epoch)
		avg_loss=0
		for i in range(0, len(obs), args.batch_size):
			decoder.zero_grad()
			encoder.zero_grad()
			if i+args.batch_size<len(obs):
				inp = obs[i:i+args.batch_size]
			else:
				inp = obs[i:]
			inp=torch.from_numpy(inp)
			inp =Variable(inp).cuda()
			# ===================forward=====================
			h = encoder(inp)
			output = decoder(h)
			keys=encoder.state_dict().keys()
			W=encoder.state_dict()['encoder.6.weight'] # regularize or contracting last layer of encoder. Print keys to displace the layers name. 
			loss = loss_function(W,inp,output,h)
			avg_loss=avg_loss+loss.data[0]
			# ===================backward====================
			loss.backward()
			optimizer.step()
		print "--average loss:"
		print avg_loss/(len(obs)/args.batch_size)
		total_loss.append(avg_loss/(len(obs)/args.batch_size))

	avg_loss=0
	for i in range(len(obs)-5000, len(obs), args.batch_size):
		inp = obs[i:i+args.batch_size]
		inp=torch.from_numpy(inp)
		inp =Variable(inp).cuda()
		# ===================forward=====================
		output = encoder(inp)
		output = decoder(output)
		loss = mse_loss(output,inp)
		avg_loss=avg_loss+loss.data[0]
		# ===================backward====================
	print "--Validation average loss:"
	print avg_loss/(5000/args.batch_size)


    
	torch.save(encoder.state_dict(),os.path.join(args.model_path,'cae_encoder.pkl'))
	torch.save(decoder.state_dict(),os.path.join(args.model_path,'cae_decoder.pkl'))
	torch.save(total_loss,'total_loss.dat')


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--model_path', type=str, default='./models/',help='path for saving trained models')
	parser.add_argument('--no_env', type=int, default=50,help='directory for obstacle images')
	parser.add_argument('--no_motion_paths', type=int,default=2000,help='number of optimal paths in each environment')
	parser.add_argument('--log_step', type=int , default=10,help='step size for prining log info')
	parser.add_argument('--save_step', type=int , default=1000,help='step size for saving trained models')

	# Model parameters
	parser.add_argument('--input_size', type=int , default=18, help='dimension of the input vector')
	parser.add_argument('--output_size', type=int , default=2, help='dimension of the input vector')
	parser.add_argument('--hidden_size', type=int , default=256, help='dimension of lstm hidden states')
	parser.add_argument('--num_layers', type=int , default=4, help='number of layers in lstm')

	parser.add_argument('--num_epochs', type=int, default=400)
	parser.add_argument('--batch_size', type=int, default=100)
	parser.add_argument('--learning_rate', type=float, default=0.001)
	args = parser.parse_args()
	print(args)
	main(args)
