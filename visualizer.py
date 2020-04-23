import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt
import struct
import numpy as np
import argparse

def main(args):
    # visualize point cloud (obstacles)
    obs = []
    temp=np.fromfile(args.obs_file)
    obs.append(temp)
    obs = np.array(obs).astype(np.float32).reshape(-1,2)
    plt.scatter(obs[:,0], obs[:,1], c='blue')




    # visualize path
    path = np.loadtxt(args.path_file)
    print(path)
    path = path.reshape(-1, 2)
    path_x = []
    path_y = []
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    plt.plot(path_x, path_y, c='r', marker='o')

    plt.show()


parser = argparse.ArgumentParser()
# for training
parser.add_argument('--obs_file', type=str, default='./data/obs_cloud/obc0.dat',help='obstacle point cloud file')
parser.add_argument('--path_file', type=str, default='./results/env_0/path_0.txt',help='path file')
args = parser.parse_args()
print(args)
main(args)
