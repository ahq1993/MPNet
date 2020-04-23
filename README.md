# Motion Planning Networks
Implementation of [MPNet: Motion Planning Networks](https://sites.google.com/view/mpnet). [[arXiv]](https://arxiv.org/abs/1806.05767)  



The code can easily be adapted for [Informed Neural Sampling](https://arxiv.org/abs/1809.10252).

## Contains
* Data Generation
	* Any existing classical motion planner can be used to generate datasets. However, we provide following implementations in C++:
		* [P-RRT*](https://link.springer.com/article/10.1007/s10514-015-9518-0)
		* [RRT*](https://arxiv.org/abs/1105.1186)
		* Example dataset: [simple2D](https://drive.google.com/open?id=1oADJ85qxb3WKHXE4Bj6lwio-soGOktRa)
		* Example dataset: [Complex3D]
* MPNet algorithm
* A navie python visualization files


## Data Description
* Simple 2D has 7 blocks each of size 5x5 that are placed randomly.
* Complex 3D contains 10 blocks with sizes as follow:
	* shape=[[5.0,5.0,10.0],[5.0,10.0,5.0],[5.0,10.0,10.0],
              [10.0,5.0,5.0],[10.0,5.0,10.0],[10.0,10.0,5.0],
              [10.0,10.0,10.0],[5.0,5.0,5.0],[10.0,10.0,10.0],[5.0,5.0,5.0]]
* e0-109 has the training and testing paths in 110 different environments.
	* 0-100 environments and 0-4000 paths/environment are for training.
	* Seen test dataset: 0-100 envs and 4000-4200=200 paths/env.
	* Unseen test dataset: 100-110 envs and 0-2000 paths/env.
* obs_cloud is the point-cloud of randomly generated 30,000 environments.
	* 0-110 corresponds to the same environments for which path data is provided.
	* You may use full dataset to train encoder network via unsupervised learning.
* obs.dat contains the center location (x,y) of each obstacle in the environments.
* obs_perm2.dat contains the order in which the blocks should be placed in preset locations given by obs.dat file to setup environments.
	* For instance, in complex 3D, the permutation 8342567901 indicates obstacle #8 of size 10x10x10 should be placed at the location #0 given by obs.dat.


## Requirements
* Data Generation

	1. Install [libbot2]( https://github.com/libbot2/libbot2)
		* Make sure all dependencies of libbot2 (e.g., lcm) are installed.
		* Install libbot2 with the local installation procedure.
		* Run "make" in the data_generation folder where the README file is located.

	2. Use any compiler such as Netbeans to load the precomplie code.
		* data_generation/src/rrts_main.cpp contains the main rrt/prrt code. 	
		* data_generation/viewer/src/viewer_main.cpp contains the visualization code.
			* Also checkout comments in data_generation/viewer/src/renderers/graph_renderer.cpp

		* Note: main_viewer and rrts_main should run in parallel as:
			* rrts_main sends the path solution as well as the tree to the main_viewer to publish through local network.
			* data is transmitted through LCM network protocol.

* MPNet
	* [PyTorch](http://pytorch.org/) 


## Examples

1. Assuming paths to obstacles point-cloud are declared, train obstacle-encoder:
```python MPNET/AE/CAE.py```

2. Assuming paths to demonstration dataset and obstacle-encoder are declared, run mpnet_trainer:
	
    ```python MPNET/train.py```
    
3. Run tests by first loading the trained models:
	
    ```python MPNET/neuralplanner.py``` 

## References

```
@inproceedings{qureshi2019motion,
  title={Motion planning networks},
  author={Qureshi, Ahmed H and Simeonov, Anthony and Bency, Mayur J and Yip, Michael C},
  booktitle={2019 International Conference on Robotics and Automation (ICRA)},
  pages={2118--2124},
  year={2019},
  organization={IEEE}
}
@inproceedings{qureshi2018deeply,
  title={Deeply Informed Neural Sampling for Robot Motion Planning},
  author={Qureshi, Ahmed H and Yip, Michael C},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={6582--6588},
  year={2018},
  organization={IEEE}
}
@article{qureshi2019motion,
  title={Motion Planning Networks: Bridging the Gap Between Learning-based and Classical Motion Planners},
  author={Qureshi, Ahmed H and Miao, Yinglong and Simeonov, Anthony and Yip, Michael C},
  journal={arXiv preprint arXiv:1907.06013},
  year={2019}
}
```


