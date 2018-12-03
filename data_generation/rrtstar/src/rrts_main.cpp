#define LIBBOT_PRESENT 0

#include <iostream>
#include <fstream>
#include <ctime>

#include <bot_core/bot_core.h>

#include <lcm/lcm.h>

#include <lcmtypes/lcmtypes.h>

#include "rrts.hpp"
#include "system_single_integrator.h"
#include<list>
#include <time.h>
#include <string>
#include <sstream>
#include <sys/stat.h>

using namespace RRTstar;
using namespace SingleIntegrator;

using namespace std;


//int sw=1;
//int algo=0;
typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;
class obst
{
public:
	double center[3];
	double size[3];
	double radius;
};
bool check (double* first, double* second)
{
    if(first[0]==second[0] && first[1]==second[1])
    return true;
    else
        return false;
}
int size=50000;
int publishTree (lcm_t *lcm, planner_t& planner, System& system);
int publishPC (lcm_t *lcm, double nodes[8000][2], int sze, System& system);
int publishTraj (lcm_t *lcm, planner_t& planner, System& system, int num, string fod);
//lcm_t *lcm, region& regionOperating, region& regionGoal,list<region*>& obstacles
//int publishEnvironment (lcm_t *lcm);

int publishEnvironment(lcm_t *lcm, region& regionOperating, region& regionGoal, list<region*>& obstacles);
//ofstream out("nodes1", ios::out | ios::binary);
// double nodes[50000][2];
string env_path="env";
mkdir(env_path.c_str(),ACCESSPERMS); // create folder with env label to store generated trajectories

int main () {
    
    double nodes[size][2]; // nodes from obstacle-free space that will become random start-goal pairs

    srand (time(0));
    
  
    /*
	//-In order to generate random environments, we randomly sample 20 obstacles locations in the workspace, as follow:
	//-Orignal workspace is 40X40 but we sample locations from 30X30 space in order to avoid obstacles going out of workspace boundry.
	////////////////////////////////////////////////////// 
	double obst[20][2];
    for (int i=0;i<20;i++)
    for (int j = 0; j < 2; j++)
	     obst[i][j] = (double)rand()/(RAND_MAX + 1.0)*30.0 
        - 15.0 + 0.0;
         
    ofstream out("obs.dat", ios::out | ios::binary);
          if(!out) {
                        cout << "Cannot open file.";
                return 1;
                }

          out.write((char *) &obst, sizeof nodes);
          out.close();
	//////////////////////////////////////////////////
    */
    // load obstacle locations 
    double fnum[20][2];
    ifstream in("obs.dat", ios::in | ios::binary);
    in.read((char *) &fnum, sizeof fnum);
    //We drop 7 obstacle blocks in the workspace to generate random environments using 20P7=77520 permutations. Note that we can have now 77520 different environments but we use 110 envs only
    int perm[77520][7];
    ifstream in2("obs_perm2.dat", ios::in | ios::binary);
    in2.read((char *) &perm, sizeof perm);
    
    //start and goal region
   
    
   
   int i=0;
   for (i=0;i<1;i++){
    	string env_no;          // string which will contain the result
    	ostringstream convert2;   // stream used for the conversion
    	convert2 << i;      // insert the textual representation of 'Number' in the characters in the stream
    	env_no =convert2.str();
    	string path="env/e"+env_no;
    	mkdir(path.c_str(),ACCESSPERMS); // create folder with env label to store generated trajectories
		/*
		We also generted a random set of nodes from obstacle-free space, denoted as graph. These nodes are used as start and goal pairs 
		*/
		double fnum2[50000][2];
		path="graph/graph"+env_no+".dat";
	    ifstream in3(path.c_str(), ios::in | ios::binary);
	    in3.read((char *) &fnum2, sizeof fnum2);
	    int t=0;
  		for (int t=0;t<100;t++){  
			cout<<"t"<<t<<endl;  
			 
			planner_t rrts;
		
			cout << "RRTstar is alive" << endl;
		
		
			// Get lcm
			lcm_t *lcm = bot_lcm_get_global (NULL);
				
		
			// Create the dynamical system
			System system;
		
			// Three dimensional configuration space
			system.setNumDimensions (2);
			// Define the operating region
			system.regionOperating.setNumDimensions(2);
			system.regionOperating.center[0] = 0.0;
			system.regionOperating.center[1] = 0.0;
			system.regionOperating.center[2] = 0.0;
			system.regionOperating.size[0] = 40.0;
			system.regionOperating.size[1] = 40.0;
			system.regionOperating.size[2] = 0.0;
			// Define the goal region		
			system.regionGoal.setNumDimensions(2);
			system.regionGoal.center[0] =fnum2[t][0];
			system.regionGoal.center[1] =fnum2[t][1];
			system.regionGoal.center[2] = 0.0;// fnum2[t][2] //if 3D
			system.regionGoal.size[0] = 1.0;
			system.regionGoal.size[1] = 1.0;
			system.regionGoal.size[2] = 0.0;
		    region *obstacle,*obstacle1,*obstacle2,*obstacle3,*obstacle4,*obstacle5,*obstacle6;
		    obstacle = new region;
		    obstacle1 = new region;  
		    obstacle2 = new region;  
		    obstacle3=new region;
		    obstacle4=new region;
		    obstacle5=new region;
		    obstacle6=new region;

 
			obstacle->setNumDimensions(2);
			obstacle->center[0] =fnum[perm[i][0]][0];
			obstacle->center[1] = fnum[perm[i][0]][1];
			obstacle->center[2] = 0.0;
			obstacle->size[0] = 5.0;
			obstacle->size[1] = 5.0;
			obstacle->size[2] = 0.0;
		 
			obstacle1->setNumDimensions(2);
			obstacle1->center[0] = fnum[perm[i][1]][0];
			obstacle1->center[1] = fnum[perm[i][1]][1];
			obstacle1->center[2] = 0.0;
			obstacle1->size[0] = 5.0;
			obstacle1->size[1] = 5.0;
			obstacle1->size[2] = 0.0;

			obstacle2->setNumDimensions(2);
			obstacle2->center[0] = fnum[perm[i][2]][0];
			obstacle2->center[1] = fnum[perm[i][2]][1];
			obstacle2->center[2] = 0.0;
			obstacle2->size[0] = 5.0;
			obstacle2->size[1] = 5.0;
			obstacle2->size[2] = 0.0;
		
			obstacle3->setNumDimensions(2);
			obstacle3->center[0] = fnum[perm[i][3]][0];
			obstacle3->center[1] = fnum[perm[i][3]][1];
			obstacle3->center[2] = 0.0;
			obstacle3->size[0] = 5.0;
			obstacle3->size[1] = 5.0;
			obstacle3->size[2] = 0.0;
		
			obstacle4->setNumDimensions(2);
			obstacle4->center[0] = fnum[perm[i][4]][0];
			obstacle4->center[1] = fnum[perm[i][4]][1];
			obstacle4->center[2] = 0.0;
			obstacle4->size[0] = 5.0;
			obstacle4->size[1] = 5.0;
			obstacle4->size[2] = 0.0;

			obstacle5->setNumDimensions(2);
			obstacle5->center[0] = fnum[perm[i][5]][0];
			obstacle5->center[1] = fnum[perm[i][5]][1];
			obstacle5->center[2] = 0.0;
			obstacle5->size[0] = 5.0;
			obstacle5->size[1] = 5.0;
			obstacle5->size[2] = 0.0;
		
		
			obstacle6->setNumDimensions(2);
			obstacle6->center[0] = fnum[perm[i][6]][0];
			obstacle6->center[1] = fnum[perm[i][6]][1];
			obstacle6->center[2] = 0.0;
			obstacle6->size[0] = 5.0;
			obstacle6->size[1] = 5.0;
			obstacle6->size[2] = 0.0;
    

                                        
			system.obstacles.push_front(obstacle);  // Add the obstacle to the list
			system.obstacles.push_front(obstacle1);  // Add the obstacle to the list
			system.obstacles.push_front(obstacle2);  // Add the obstacle to the list
			system.obstacles.push_front(obstacle3);  // Add the obstacle to the list
			system.obstacles.push_front(obstacle4);  // Add the obstacle to the list
			system.obstacles.push_front(obstacle5);
			system.obstacles.push_front(obstacle6);

			// publishEnvironment(lcm, system.regionOperating, system.regionGoal, system.obstacles);
			// Add the system to the planner
			rrts.setSystem (system);
			//publishEnvironment (lcm);
			// Set up the root vertex
			vertex_t &root = rrts.getRootVertex();  
			State &rootState = root.getState();
			
			// Define start state			
			rootState[0] =fnum2[t+1][0];
			rootState[1] =fnum2[t+1][1];
			rootState[2] = 0.0;
 
 
			// Initialize the planner
			rrts.initialize ();

			// This parameter should be larger than 1.5 for asymptotic 
			//   optimality. Larger values will weigh on optimization 
			//   rather than exploration in the RRT* algorithm. Lower 
			//   values, such as 0.1, should recover the RRT.
			rrts.setGamma (1.5);

    
    		clock_t start = clock();
    		int j=0;
			double node[2];


			// random obstacle-free nodes generation. These nodes were generated to form random start and goal pairs.
			/*
			  int s=0;
			while(j<80000)
				{
			  
						     rrts.iteration(node);
						         j++;
						      if (node[0]!=0 && node[1]!=0)
						      {
						         if(s<size){ 
						          nodes[s][0]=node[0];
						          nodes[s][1]=node[1];
						          s++;}
						      } 
						         
			}    

			 
			cout<<"s:"<<s<<endl;
			string env_no;          // string which will contain the result
			ostringstream convert;   // stream used for the conversion
			convert << i;      // insert the textual representation of 'Number' in the characters in the stream
			Result =convert.str();
			 ofstream out(("graph/graph"+env_no+".dat").c_str(), ios::out | ios::binary);
					  if(!out) {
						            cout << "Cannot open file.";
						    return 1;
						    }

					  out.write((char *) &nodes, sizeof nodes);
					  out.close();
			*/


		 
		 // p-rrt* path generation
		double cost=1000;
		int k=0;
		int c=0, cp=0;
		for (int j=0;j<=100000;j+=2000){
		
			int limit= 5000+j;
			//cout<<limit<<endl;
			while(k<limit)
			{
				
				rrts.iteration(node,-1,-1);
			    k++;
			}
			vertex_t & vertexBest=rrts.getBestVertex ();
			if(& vertexBest!=NULL)
				if (vertexBest.costFromRoot< cost){
				       cost=vertexBest.costFromRoot;
				       c++;
				    }
			if(cp!=c)
				cp=c;
			else
				break;
				  
		
		}

		cout<<"iterations:"<<k<<endl;

  		 // Run the algorithm for 10000 iteartions


		// Generate obstacle point cloud for each environment to train auto encoder.
		/*
		int s=0;
		 double obcloud[1400][2]; 
			for (list<region*>::iterator iter = system.obstacles.begin(); iter != system.obstacles.end(); iter++){
				
				region* obstacleCurr = *iter;
				
				for (int i=s; i< (200+s); i++){
				    for (int j = 0; j < system.getNumDimensions(); j++)
					    obcloud[i][j] = (double)rand()/(RAND_MAX + 1.0)*obstacleCurr->size[j] 
				- obstacleCurr->size[j]/2.0 + obstacleCurr->center[j];

				}
				if (s< 1400)
				    s=s+200;
				else
				    break;
			}       
		 string Result;          // string which will contain the result
		ostringstream convert;   // stream used for the conversion
		convert << i;      // insert the textual representation of 'Number' in the characters in the stream
		Result =convert.str();
		 ofstream out(("obs_cloud/obc"+Result+".dat").c_str(), ios::out | ios::binary);
				  if(!out) {
				                cout << "Cannot open file.";
				        return 1;
				        }

				  out.write((char *) &obcloud, sizeof obcloud);
				  out.close();  
		 */

    	clock_t finish = clock();
    	cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;


    	//publishTree (lcm, rrts, system);
    	// stores path in the folder env_no
    	publishTraj (lcm, rrts, system,t, env_no );
    
   

   } } 
      
    return 1;
}


int publishEnvironment (lcm_t *lcm, region& regionOperating, region& regionGoal, list<region*>& obstacles) {
    
    // Publish the environment
    lcmtypes_environment_t *environment = (lcmtypes_environment_t*) malloc (sizeof(lcmtypes_environment_t));
    
    environment->operating.center[0] = regionOperating.center[0];
    environment->operating.center[1] = regionOperating.center[1];
    environment->operating.center[2] = regionOperating.center[2];
    environment->operating.size[0] = regionOperating.size[0];
    environment->operating.size[1] = regionOperating.size[1];
    environment->operating.size[2] = regionOperating.size[2];

    environment->goal.center[0] = regionGoal.center[0];
    environment->goal.center[1] = regionGoal.center[1];
    environment->goal.center[2] = regionGoal.center[2];
    environment->goal.size[0] = regionGoal.size[0];
    environment->goal.size[1] = regionGoal.size[1];
    environment->goal.size[2] = regionGoal.size[2];
    
    
    environment->num_obstacles = obstacles.size();
    
    if (environment->num_obstacles > 0) 
        environment->obstacles = (lcmtypes_region_3d_t *) malloc (sizeof(lcmtypes_region_3d_t));
    
    int idx_obstacles = 0;
    for (list<region*>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++){
        
        region* obstacleCurr = *iter;
        
        environment->obstacles[idx_obstacles].center[0] = obstacleCurr->center[0];
        environment->obstacles[idx_obstacles].center[1] = obstacleCurr->center[1];
        environment->obstacles[idx_obstacles].center[2] = obstacleCurr->center[2];
        environment->obstacles[idx_obstacles].size[0] = obstacleCurr->size[0];
        environment->obstacles[idx_obstacles].size[1] = obstacleCurr->size[1];
        environment->obstacles[idx_obstacles].size[2] = obstacleCurr->size[2];
        
        idx_obstacles++;
    }
    
    
    lcmtypes_environment_t_publish (lcm, "ENVIRONMENT", environment);
    lcmtypes_environment_t_destroy (environment);
    
    return 1;
}

int publishTraj (lcm_t *lcm, planner_t& planner, System& system, int num, string fod) {
    
    
    cout << "Publishing trajectory -- start" << endl;

    
    vertex_t& vertexBest = planner.getBestVertex ();
    
    if (&vertexBest == NULL) {
        cout << "No best vertex" << endl;
        double path[1][2];
        path[0][0]=0;
        path[0][1]=0;
        string Result;          // string which will contain the result
    	ostringstream convert;   // stream used for the conversion
    	convert << num;      // insert the textual representation of 'Number' in the characters in the stream
    	Result =convert.str(); // set 'Result' to the contents of the stream



  		ofstream out(("e"+fod+"/path"+Result+".dat").c_str(), ios::out | ios::binary);
  		if(!out) {
      		cout << "Cannot open file.";
                return 1;
        }

  		out.write((char *) &path, sizeof path);
  		out.close();
        return 0;
    }

    cout<<"Cost From root "<<vertexBest.costFromRoot;
    list<double*> stateList;
    planner.getBestTrajectory (stateList);
    lcmtypes_trajectory_t *opttraj = (lcmtypes_trajectory_t *) malloc (sizeof (lcmtypes_trajectory_t));
    opttraj->num_states = stateList.size();
    opttraj->states = (lcmtypes_state_t *) malloc (opttraj->num_states * sizeof (lcmtypes_state_t));
    int psize=(stateList.size()-1)/2+1;
    int pindex=0;
    double path[psize][2];
    int stateIndex = 0;

       for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) {
        
        double* stateRef = *iter;
        opttraj->states[stateIndex].x = stateRef[0];
        opttraj->states[stateIndex].y = stateRef[1];
        if(pindex>0){

            if(path[pindex-1][0]!=stateRef[0]){
            path[pindex][0]=stateRef[0];
            path[pindex][1]=stateRef[1];
            pindex++;
            }
        }
        else{
        path[pindex][0]=stateRef[0];
        path[pindex][1]=stateRef[1];
        pindex++;
        }

        if (system.getNumDimensions() > 2)
            opttraj->states[stateIndex].z = stateRef[2];
        else
            opttraj->states[stateIndex].z = 0.0;
        

        delete [] stateRef;
        
        stateIndex++;
    }
  	string Result;          // string which will contain the result
    ostringstream convert;   // stream used for the conversion
    convert << num;      // insert the textual representation of 'Number' in the characters in the stream
    Result =convert.str(); // set 'Result' to the contents of the stream

  	ofstream out(("env/e"+fod+"/path"+Result+".dat").c_str(), ios::out | ios::binary);
 	if(!out) {
      cout << "Cannot open file.";
                return 1;
               }

    out.write((char *) &path, sizeof path);
  	out.close();
    
    lcmtypes_trajectory_t_publish (lcm, "TRAJECTORY", opttraj);
    
    lcmtypes_trajectory_t_destroy (opttraj);
    
    cout << "Publishing trajectory -- end" << endl;
    
    
    
    return 1;
}


int publishTree (lcm_t *lcm, planner_t& planner, System& system) {
    
    
    cout << "Publishing the tree -- start" << endl;
    
    bool plot3d = (system.getNumDimensions() > 2);
    
    lcmtypes_graph_t *graph = (lcmtypes_graph_t *) malloc (sizeof (lcmtypes_graph_t));
    graph->num_vertices = planner.numVertices; 
    cout<<"num_Vertices: "<< graph->num_vertices<< endl;
    
    if (graph->num_vertices > 0) {    
        
        graph->vertices = (lcmtypes_vertex_t *) malloc (graph->num_vertices * sizeof(lcmtypes_vertex_t));
        
        int vertexIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {
            
            
            vertex_t &vertexCurr = **iter;
            State &stateCurr = vertexCurr.getState ();
            
            graph->vertices[vertexIndex].state.x = stateCurr[0];
            graph->vertices[vertexIndex].state.y = stateCurr[1];
            if (plot3d){ 
                graph->vertices[vertexIndex].state.z = stateCurr[2];
            }
            else 
                graph->vertices[vertexIndex].state.z = 0.0;
            
            vertexIndex++;
            
        }
    }
    else {
        graph->vertices = NULL;
    }
    
    if (graph->num_vertices > 1) {
        
        graph->num_edges = graph->num_vertices - 1;
        graph->edges = (lcmtypes_edge_t *) malloc (graph->num_edges * sizeof(lcmtypes_edge_t));
        
        
        int edgeIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {
            
            vertex_t &vertexCurr = **iter;
            
            vertex_t &vertexParent = vertexCurr.getParent();
            
            if ( &vertexParent == NULL ) 
                continue;
            
            State &stateCurr = vertexCurr.getState ();
            State &stateParent = vertexParent.getState();
            
            
            graph->edges[edgeIndex].vertex_src.state.x = stateParent[0];
            graph->edges[edgeIndex].vertex_src.state.y = stateParent[1];
            if (plot3d)
                graph->edges[edgeIndex].vertex_src.state.z = stateParent[2];
            else 
                graph->edges[edgeIndex].vertex_src.state.z = 0.0;
            
            
            graph->edges[edgeIndex].vertex_dst.state.x = stateCurr[0];
            graph->edges[edgeIndex].vertex_dst.state.y = stateCurr[1];
            if (plot3d)
                graph->edges[edgeIndex].vertex_dst.state.z = stateCurr[2];
            else 
                graph->edges[edgeIndex].vertex_dst.state.z = 0.0;
            
            graph->edges[edgeIndex].trajectory.num_states = 0;
            graph->edges[edgeIndex].trajectory.states = NULL;
            
            edgeIndex++;
        }
        
    }
    else {
        graph->num_edges = 0;
        graph->edges = NULL;
    }
    
    lcmtypes_graph_t_publish (lcm, "GRAPH", graph);
    
    lcmtypes_graph_t_destroy (graph);
    
    cout << "Publishing the tree -- end" << endl;
    
    return 1;
}



