#include "graph_renderer.h"

#include <lcmtypes/lcmtypes.h>

#include <vector>
#include <cmath>
#include <sstream>
#include <fstream>
#define RENDERER_NAME "Graph Visualizer"

using namespace std;


class RendererGraph {
public:
    BotRenderer renderer;
    BotGtkParamWidget *pw;
    BotViewer *viewer;
    lcm_t * lcm;
    
    lcmtypes_graph_t *graph_last;
    lcmtypes_environment_t *environment_last;
   lcmtypes_trajectory_t *trajectory_last;
    
    double obstacle_opacity;
};


        
static void graph_message_handler (const lcm_recv_buf_t *rbuf, const char *channel, const lcmtypes_graph_t *msg, void *user) {
    
    RendererGraph *self = (RendererGraph *) user;
    
    if (self->graph_last) 
        lcmtypes_graph_t_destroy (self->graph_last);
    
    self->graph_last = lcmtypes_graph_t_copy (msg);
    
    bot_viewer_request_redraw (self->viewer);
}


static void environment_message_handler (const lcm_recv_buf_t *rbuf, const char *channel, const lcmtypes_environment_t *msg, void *user) {
    cout<<"here"<<endl;
    RendererGraph *self = (RendererGraph *) user;
    
    if (self->environment_last) 
        lcmtypes_environment_t_destroy (self->environment_last);
    
    self->environment_last = lcmtypes_environment_t_copy (msg);
    
    bot_viewer_request_redraw (self->viewer);
}

static void trajectory_message_handler (const lcm_recv_buf_t *rbuf, const char *channel, const lcmtypes_trajectory_t *msg, void *user) {

    RendererGraph *self = (RendererGraph *) user;

    if (self->trajectory_last)
        lcmtypes_trajectory_t_destroy (self->trajectory_last);

    self->trajectory_last = lcmtypes_trajectory_t_copy (msg);

    bot_viewer_request_redraw (self->viewer);
}


static void renderer_graph_draw(BotViewer *viewer, BotRenderer *renderer)
{   
    
    RendererGraph *self = (RendererGraph*) renderer;

    
    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glEnable (GL_RESCALE_NORMAL);

    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel (GL_SMOOTH);
    glEnable (GL_LIGHTING);

    float color_goal[] = { 0.0, 0.0, 1.0,1.0};
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_goal);
	glPushMatrix ();
	
	// root/start state
	//glTranslated (self->environment_last->goal.center[0], self->environment_last->goal.center[1], self->environment_last->goal.center[2]);
    glTranslated (0.0, -60.0, 0.0);
    glRotatef (0.0, 0.0, 0.0, 1.0);

    //glScalef (self->environment_last->goal.size[0], self->environment_last->goal.size[1], self->environment_last->goal.size[2]);

    bot_gl_draw_disk (0.0);

    glPopMatrix ();
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_goal);


    glPushMatrix ();

	// goal-state
    //glTranslated (self->environment_last->goal.center[0], self->environment_last->goal.center[1], self->environment_last->goal.center[2]);
    glTranslated (13.2117605209,-7.65872478485, 0.0);
    glRotatef (0.0, 0.0, 0.0, 1.0);


    bot_gl_draw_disk (0.0);

    glPopMatrix ();
            
    if (self->trajectory_last)  // if there exist a path solution
    {

		//Draw generated trajectory

        for (int i = 0; i < self->trajectory_last->num_states-1; i++) {

           
		    glLineWidth (4.0);
		    float color_edge[] ={1.0, 0.0, 0.0, 1.0};
		    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_edge);



		    glBegin (GL_LINE_STRIP);

		    glVertex3f (self->trajectory_last->states[i].x,
		                self->trajectory_last->states[i].y,
		                self->trajectory_last->states[i].z);
		    glVertex3f (self->trajectory_last->states[i+1].x,
		                self->trajectory_last->states[i+1].y,
		                self->trajectory_last->states[i+1].z);
		    glEnd();
         }
              // Draw the graph
         if (self->graph_last) {

             // Draw the vertices

             glPointSize(1.0);
             float color_vertex[] = {0.1, 0.1, 0.8, 1.0};
             glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_vertex);
             glEnable (GL_POINT_SMOOTH);
             glBegin (GL_POINTS);

             for (int i = 0; i < self->graph_last->num_vertices; i++) {

                 glVertex3f (self->graph_last->vertices[i].state.x,
                             self->graph_last->vertices[i].state.y,
                             self->graph_last->vertices[i].state.z);
             }

             glEnd();


             // Draw the edges
             for (int i = 0; i < self->graph_last->num_edges; i++) {

                 glLineWidth (1.0); 
                 float color_edge[] = {0.8, 0.3, 0.3, 0.8};
                 glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_edge);

                 glBegin (GL_LINE_STRIP);

                 glVertex3f (self->graph_last->edges[i].vertex_src.state.x,
                             self->graph_last->edges[i].vertex_src.state.y,
                             self->graph_last->edges[i].vertex_src.state.z);
                 glVertex3f (self->graph_last->edges[i].vertex_dst.state.x,
                             self->graph_last->edges[i].vertex_dst.state.y,
                             self->graph_last->edges[i].vertex_dst.state.z);
                 glEnd();
             }

           }
    }

    //Environments


    //load obstacle_location file here                        
	double fnum[20][2];
    ifstream in("obs.dat", ios::in | ios::binary);
    in.read((char *) &fnum, sizeof fnum);

	
	//load obstacle permutations here (see rrts_main for more detail on environment generation)
    int perm[77520][7];
    ifstream in2("obs_perm2.dat", ios::in | ios::binary);
    in2.read((char *) &perm, sizeof perm);                            


    int i=0; //env_no

	float color_obstacles[] = { 0.37, 0.3, 0.3,((double)(self->obstacle_opacity))/100.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
 

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][0]][0],fnum[perm[i][0]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][1]][0],fnum[perm[i][1]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][2]][0],fnum[perm[i][2]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][3]][0],fnum[perm[i][3]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();


    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][4]][0],fnum[perm[i][4]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][5]][0],fnum[perm[i][5]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();
                            
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][6]][0],fnum[perm[i][6]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();

    return;
} 


/*Comment out above renderer_graph_draw function in order to use followinh

Following function can be adapted to display: 
-obstacles point-cloud 
-obstacle-free space (random start-goal pairs) 
-MPNet generated paths
-DeepSMP generated samples  
TO DO:
-Load MPNet generated data from files to publish
-Load oracle (RRT*,P-RRT*) generated paths to publish
*/

/*
static void renderer_graph_draw(BotViewer *viewer, BotRenderer *renderer)
{   
    
    RendererGraph *self = (RendererGraph*) renderer;

    
    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glEnable (GL_RESCALE_NORMAL);

    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel (GL_SMOOTH);
    glEnable (GL_LIGHTING);

    int s=1;
    int size=50000;
    double fnum[20][2];
    ifstream in("obs.dat", ios::in | ios::binary);
    in.read((char *) &fnum, sizeof fnum);

    int perm[77520][7];
    ifstream in2("obs_perm2.dat", ios::in | ios::binary);
    in2.read((char *) &perm, sizeof perm);
    
    double obs[size][2];
    ifstream in3("graph/graph50.dat", ios::in | ios::binary);
    in3.read((char *) &obs, sizeof obs);
    
	//visualize obstacles point cloud that will be passed on to obstacle space encoder of MPNet
    //double obs_cloud[1400][2];
    //ifstream in4("obs_cloud/obc10000.dat", ios::in | ios::binary);
    //in4.read((char *) &obs_cloud, sizeof obs_cloud);
    //cout<<"point cloud"<<endl;
    //glPointSize(0.0);
    //float color_vertex[] = {0.1, 0.1, 0.8, 1.0};
    //glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_vertex);
    //glEnable (GL_POINT_SMOOTH);
    //glBegin (GL_POINTS);

   	//for (int i = 0; i <1400; i++) {

      	//glVertex3f (obs_cloud[i][0],obs_cloud[i][1],0.0);
   	//}

   	//glEnd();
   
	//load MPNet generated paths here and assign it's x and y coordinates to arrays pathx and path. respectively.
		
	double pathx[]=
	-12.3745660782,
	-7.70831871033,
	4.93411445618,
	11.2,
	13.2117605209};

	double pathy[]=
	8.56354236603,
	9.5905714035,
	2.13003230095,
	0.385264575481,
	-7.65872478485};
	
	// load oracle paths (rrtstar/p-rrtstar/BIT*) here and assign their x and y coordinates to px and py respectively

	double px[]={
	-12.3745660782,
	-7.70831871033,
	4.93411445618,
	11.2,
	13.2117605209};
	double py[]={
	8.56354236603,
	9.5905714035,
	2.13003230095,
	0.385264575481,
	-7.65872478485};


      
    // publist MPNet paths     
    for (int i = 0; i <sizeof(pathx)/sizeof(pathx[0])-1; i++) {

           
       	glLineWidth (2.5);
        float color_edge[] ={1.0, 0.0, 0.0, 1.0};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_edge);
		glPointSize(0.5);
		glBegin (GL_LINE_STRIP);
			glVertex3f (pathx[i],pathy[i],0.0);
            glVertex3f (pathx[i+1],pathy[i+1],0.0);
        glEnd();
    }
    //publish oracle paths for comparison     
    for (int i = 0; i <sizeof(px)/sizeof(px[0])-1; i++) {

           
        glLineWidth (2.5);
        float color_edge[] ={0.1, 0.1, 0.8, 1.0};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_edge);
        glBegin (GL_LINE_STRIP);
        	glVertex3f (px[i],py[i],0.0);
        	glVertex3f (px[i+1],py[i+1],0.0);
        glEnd();
    }
          
	int i=8; //env_no

	float color_obstacles[] = { 0.37, 0.3, 0.3,((double)(self->obstacle_opacity))/100.0};
 

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][0]][0],fnum[perm[i][0]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
   	glPushMatrix ();
   	glTranslated (fnum[perm[i][1]][0],fnum[perm[i][1]][1], 0);
   	glRotatef (0.0, 0.0, 0.0, 1.0);
   	glScalef (5.0,5.0,0.0);
   	bot_gl_draw_cube ();
   	glPopMatrix ();

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][2]][0],fnum[perm[i][2]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][3]][0],fnum[perm[i][3]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();


    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][4]][0],fnum[perm[i][4]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();

    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][5]][0],fnum[perm[i][5]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0,5.0,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();
                            
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
    glPushMatrix ();
    glTranslated (fnum[perm[i][6]][0],fnum[perm[i][6]][1], 0);
    glRotatef (0.0, 0.0, 0.0, 1.0);
    glScalef (5.0/s,5.0/s,0.0);
    bot_gl_draw_cube ();
    glPopMatrix ();
                   

   return;
}*/


static void renderer_graph_free (BotRenderer *renderer)
{
    RendererGraph *self = (RendererGraph*) renderer;
    
    if (self->graph_last)
        lcmtypes_graph_t_destroy (self->graph_last);
    free(self);
}


void add_graph_renderer_to_viewer (BotViewer* viewer, int render_priority, lcm_t* lcm)
{
    
    RendererGraph *self = new RendererGraph;
    BotRenderer *renderer = &self->renderer;
    self->lcm = lcm;
    self->viewer = viewer;
    
    renderer->draw = renderer_graph_draw;
    renderer->destroy = renderer_graph_free;
    renderer->widget = gtk_vbox_new(FALSE, 0);
    renderer->name = (char *) RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;
    
    
    self->graph_last = NULL;
    self->environment_last = NULL;
    self->trajectory_last = NULL;
    self->obstacle_opacity = 80;
    
    // subscribe to messages
    lcmtypes_graph_t_subscribe (lcm, "GRAPH", graph_message_handler, self);
    lcmtypes_environment_t_subscribe (lcm, "ENVIRONMENT", environment_message_handler, self);
    lcmtypes_trajectory_t_subscribe (lcm, "TRAJECTORY", trajectory_message_handler, self);
    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
}
