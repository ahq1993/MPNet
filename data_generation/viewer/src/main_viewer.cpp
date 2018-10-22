#include <iostream>
#include <ctime>

//#include <gtk/gtk.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <renderers/graph_renderer.h>

using namespace std;


typedef struct {
    BotViewer *viewer;
    lcm_t *lcm;
} viewer_app_t;



int main(int argc, char *argv[])
{
    
    gtk_init(&argc, &argv);
    glutInit(&argc, argv);
    g_thread_init(NULL);
    
    setlinebuf(stdout);
    
    viewer_app_t app;
    memset(&app, 0, sizeof(app));
    
    
    BotViewer *viewer = bot_viewer_new("Viewer");
    app.viewer = viewer;
    app.lcm = lcm_create(NULL);
    bot_glib_mainloop_attach_lcm(app.lcm);
    
    // setup renderers
    //bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 0); 
    add_graph_renderer_to_viewer (viewer, 1, app.lcm);

    
    // run the main loop
    gtk_main();

    // cleanup
    bot_viewer_unref(viewer);    
    
    cout << "RRTstar is alive" << endl;
    
    return 1;
}

