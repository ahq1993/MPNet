#ifndef RACECAR_ROAD_RENDERER_H_
#define RACECAR_ROAD_RENDERER_H_

#include <iostream>

#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>

#include <gtk/gtk.h>
#include <lcm/lcm.h>

#include <GL/glut.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
    
    void add_graph_renderer_to_viewer (BotViewer* viewer, int render_priority, lcm_t* lcm);
    
#ifdef __cplusplus
}
#endif

#endif
