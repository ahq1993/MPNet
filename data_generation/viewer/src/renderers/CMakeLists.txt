SET(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/local/lib/pkgconfig:/opt/local/lib/pkgconfig:/usr/local/share/pkgconfig")


add_library(renderers SHARED 
            graph_renderer.cpp)

pods_use_pkg_config_packages(renderers bot2-vis lcmtypes)

pods_install_headers(graph_renderer.h DESTINATION renderers)

# make the library public
pods_install_libraries(renderers)

target_link_libraries(renderers
    ${GTK2_LDFLAGS}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${LCM_LDFLAGS}
    ${BOT2_VIS_LDFLAGS})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(renderers
    CFLAGS
    LIBS -lrenderers
    REQUIRES bot2-core bot2-vis lcmtypes
    VERSION 0.0.1)
