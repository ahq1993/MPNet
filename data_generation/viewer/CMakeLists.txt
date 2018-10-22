cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME viewer)
include(cmake/pods.cmake)

find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)

list(APPEND OPENGL_LIBRARIES GL )
set(GLUT_CFLAGS "")
set(GLUT_LIBRARIES -lglut)
set(ZLIB_LIBRARIES -lz)

pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(BOT2_VIS REQUIRED bot2-vis)



#tell cmake to build these subdirectories
add_subdirectory(src/renderers)
add_subdirectory(src)
