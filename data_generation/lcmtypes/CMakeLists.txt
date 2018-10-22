SET(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/local/lib/pkgconfig:/opt/local/lib/pkgconfig:/usr/local/share/pkgconfig")

cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME lcmtypes)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

include_directories(${LCMTYPES_INCLUDE_DIRS})

pods_install_pkg_config_file(lcmtypes
    CFLAGS
    LIBS -llcmtypes_lcmtypes
    VERSION 0.0.1)
