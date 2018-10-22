SET(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/local/lib/pkgconfig:/opt/local/lib/pkgconfig:/usr/local/share/pkgconfig")

pods_install_pkg_config_file(rrtstar
    CFLAGS
    LIBS  -lrrtstar
    REQUIRES lcmtypes
    VERSION 0.0.1)

include_directories(
    ${LCM_INCLUDE_DIRS})

add_executable(rrtstar rrts_main.cpp system_single_integrator.cpp kdtree.c)

pods_use_pkg_config_packages(rrtstar
    bot2-core
    lcmtypes)

pods_install_executables(rrtstar)