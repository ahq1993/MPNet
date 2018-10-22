# Macros to simplify compliance with the pods build policies.
#
# To enable the macros, add the following lines to CMakeLists.txt:
#   set(POD_NAME <pod-name>)
#   include(cmake/pods.cmake)
#
# If POD_NAME is not set, then the CMake source directory is used as POD_NAME
#
# Next, any of the following macros can be used.  See the individual macro
# definitions in this file for individual documentation.
#
# C/C++
#   pods_install_headers(...)
#   pods_install_libraries(...)
#   pods_install_executables(...)
#   pods_install_pkg_config_file(...)
#
#   pods_use_pkg_config_packages(...)
#
# Python
#   pods_install_python_packages(...)
#   pods_install_python_script(...)
#
# Java
#   None yet
#
# ----
# File: pods.cmake
# Distributed with pods version: 11.02.09

# pods_install_headers(<header1.h> ... DESTINATION <subdir_name>)
# 
# Install a (list) of header files.
#
# Header files will all be installed to include/<subdir_name>
#
# example:
#   add_library(perception detector.h sensor.h)
#   pods_install_headers(detector.h sensor.h DESTINATION perception)
#
function(pods_install_headers)
    list(GET ARGV -2 checkword)
    if(NOT checkword STREQUAL DESTINATION)
        message(FATAL_ERROR "pods_install_headers missing DESTINATION parameter")
    endif()

    list(GET ARGV -1 dest_dir)
    list(REMOVE_AT ARGV -1)
    list(REMOVE_AT ARGV -1)
    #copy the headers to the INCLUDE_OUTPUT_PATH (${CMAKE_BINARY_DIR}/include)
    foreach(header ${ARGV})
        get_filename_component(_header_name ${header} NAME)
        configure_file(${header} ${INCLUDE_OUTPUT_PATH}/${dest_dir}/${_header_name} COPYONLY)
	endforeach(header)
	#mark them to be installed
	install(FILES ${ARGV} DESTINATION include/${dest_dir})


endfunction(pods_install_headers)

# pods_install_executables(<executable1> ...)
#
# Install a (list) of executables to bin/
function(pods_install_executables)
    install(TARGETS ${ARGV} RUNTIME DESTINATION bin)
endfunction(pods_install_executables)

# pods_install_libraries(<library1> ...)
#
# Install a (list) of libraries to lib/
function(pods_install_libraries)
    install(TARGETS ${ARGV} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
endfunction(pods_install_libraries)


# pods_install_pkg_config_file(<package-name> 
#                              [VERSION <version>]
#                              [DESCRIPTION <description>]
#                              [CFLAGS <cflag> ...]
#                              [LIBS <lflag> ...]
#                              [REQUIRES <required-package-name> ...])
# 
# Create and install a pkg-config .pc file.
#
# example:
#    add_library(mylib mylib.c)
#    pods_install_pkg_config_file(mylib LIBS -lmylib REQUIRES glib-2.0)
function(pods_install_pkg_config_file)
    list(GET ARGV 0 pc_name)
    # TODO error check

    set(pc_version 0.0.1)
    set(pc_description ${pc_name})
    set(pc_requires "")
    set(pc_libs "")
    set(pc_cflags "")
    set(pc_fname "${PKG_CONFIG_OUTPUT_PATH}/${pc_name}.pc")
    
    set(modewords LIBS CFLAGS REQUIRES VERSION DESCRIPTION)
    set(curmode "")

    # parse function arguments and populate pkg-config parameters
    list(REMOVE_AT ARGV 0)
    foreach(word ${ARGV})
        list(FIND modewords ${word} mode_index)
        if(${mode_index} GREATER -1)
            set(curmode ${word})
        elseif(curmode STREQUAL LIBS)
            set(pc_libs "${pc_libs} ${word}")
        elseif(curmode STREQUAL CFLAGS)
            set(pc_cflags "${pc_cflags} ${word}")
        elseif(curmode STREQUAL REQUIRES)
            set(pc_requires "${pc_requires} ${word}")
        elseif(curmode STREQUAL VERSION)
            set(pc_version ${word})
            set(curmode "")
        elseif(curmode STREQUAL DESCRIPTION)
            set(pc_description "${word}")
            set(curmode "")
        else(${mode_index} GREATER -1)
            message("WARNING incorrect use of pods_add_pkg_config (${word})")
            break()
        endif(${mode_index} GREATER -1)
    endforeach(word)

    # write the .pc file out
    file(WRITE ${pc_fname}
        "prefix=${CMAKE_INSTALL_PREFIX}\n"
        "exec_prefix=\${prefix}\n"
        "libdir=\${exec_prefix}/lib\n"
        "includedir=\${prefix}/include\n"
        "\n"
        "Name: ${pc_name}\n"
        "Description: ${pc_description}\n"
        "Requires: ${pc_requires}\n"
        "Version: ${pc_version}\n"
        "Libs: -L\${exec_prefix}/lib ${pc_libs}\n"
        "Cflags: ${pc_cflags}\n")

    # mark the .pc file for installation to the lib/pkgconfig directory
    install(FILES ${pc_fname} DESTINATION lib/pkgconfig)
    
    # find targets that this pkg-config file depends on
    string(REPLACE " " ";" split_lib ${pc_libs})
    foreach(lib ${split_lib})
        string(REGEX REPLACE "^-l" "" libname ${lib})
        get_target_property(IS_TARGET ${libname} LOCATION)
        if (NOT IS_TARGET STREQUAL "IS_TARGET-NOTFOUND")
            set_property(GLOBAL APPEND PROPERTY "PODS_PKG_CONFIG_TARGETS-${pc_name}" ${libname})
        endif() 
    endforeach()
    
endfunction(pods_install_pkg_config_file)


# pods_install_python_script(<script_name> <python_module>)
#
# Create and install a script that invokes the python interpreter with a
# specified module.
#
# A script will be installed to bin/<script_name>.  The script simply
# adds <install-prefix>/lib/pythonX.Y/site-packages to the python path, and
# then invokes `python -m <python_module>`.
#
# example:
#    pods_install_python_script(run-pdb pdb)
function(pods_install_python_script script_name py_module)
    find_package(PythonInterp REQUIRED)

    # which python version?
    execute_process(COMMAND 
        ${PYTHON_EXECUTABLE} -c "import sys; sys.stdout.write(sys.version[:3])"
        OUTPUT_VARIABLE pyversion)

    # where do we install .py files to?
    set(python_install_dir 
        ${CMAKE_INSTALL_PREFIX}/lib/python${pyversion}/site-packages)

    # write the script file
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${script_name} "#!/bin/sh\n"
        "export PYTHONPATH=${python_install_dir}:\${PYTHONPATH}\n"
        "exec ${PYTHON_EXECUTABLE} -m ${py_module} $*\n")

    # install it...
    install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${script_name} DESTINATION bin)
endfunction()

# pods_install_python_packages(<src_dir>)
#
# Install python packages to lib/pythonX.Y/site-packages, where X.Y refers to
# the current python version (e.g., 2.6)
#
# Recursively searches <src_dir> for .py files, byte-compiles them, and
# installs them
function(pods_install_python_packages py_src_dir)
    find_package(PythonInterp REQUIRED)

    # which python version?
    execute_process(COMMAND 
        ${PYTHON_EXECUTABLE} -c "import sys; sys.stdout.write(sys.version[:3])"
        OUTPUT_VARIABLE pyversion)

    # where do we install .py files to?
    set(python_install_dir 
        ${CMAKE_INSTALL_PREFIX}/lib/python${pyversion}/site-packages)

    if(ARGC GREATER 1)
        message(FATAL_ERROR "NYI")
    else()
        # get a list of all .py files
        file(GLOB_RECURSE py_files RELATIVE ${py_src_dir} ${py_src_dir}/*.py)

        # add rules for byte-compiling .py --> .pyc
        foreach(py_file ${py_files})
            get_filename_component(py_dirname ${py_file} PATH)
            add_custom_command(OUTPUT "${py_src_dir}/${py_file}c" 
                COMMAND ${PYTHON_EXECUTABLE} -m py_compile ${py_src_dir}/${py_file} 
                DEPENDS ${py_src_dir}/${py_file})
            list(APPEND pyc_files "${py_src_dir}/${py_file}c")

            # install python file and byte-compiled file
            install(FILES ${py_src_dir}/${py_file} ${py_src_dir}/${py_file}c
                DESTINATION "${python_install_dir}/${py_dirname}")
#            message("${py_src_dir}/${py_file} -> ${python_install_dir}/${py_dirname}")
        endforeach()
        string(REGEX REPLACE "[^a-zA-Z0-9]" "_" san_src_dir "${py_src_dir}")
        add_custom_target("pyc_${san_src_dir}" ALL DEPENDS ${pyc_files})
    endif()
endfunction()


# pods_use_pkg_config_packages(<target> <package-name> ...)
#
# Convenience macro to get compiler and linker flags from pkg-config and apply them
# to the specified target.
#
# Invokes `pkg-config --cflags-only-I <package-name> ...` and adds the result to the
# include directories.
#
# Additionally, invokes `pkg-config --libs <package-name> ...` and adds the result to
# the target's link flags (via target_link_libraries)
#
# example:
#   add_executable(myprogram main.c)
#   pods_use_pkg_config_packages(myprogram glib-2.0 opencv)
macro(pods_use_pkg_config_packages target)
    if(${ARGC} LESS 2)
        message(WARNING "Useless invocation of pods_use_pkg_config_packages")
        return()
    endif()
    find_package(PkgConfig REQUIRED)
    execute_process(COMMAND 
        ${PKG_CONFIG_EXECUTABLE} --cflags-only-I ${ARGN}
        OUTPUT_VARIABLE _pods_pkg_include_flags)
    string(STRIP ${_pods_pkg_include_flags} _pods_pkg_include_flags)
    string(REPLACE "-I" "" _pods_pkg_include_flags "${_pods_pkg_include_flags}")
	separate_arguments(_pods_pkg_include_flags)
    #    message("include: ${_pods_pkg_include_flags}")
    execute_process(COMMAND 
        ${PKG_CONFIG_EXECUTABLE} --libs ${ARGN}
        OUTPUT_VARIABLE _pods_pkg_ldflags)
    string(STRIP ${_pods_pkg_ldflags} _pods_pkg_ldflags)
    #    message("ldflags: ${_pods_pkg_ldflags}")
    include_directories(${_pods_pkg_include_flags})
    target_link_libraries(${target} ${_pods_pkg_ldflags})
   
    # make the target depend on libraries being installed by this source build
    foreach(_pkg ${ARGN})
        get_property(_has_dependencies GLOBAL PROPERTY "PODS_PKG_CONFIG_TARGETS-${_pkg}" SET)
        if(_has_dependencies)
            get_property(_dependencies GLOBAL PROPERTY "PODS_PKG_CONFIG_TARGETS-${_pkg}")
            add_dependencies(${target} ${_dependencies})
            #            message("Found dependencies for ${_pkg}: ${dependencies}")
        endif()
        unset(_has_dependencies)
        unset(_dependencies)
    endforeach()

    unset(_pods_pkg_include_flags)
    unset(_pods_pkg_ldflags)
endmacro()


# pods_config_search_paths()
#
# Setup include, linker, and pkg-config paths according to the pods core
# policy.  This macro is automatically invoked, there is no need to do so
# manually.
macro(pods_config_search_paths)
    if(NOT DEFINED __pods_setup)
		#set where files should be output locally
	    set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
	    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
	    set(INCLUDE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/include)
	    set(PKG_CONFIG_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/pkgconfig)
		
		#set where files should be installed to
	    set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
	    set(EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
	    set(INCLUDE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)
	    set(PKG_CONFIG_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)


        # add build/lib/pkgconfig to the pkg-config search path
        set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_INSTALL_PATH}:$ENV{PKG_CONFIG_PATH})
        set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_OUTPUT_PATH}:$ENV{PKG_CONFIG_PATH})

        # add build/include to the compiler include path
        include_directories(BEFORE ${INCLUDE_OUTPUT_PATH})
        include_directories(${INCLUDE_INSTALL_PATH})

        # add build/lib to the link path
        link_directories(${LIBRARY_INSTALL_PATH})
        link_directories(${LIBRARY_OUTPUT_PATH})

        # abuse RPATH
        if(${CMAKE_INSTALL_RPATH})
            set(CMAKE_INSTALL_RPATH ${LIBRARY_INSTALL_PATH}:${CMAKE_INSTALL_RPATH})
        else(${CMAKE_INSTALL_RPATH})
            set(CMAKE_INSTALL_RPATH ${LIBRARY_INSTALL_PATH})
        endif(${CMAKE_INSTALL_RPATH})

        # for osx, which uses "install name" path rather than rpath
        #set(CMAKE_INSTALL_NAME_DIR ${LIBRARY_OUTPUT_PATH})
        set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_RPATH})
        
        # hack to force cmake always create install and clean targets 
        install(FILES DESTINATION)
        add_custom_target(tmp)

        set(__pods_setup true)
    endif(NOT DEFINED __pods_setup)
endmacro(pods_config_search_paths)

macro(enforce_out_of_source)
    if(CMAKE_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
      message(FATAL_ERROR 
      "\n
      Do not run cmake directly in the pod directory. 
      use the supplied Makefile instead!  You now need to
      remove CMakeCache.txt and the CMakeFiles directory.

      Then to build, simply type: 
       $ make
      ")
    endif()
endmacro(enforce_out_of_source)

#set the variable POD_NAME to the directory path, and set the cmake PROJECT_NAME
if(NOT POD_NAME)
    get_filename_component(POD_NAME ${CMAKE_SOURCE_DIR} NAME)
    message(STATUS "POD_NAME is not set... Defaulting to directory name: ${POD_NAME}") 
endif(NOT POD_NAME)
project(${POD_NAME})

#make sure we're running an out-of-source build
enforce_out_of_source()

#call the function to setup paths
pods_config_search_paths()
