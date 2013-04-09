cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME timesync)
include(cmake/pods.cmake)

include(cmake/lcmtypes.cmake)
lcmtypes_build()

# create an executable, and make it public
add_executable(timesync src/timesync.c)

pods_use_pkg_config_packages(timesync bot2-core lcmtypes_timesync)

pods_install_executables(timesync)


# timesync-as-root
set(script_name timesync-as-root)
file(WRITE ${CMAKE_BINARY_DIR}/bin/${script_name} 
    "#!/bin/sh\n"
    "exec ssh -t root@localhost ${CMAKE_INSTALL_PREFIX}/bin/timesync $@\n")

# install it...
install(PROGRAMS ${CMAKE_BINARY_DIR}/bin/${script_name} DESTINATION bin)