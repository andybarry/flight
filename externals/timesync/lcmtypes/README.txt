Place LCM type definitions in this directory.

Each type should be named as:

    typename.lcm

To automatically build message types from CMake, add the following lines to
the root CMakeLists.txt file:

    include(cmake/lcmtypes.cmake)
    lcmtypes_build()

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
