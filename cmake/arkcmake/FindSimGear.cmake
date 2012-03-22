# - Try to find  SimGear
# Once done, this will define
#
#  SIMGEAR_FOUND        : library found
#  SIMGEAR_INCLUDE_DIRS : include directories
#  SIMGEAR_LIBRARIES    : libraries to link to
#  SIMGEAR_VERSION      : version
#
# when listing components, list in the order below
# to ensure proper static linking
#
# core compoennts:
#        comps
#        environment
#        nasal
#        tsync
#        bucket
#        route
#        io
#        serial
#        math
#        props
#        structure
#        timing
#        xml
#        misc
#        threads
#        debug
#        magvar
#
# scene components:
#        ephem
#        sky
#        material
#        tgdb
#        model
#        screen
#        bvh
#        util
#        sound

# macros
include(FindPackageHandleStandardArgs)

set(_SIMGEAR_EXTRA_SEARCH_PATHS
    /usr/local
    /opt/local
    )

# find the include directory
find_path(_SIMGEAR_INCLUDE_DIR
	NAMES simgear/version.h
    PATHS ${_SIMGEAR_EXTRA_SEARCH_PATHS}
    PATH_SUFFIXES include
    )

# read the version
if (EXISTS ${_SIMGEAR_INCLUDE_DIR}/simgear/version.h)
    file(READ ${_SIMGEAR_INCLUDE_DIR}/simgear/version.h SIMGEAR_VERSION_FILE)
    string(REGEX MATCH "#define SIMGEAR_VERSION[ ]+(([0-9]+\\.)+[0-9]+)"
        _SIMGEAR_VERSION_MATCH ${SIMGEAR_VERSION_FILE})
    set(SIMGEAR_VERSION ${CMAKE_MATCH_1})
else()
    set(SIMGEAR_VERSION "")
endif()

# find components
set(SIMGEAR_LIBRARIES "")
if ("${SimGear_FIND_COMPONENTS}" STREQUAL "")
    message(FATAL_ERROR "FindSimGear: must specify a simgaer library as a component.")
endif()
foreach(component ${SimGear_FIND_COMPONENTS})
    string(TOUPPER ${component} component_uc) 
    string(TOLOWER ${component} component_lc) 
    find_library(SIMGEAR_${component_uc}
        NAMES sg${component_lc}
        PATHS ${_SIMGEAR_EXTRA_SEARCH_PATHS}
        PATH_SUFFIXES lib
        )
    list(APPEND SIMGEAR_LIBRARIES ${SIMGEAR_${component_uc}})
endforeach()

# handle arguments
set(SIMGEAR_INCLUDE_DIRS ${_SIMGEAR_INCLUDE_DIR})
find_package_handle_standard_args(SimGear
    REQUIRED_VARS SIMGEAR_LIBRARIES SIMGEAR_INCLUDE_DIRS SIMGEAR_VERSION
    VERSION_VAR SIMGEAR_VERSION
    )
