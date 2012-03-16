# - Try to find  ArkOsg
# Once done, this will define
#
#  ARKOSG_FOUND        : library found
#  ARKOSG_INCLUDE_DIRS : include directories
#  ARKOSG_LIBRARIES    : libraries to link to
#  ARKOSG_DATADIR      : data directory 
#  ARKOSG_VERSION      : version

# macros
include(FindPackageHandleStandardArgs)

set(_ARKOSG_EXTRA_SEARCH_PATHS
    /usr/local
    /opt/local
    )

# find the include directory
find_path(_ARKOSG_INCLUDE_DIR
	NAMES arkosg/osgUtils.hpp
    PATHS ${_ARKOSG_EXTRA_SEARCH_PATHS}
    PATH_SUFFIXES include
    )

# find the library
find_library(_ARKOSG_LIBRARY
	NAMES arkosg
    PATHS ${_ARKOSG_EXTRA_SEARCH_PATHS}
    PATH_SUFFIXES lib
    )

# find the data directory
find_path(ARKOSG_DATADIR
	NAMES arkosg/images/ocean.rgb
    PATHS ${_ARKOSG_EXTRA_SEARCH_PATHS}
    PATH_SUFFIXES share
    )

# read the version
if (EXISTS ${_ARKOSG_INCLUDE_DIR}/arkosg/config.h)
    file(READ ${_ARKOSG_INCLUDE_DIR}/arkosg/config.h ARKOSG_CONFIG_FILE)
    string(REGEX MATCH "#define ARKOSG_VERSION[ ]+\"([0-9]\\.[0-9]\\.[0-9])\""
        _ARKOSG_VERSION_MATCH ${ARKOSG_CONFIG_FILE})
    set(ARKOSG_VERSION ${CMAKE_MATCH_1})
endif()

# handle arguments
set(ARKOSG_INCLUDE_DIRS ${_ARKOSG_INCLUDE_DIR})
set(ARKOSG_LIBRARIES ${_ARKOSG_LIBRARY})
find_package_handle_standard_args(ArkOsg
    REQUIRED_VARS ARKOSG_LIBRARIES ARKOSG_INCLUDE_DIRS ARKOSG_DATADIR 
    VERSION_VAR ARKOSG_VERSION
    )
