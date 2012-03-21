# - Try to find  ArkComm
# Once done, this will define
#
#  ARKCOMM_FOUND        : library found
#  ARKCOMM_INCLUDE_DIRS : include directories
#  ARKCOMM_LIBRARIES    : libraries to link to
#  ARKCOMM_VERSION      : version

# macros
include(FindPackageHandleStandardArgs)

set(_ARKCOMM_EXTRA_SEARCH_PATHS
    /usr/local
    /opt/local
    )

# find the include directory
find_path(_ARKCOMM_INCLUDE_DIR
	NAMES arkcomm/AsyncSerial.hpp
    PATHS ${_ARKCOMM_EXTRA_SEARCH_PATHS}
    PATH_SUFFIXES include
    )

# find the library
find_library(_ARKCOMM_LIBRARY
	NAMES arkcomm
    PATHS ${_ARKCOMM_EXTRA_SEARCH_PATHS}
    PATH_SUFFIXES lib
    )

# read the version
if (EXISTS ${_ARKCOMM_INCLUDE_DIR}/arkcomm/config.h)
    file(READ ${_ARKCOMM_INCLUDE_DIR}/arkcomm/config.h ARKCOMM_CONFIG_FILE)
    string(REGEX MATCH "#define ARKCOMM_VERSION[ ]+\"(([0-9]+\\.)+[0-9]+)\""
        ARKCOMM_VERSION_MATCH ${ARKCOMM_CONFIG_FILE})
    set(ARKCOMM_VERSION ${CMAKE_MATCH_1})
endif()

# handle arguments
set(ARKCOMM_INCLUDE_DIRS ${_ARKCOMM_INCLUDE_DIR})
set(ARKCOMM_LIBRARIES ${_ARKCOMM_LIBRARY})
find_package_handle_standard_args(ArkComm
    REQUIRED_VARS ARKCOMM_LIBRARIES ARKCOMM_INCLUDE_DIRS
    VERSION_VAR ARKCOMM_VERSION
    )
