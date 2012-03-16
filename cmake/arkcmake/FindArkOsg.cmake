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

# find the include directory
find_path(_ARKOSG_INCLUDE_DIR
	NAMES arkosg/osgUtils.hpp
    )

# find the library
find_library(_ARKOSG_LIBRARY
	NAMES arkosg
    )

# find the data directory
find_path(ARKOSG_DATADIR
	NAMES arkosg/images/ocean.rgb
    PATH_SUFFIXES share
    )

# read the version
if (EXISTS ${_ARKOSG_INCLUDE_DIR}/VERSION)
    file(READ ${_ARKOSG_INCLUDE_DIR}/VERSION ARKOSG_VERSION)
endif()

# handle arguments
set(ARKOSG_INCLUDE_DIRS ${_ARKOSG_INCLUDE_DIR})
set(ARKOSG_LIBRARIES ${_ARKOSG_LIBRARY})
find_package_handle_standard_args(ArkOsg
    REQUIRED_VARS ARKOSG_DATADIR ARKOSG_LIBRARIES ARKOSG_INCLUDE_DIRS
    VERSION_VAR ARKOSG_VERSION
    )
