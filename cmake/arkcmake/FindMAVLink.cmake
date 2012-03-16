# - Try to find  MAVLink
# Once done, this will define
#
#  MAVLINK_FOUND        : library found
#  MAVLINK_INCLUDE_DIRS : include directories
#  MAVLINK_VERSION      : version

# macros
include(FindPackageHandleStandardArgs)

# find the include directory
find_path(_MAVLINK_INCLUDE_DIR
	NAMES mavlink/v1.0/mavlink_types.h
    )

# read the version
if (EXISTS ${_MAVLINK_INCLUDE_DIR}/VERSION)
    file(READ ${_MAVLINK_INCLUDE_DIR}/VERSION MAVLINK_VERSION)
endif()

# handle arguments
set(MAVLINK_INCLUDE_DIRS ${_MAVLINK_INCLUDE_DIR}) 
find_package_handle_standard_args(MAVLink
    REQUIRED_VARS MAVLINK_INCLUDE_DIRS
    VERSION_VAR MAVLINK_VERSION
    )
