# - Try to find  ArkComm
# Once done, this will define
#
#  ARKCOMM_FOUND        : library found
#  ARKCOMM_INCLUDE_DIRS : include directories
#  ARKCOMM_LIBRARIES    : libraries to link to
#  ARKCOMM_VERSION      : version

# macros
include(FindPackageHandleStandardArgs)

# find the include directory
find_path(_ARKCOMM_INCLUDE_DIR
	NAMES arkcomm/AsyncSerial.hpp
    )

# find the library
find_library(_ARKCOMM_LIBRARY
	NAMES arkcomm
    )

# read the version
if (EXISTS ${_ARKCOMM_INCLUDE_DIR}/VERSION)
    file(READ ${_ARKCOMM_INCLUDE_DIR}/VERSION ARKCOMM_VERSION)
endif()

# handle arguments
set(ARKCOMM_INCLUDE_DIRS ${_ARKCOMM_INCLUDE_DIR})
set(ARKCOMM_LIBRARIES ${_ARKCOMM_LIBRARY})
find_package_handle_standard_args(ArkComm
    REQUIRED_VARS ARKCOMM_LIBRARIES ARKCOMM_INCLUDE_DIRS
    VERSION_VAR ARKCOMM_VERSION
    )
