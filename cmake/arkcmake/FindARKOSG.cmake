# - Try to find  ARKOSG
# Once done, this will define
#
#  ARKOSG_FOUND - system has scicoslab 
#  ARKOSG_INCLUDE_DIRS - the scicoslab include directories
#  ARKOSG_LIBRARIES - libraries to link to

include(LibFindMacros)
include(MacroCommonPaths)

MacroCommonPaths(ARKOSG)

# Include dir
find_path(ARKOSG_INCLUDE_DIR
	NAMES arkosg/osgUtils.hpp
	PATHS ${COMMON_INCLUDE_PATHS_ARKOSG}
)

# data dir
find_path(ARKOSG_DATA_DIR
	NAMES arkosg/models/plane.ac
	PATHS ${COMMON_DATA_PATHS_ARKOSG}
)

# the library itself
find_library(ARKOSG_LIBRARY
	NAMES arkosg
	PATHS ${COMMON_LIBRARY_PATHS_ARKOSG}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(ARKOSG_PROCESS_INCLUDES ARKOSG_INCLUDE_DIR)
set(ARKOSG_PROCESS_LIBS ARKOSG_LIBRARY ARKOSG_LIBRARIES)
libfind_process(ARKOSG)
