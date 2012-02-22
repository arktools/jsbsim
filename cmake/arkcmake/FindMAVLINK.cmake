# - Try to find  MAVLINK
# Once done, this will define
#
#  MAVLINK_FOUND - system has scicoslab 
#  MAVLINK_INCLUDE_DIRS - the scicoslab include directories

include(LibFindMacros)
include(MacroCommonPaths)

MacroCommonPaths(MAVLINK)

# Include dir
find_path(MAVLINK_INCLUDE_DIR
    NAMES mavlink/v1.0/mavlink_types.h
    PATHS ${COMMON_INCLUDE_PATHS_MAVLINK}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(MAVLINK_PROCESS_INCLUDES MAVLINK_INCLUDE_DIR)
libfind_process(MAVLINK)

macro(build_mavlink TAG EP_BASE_DIR CMAKE_ARGS)
    ExternalProject_Add(mavlink
        GIT_REPOSITORY "git://github.com/mavlink/mavlink.git"
        GIT_TAG ${TAG}
        UPDATE_COMMAND ""
        INSTALL_DIR ${EP_BASE_DIR}/${CMAKE_INSTALL_PREFIX}
        CMAKE_ARGS ${CMAKE_ARGS}
        INSTALL_COMMAND make DESTDIR=${EP_BASE_DIR} install
    )
endmacro()
