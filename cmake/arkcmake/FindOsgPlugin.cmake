# - Try to find  OsgPlugin
# Once done, this will define
#
#  OSGPLUGIN_FOUND - system has scicoslab 
#  OSGPLUGIN_LIBRARIES - libraries to link to

# macros
include(FindPackageHandleStandardArgs)

# dependencies
find_package(OpenSceneGraph REQUIRED QUIET)

# get the version
set(OSGPLUGIN_VERSION ${OPENSCENEGRAPH_VERSION})
get_filename_component(OSG_LIBRARY_ROOT "${OSG_LIBRARY}" PATH)

# find the plugins
set(OSGPLUGIN_DIR ${OSG_LIBRARY_ROOT}/osgPlugins-${OSGPLUGIN_VERSION})
set(OSGPLUGIN_LIBRARIES)
foreach(plugin ${OsgPlugin_FIND_COMPONENTS})
    string(TOUPPER ${plugin} plugin_uc) 
    string(TOLOWER ${plugin} plugin_lc) 
    find_library(OSGPLUGIN_${plugin_uc}
        NAMES osgdb_${plugin_lc} osgdb_${plugin_lc}.so
        PATHS ${OSGPLUGIN_DIR}
        )
    list(APPEND OSGPLUGIN_LIBRARIES ${OSGPLUGIN_${plugin_uc}})
endforeach()

# handle arguments
find_package_handle_standard_args(OsgPlugin
    REQUIRED_VARS OSGPLUGIN_LIBRARIES
    VERSION_VAR OSGPLUGIN_VERSION
    )
