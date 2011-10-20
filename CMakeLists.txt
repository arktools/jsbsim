project (arkscicos)

# Requried cmake version
cmake_minimum_required (VERSION 2.8.2)

# global needed variables
set(APPLICATION_NAME ${PROJECT_NAME})

# set these for release
set(APPLICATION_VERSION_MAJOR "1")
set(APPLICATION_VERSION_MINOR "0")
set(APPLICATION_VERSION_PATCH "0")
set(MAVLINK_VERSION "master")

set(APPLICATION_VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}.${APPLICATION_VERSION_PATCH}")
set(LIBRARY_VERSION ${APPLICATION_VERSION})

# SOVERSION scheme: CURRENT.AGE.REVISION
#   If there was an incompatible interface change:
#     Increment CURRENT. Set AGE and REVISION to 0
#   If there was a compatible interface change:
#     Increment AGE. Set REVISION to 0
#   If the source code was changed, but there were no interface changes:
#     Increment REVISION.
set(LIBRARY_SOVERSION "0.0.0")

# where to look first for cmake modules, before ${CMAKE_ROOT}/Modules/ is checked
set(CMAKE_MODULE_PATH
    ${CMAKE_SOURCE_DIR}/cmake
    ${CMAKE_SOURCE_DIR}/cmake/arkcmake
)

# add definitions
include(DefineCMakeDefaults)
include(DefinePlatformDefaults)
include(DefineCompilerFlags)
include(DefineInstallationPaths)

# cpack
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "arkTools scicoslab toolbox")
set(CPACK_PACKAGE_VENDOR "arkTools")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "james.goppert@gmail.com")
set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.txt")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/COPYING.txt")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.txt")
set(CPACK_PACKAGE_VERSION_MAJOR "${APPLICATION_VERSION_MAJOR}") 
set(CPACK_PACKAGE_VERSION_MINOR "${APPLICATION_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${APPLICATION_VERSION_PATCH}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
set(CPACK_SET_DESTDIR TRUE)
set(CPACK_SOURCE_IGNORE_FILES ${CPACK_SOURCE_IGNORE_FILES}
	/.git/;/build/;~$;.*\\\\.bin$;.*\\\\.swp$)
set(CPACK_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
# find libraries with cmake modules

find_package(SCICOSLAB REQUIRED)
find_package(OpenSceneGraph 2.8.0 COMPONENTS osgGA osgDB osgUtil osgViewer)
find_package(JSBSIM)
find_package(MAVLINK)
find_package(Boost 1.42 REQUIRED COMPONENTS thread-mt system-mt)

if(WIN32)
    set(CPACK_GENERATOR "NSIS")
    set(CPACK_SOURCE_GENERATOR "ZIP")
elseif(APPLE)
    set(CPACK_GENERATOR "PackageMaker")
    set(CPACK_SOURCE_GENERATOR "TGZ")
elseif(UNIX)
    set(CPACK_GENERATOR "DEB")
    set(CPACK_SOURCE_GENERATOR "TGZ")
else()
    message(FATAL_ERROR "unknown operating system")
endif()
#if(WIN32 and not UNIX)
  ## There is a bug in NSI that does not handle full unix paths properly. Make
  ## sure there is at least one set of four (4) backlasshes.
  #set(CPACK_PACKAGE_ICON "${CMake_SOURCE_DIR}/Utilities/Release\\\\InstallIcon.bmp")
  #set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\MyExecutable.exe")
  #set(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY} ${PROJECT_NAME}")
  #set(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.my-project-home-page.org")
  #set(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.my-personal-home-page.com")
  #set(CPACK_NSIS_CONTACT "me@my-personal-home-page.com")
  #set(CPACK_NSIS_MODIFY_PATH ON)
#else(WIN32 and not UNIX)
  #set(CPACK_STRIP_FILES "bin/MyExecutable")
  #set(CPACK_SOURCE_STRIP_FILES "")
#endif(WIN32 and not UNIX)
#set(CPACK_PACKAGE_EXECUTABLES "MyExecutable" "My Executable")
include(CPack)

# disallow in-source build
include(MacroEnsureOutOfSourceBuild)
macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build. 
Please create a separate build directory and run 'cmake /path/to/${PROJECT_NAME} [options]' there.")

# add macros
include(MacroAddPlugin)
include(MacroCopyFile)
include(MacroSetDefault)
include(language_support_v2)
include(MacroAddLinkFlags)
include(ExternalProject)
include(CTest)

# settings
option(DEV_MODE "Setup paths for developer testing." OFF)
option(BUILD_SHARED_LIBS "Build shared libraries instead of static." OFF)

# built variables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# enable languages
enable_language(C)
enable_language(CXX)
if(NOT APPLE)
	enable_language(Fortran)
endif()

# add make dist target
add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

# find libraries with cmake modules
find_package(SCICOSLAB REQUIRED)
find_package(OpenSceneGraph 2.8.0 COMPONENTS osgGA osgDB osgUtil osgViewer)
find_package(SIMGEAR)
find_package(PLIB)
find_package(LAPACK)
find_package(ARKOSG)
find_package(ARKCOMM)
find_package(ARKMATH)
find_package(JSBSIM)
find_package(MAVLINK)
find_package(Boost 1.42 REQUIRED COMPONENTS thread-mt system-mt)

# external projects
set(EP_BASE_DIR "${CMAKE_BINARY_DIR}/CMakeExternals")
set_property(DIRECTORY PROPERTY EP_BASE ${EP_BASE_DIR})
set(EP_INSTALL_ROOT ${EP_BASE_DIR}/InstallRoot${CMAKE_INSTALL_PREFIX})

# install/developer mode
if(DEV_MODE)
    message(STATUS "configuring for in source build")
    set(DATADIR ${PROJECT_SOURCE_DIR}/data)
    set(LIBDIR ${CMAKE_BINARY_DIR}/bin)
    set(BINDIR ${CMAKE_BINARY_DIR}/bin)
	set(EP_DATA_ROOT ${EP_INSTALL_ROOT}/share)
else()
    message(STATUS "configuring for install build")
    set(DATADIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data)
    set(LIBDIR ${CMAKE_INSTALL_PREFIX}/lib)
    set(BINDIR ${CMAKE_INSTALL_PREFIX}/bin)
	set(EP_DATA_ROOT ${CMAKE_INSTALL_PREFIX}/share)
endif()

# static lib prefix
if(WIN32)
    set(STATIC_LIB_PREFIX "")
elseif(APPLE)
    set(STATIC_LIB_PREFIX "lib")
elseif(UNIX)
    set(STATIC_LIB_PREFIX "lib")
else()
    message(FATAL_ERROR "unknown operating system")
endif()

if( NOT MAVLINK_FOUND)
    ExternalProject_Add(mavlink
        GIT_REPOSITORY "git://github.com/pixhawk/mavlink.git"
		GIT_TAG ${MAVLINK_VERSION}
		CONFIGURE_COMMAND ""
		BUILD_COMMAND ""
		INSTALL_COMMAND ""
        INSTALL_DIR ${EP_INSTALL_ROOT}
        UPDATE_COMMAND ""
		BUILD_IN_SOURCE 1
	)
	set(MAVLINK_FOUND TRUE)
	set(MAVLINK_INCLUDE_DIRS ${EP_BASE_DIR}/Source/mavlink/include)
endif()

set(ARKSCICOS_LIBRARIES "")

# install data files
#install(DIRECTORY "${PROJECT_SOURCE_DIR}/data" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)

install(DIRECTORY "${PROJECT_SOURCE_DIR}/arkscicos-toolbox"
    DESTINATION ${SCICOSLAB_CONTRIB_DIR}
	PATTERN "*.git*" EXCLUDE)

# install required dependencies from external projects
install(DIRECTORY "${EP_BASE_DIR}/InstallRoot${CMAKE_INSTALL_PREFIX}/include" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY "${EP_BASE_DIR}/InstallRoot${CMAKE_INSTALL_PREFIX}/lib" DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY "${EP_BASE_DIR}/InstallRoot${CMAKE_INSTALL_PREFIX}/share" DESTINATION ${CMAKE_INSTALL_PREFIX})

# pkgconfig
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pc.cmake ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc)
install(FILES
	${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION lib/pkgconfig
	)

include_directories(src)

# math library
set(math_src
	src/math/FGColumnVector3.cpp
	src/math/FGCondition.cpp
	src/math/FGFunction.cpp
	src/math/FGLocation.cpp
	src/math/FGMatrix33.cpp
	src/math/FGModelFunctions.cpp
	src/math/FGNelderMead.cpp
	src/math/FGPropertyValue.cpp
	src/math/FGQuaternion.cpp
	src/math/FGRealValue.cpp
	src/math/FGRungeKutta.cpp
	src/math/FGStateSpace.cpp
	src/math/FGTable.cpp
	)
set(math_hdr
	src/math/FGColumnVector3.h
	src/math/FGCondition.h
	src/math/FGFunction.h
	src/math/FGLocation.h
	src/math/FGMatrix33.h
	src/math/FGModelFunctions.h
	src/math/FGNelderMead.h
	src/math/FGParameter.h
	src/math/FGPropertyValue.h
	src/math/FGQuaternion.h
	src/math/FGRealValue.h
	src/math/FGRungeKutta.h
	src/math/FGStateSpace.h
	src/math/FGTable.h
)
add_library(math ${math_src})

# intialization library
set(initilization_src 
	src/initialization/FGInitialCondition.cpp
	src/initialization/FGSimplexTrim.cpp
	#src/initialization/FGTrimAnalysisControl.cpp
	#src/initialization/FGTrimAnalysis.cpp
	src/initialization/FGTrimAxis.cpp
	src/initialization/FGTrim.cpp
	src/initialization/FGTrimmer.cpp
	)
set(initialization_hdr
	src/initialization/FGInitialCondition.h
	src/initialization/FGSimplexTrim.h
	src/initialization/FGTrimAnalysisControl.h
	src/initialization/FGTrimAnalysis.h
	src/initialization/FGTrimAxis.h
	src/initialization/FGTrim.h
	src/initialization/FGTrimmer.h
	)
add_library(initialization ${initilization_src})

# input_output library
set(input_output_src
	src/input_output/FGfdmSocket.cpp
	src/input_output/FGGroundCallback.cpp
	src/input_output/FGPropertyManager.cpp
	src/input_output/FGScript.cpp
	src/input_output/FGXMLElement.cpp
	src/input_output/FGXMLParse.cpp
	src/input_output/flightGearIO.cpp
	)
set(input_output_hdr
	src/input_output/FGfdmSocket.h
	src/input_output/FGGroundCallback.h
	src/input_output/FGPropertyManager.h
	src/input_output/FGScript.h
	src/input_output/FGXMLElement.h
	src/input_output/FGXMLFileRead.h
	src/input_output/FGXMLParse.h
	src/input_output/flightGearIO.h
	src/input_output/string_utilities.h
	)
add_library(input_output ${input_output_src})

# models
set(models_src
	src/models/FGAerodynamics.cpp
	src/models/FGAircraft.cpp
	src/models/FGAtmosphere.cpp
	src/models/FGAuxiliary.cpp
	src/models/FGBuoyantForces.cpp
	src/models/FGExternalForce.cpp
	src/models/FGExternalReactions.cpp
	src/models/FGFCS.cpp
	src/models/FGGasCell.cpp
	src/models/FGGroundReactions.cpp
	src/models/FGInertial.cpp
	src/models/FGInput.cpp
	src/models/FGLGear.cpp
	src/models/FGMassBalance.cpp
	src/models/FGModel.cpp
	src/models/FGOutput.cpp
	src/models/FGPropagate.cpp
	src/models/FGPropulsion.cpp
	)
set(models_hdr
	src/models/FGAerodynamics.h
	src/models/FGAircraft.h
	src/models/FGAtmosphere.h
	src/models/FGAuxiliary.h
	src/models/FGBuoyantForces.h
	src/models/FGExternalForce.h
	src/models/FGExternalReactions.h
	src/models/FGFCS.h
	src/models/FGGasCell.h
	src/models/FGGroundReactions.h
	src/models/FGInertial.h
	src/models/FGInput.h
	src/models/FGLGear.h
	src/models/FGMassBalance.h
	src/models/FGModel.h
	src/models/FGOutput.h
	src/models/FGPropagate.h
	src/models/FGPropulsion.h
	)
add_library(models ${models_src})

# utilities library
set(utilities_src
	src/utilities/datafile.cpp
	src/utilities/main.cpp
	src/utilities/plotXMLVisitor.cpp
	src/utilities/prep_plot.cpp
	src/utilities/XMLParseTest.cpp
	)
set(utilities_hdr
	src/utilities/datafile.h
	src/utilities/plotXMLVisitor.h
	)
add_library(utilities ${utilities_src})

# simgear library
set(simgear_src
	)	
set(simgear_hdr
	src/simgear/compiler.h
	)
#add_library(simgear ${simgear_src})

# vim:sw=4:ts=4:expandtab
