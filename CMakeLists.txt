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
option(VERBOSE_LIB "Verbose library building." OFF)
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

macro(add_jsbsim_library)

	cmake_parse_arguments(PARSE "FORTRAN" "LIB_NAME;DIR;LIBS" "SOURCES;HEADERS;EXCEPT" ${ARGN})

	message(STATUS "adding library: ${PARSE_LIB_NAME}")

	if("${PARSE_SOURCES}" STREQUAL "")
		message(STATUS "\tauto detecting sources")
		file(GLOB_RECURSE ${PARSE_LIB_NAME}_src ${PARSE_DIR}/*.c*)
		if (FORTRAN)
			message(STATUS "\tlooking for fortran sources")
			file(GLOB_RECURSE ${PARSE_LIB_NAME}_f_src ${PARSE_DIR}/*.f*)
			list(APPEND ${PARSE_LIB_NAME}_src ${${PARSE_LIB_NAME}_f_src})
		endif()
	else()
		message(STATUS "\tsources manually specified")
		string(REPLACE "\n" " " PARSE_SOURCES "${PARSE_SOURCES}")
		string(REPLACE "\t" "" PARSE_SOURCES "${PARSE_SOURCES}")
		separate_arguments(PARSE_SOURCES)
		set(${PARSE_LIB_NAME}_src ${PARSE_SOURCES})
	endif()

	if("${PARSE_HEADERS}" STREQUAL "")
		message(STATUS "\tauto detecting headers")
		file(GLOB_RECURSE ${PARSE_LIB_NAME}_hdr ${PARSE_DIR}/*.h*)
	else()
		message(STATUS "\theaders manually specified")
		string(REPLACE "\n" " " PARSE_HEADERS "${PARSE_HEADERS}")
		string(REPLACE "\t" "" PARSE_HEADERS "${PARSE_HEADERS}")
		separate_arguments(PARSE_HEADERS)
		set(${PARSE_LIB_NAME}_hdr ${PARSE_HEADERS})
	endif()

	if(NOT "${PARSE_EXCEPT}" STREQUAL "")
		string(REPLACE "\n" " " PARSE_EXCEPT "${PARSE_EXCEPT}")
		string(REPLACE "\t" "" PARSE_EXCEPT "${PARSE_EXCEPT}")
		separate_arguments(PARSE_EXCEPT)
		list(REMOVE_ITEM ${PARSE_LIB_NAME}_hdr ${PARSE_EXCEPT})
		list(REMOVE_ITEM ${PARSE_LIB_NAME}_src ${PARSE_EXCEPT})
		if(VERBOSE_LIB)
			message(STATUS "\texcept: ")
			foreach(except ${PARSE_EXCEPT})
				message(STATUS "\t\t${except}")
			endforeach()
		endif()
	endif()

	if(VERBOSE_LIB)
		message(STATUS "\theaders: ")
		foreach(hdr ${${PARSE_LIB_NAME}_hdr})
			message(STATUS "\t\t${hdr}")
		endforeach()
		message(STATUS "\tsources: ")
		foreach(hdr ${${PARSE_LIB_NAME}_src})
			message(STATUS "\t\t${hdr}")
		endforeach()
	endif()

	add_library(${PARSE_LIB_NAME} ${${PARSE_LIB_NAME}_src}) 

	if (NOT "${PARSE_LIBS}" STREQUAL "")
		string(REPLACE "\n" " " PARSE_LIBS "${PARSE_LIBS}")
		string(REPLACE "\t" "" PARSE_LIBS "${PARSE_LIBS}")
		separate_arguments(PARSE_LIBS)
		message(STATUS "\tlibraries: ")
		foreach(lib ${PARSE_LIBS})
			message(STATUS "\t\t${lib}")
		endforeach()
		target_link_libraries(${PARSE_LIB_NAME} ${PARSE_LIBS})
	endif()

endmacro()


include_directories(src)

add_jsbsim_library(LIB_NAME "initialization" 
	DIR "src/initialization" 
	EXCEPT "
		${CMAKE_CURRENT_SOURCE_DIR}/src/initialization/FGTrimAnalysis.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/initialization/FGTrimAnalysisControl.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/initialization/FGTrimAnalysis.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/initialization/FGTrimAnalysisControl.h"
	)
add_jsbsim_library(LIB_NAME "math" DIR "src/math")
add_jsbsim_library(LIB_NAME "input_output" DIR "src/input_output")
add_jsbsim_library(LIB_NAME "models" DIR "src/models" LIBS "math input_output")
add_jsbsim_library(LIB_NAME "simgear" DIR "src/simgear"
	SOURCES "
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/xmlparse.c
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/xmltok.c
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/xmlrole.c
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/easyxml.cxx
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/props/props.cxx"
	HEADERS "
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/ascii.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/asciitab.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/easyxml.hxx
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/expat_config.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/expat_external.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/expat.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/iasciitab.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/internal.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/latin1tab.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/macconfig.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/nametab.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/utf8tab.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/winconfig.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/xmlrole.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/xmltok.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/xml/xmltok_impl.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/props/props.hxx
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/props/SGReferenced.hxx
		${CMAKE_CURRENT_SOURCE_DIR}/src/simgear/props/SGSharedPtr.hxx"
	)
add_executable(jsbsim
	src/JSBSim.cpp
	src/FGFDMExec.cpp
	src/FGJSBBase.cpp
	src/FGState.cpp
	)
target_link_libraries(jsbsim
	math
	input_output
	models
	initialization
	simgear
	)

# vim:sw=4:ts=4:expandtab
