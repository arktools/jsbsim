project(jsbsim CXX C)

# settings
cmake_minimum_required(VERSION 2.8)
set(PROJECT_VERSION_MAJOR "1")
set(PROJECT_VERSION_MINOR "0")
set(PROJECT_VERSION_PATCH "0.112")
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
set(PROJECT_CONTACT_EMAIL james.goppert@gmail.com)
set(PROJECT_CONTACT_VENDOR JSBSim)
set(LIBRARY_VERSION ${PROJECT_VERSION})
set(LIBRARY_SOVERSION "0.0.0")

# third party library version
set(ARKOSG_REQUIRED_VERSION             0.2.3)
set(ARKCOMM_REQUIRED_VERSION            0.1.7)
set(OPENSCENEGRAPH_REQUIRED_VERSION     2.8.3) 
set(BOOST_REQUIRED_VERSION              1.42) 
set(QT_REQUIRED_VERSION                 4.6.3) 
set(SIMGEAR_REQUIRED_VERSION 2.7.0)

# md5 check sums
set(ARKOSG_MD5      33f3d9f09f14efe53e5d15a4919d4663)
set(ARKCOMM_MD5     a8151d8de8c5c560826858c39780a508)
set(SIMGEAR_MD5     fcb8eac3355778904b2d11e487033e4b)

# options
option(WITH_BUILD_DEPS "Build dependencies." OFF)
option(WITH_BUILD_SHARED "Build shared library." OFF)
option(WITH_PREFER_STATIC "Build preferring static linking." ON)
option(WITH_GUI "Build with GUI." OFF)
option(WITH_ARKOSG "Build arkosg graphics." OFF)
option(WITH_ARKCOMM "Build arkcomm communications." OFF)
option(WITH_BUNDLE "Attempt to package external shared library dependencies." OFF) # not working
option(WITH_WARNINGS "Enable warnings." OFF)

set(RECURSE_OPTIONS_LIST
    WITH_BUILD_DEPS
    WITH_BUILD_SHARED
    WITH_PREFER_STATIC
    WITH_GUI
    WITH_ARKOSG
    WITH_ARKCOMM
    WITH_BUNDLE
    WITH_WARNINGS
    CMAKE_TOOLCHAIN_FILE
    CMAKE_INSTALL_PREFIX
    EP_BASE_DIR
)

# variables
set(ROOT_THREAD TRUE CACHE INTERNAL "Is this the top level of the recursion?")
set(INSTALL_DATA_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
if (MINGW)
    set(INSTALL_DATA_DIR "C:/Program Files/${PROJECT_NAME} ${PROJECT_VERSION}/share/${PROJECT_NAME}")
endif()

# modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_SOURCE_DIR}/cmake/arkcmake)
include(DefineCMakeDefaults)
include(DefineCompilerFlags)
include(CheckIncludeFiles)
include(CheckFunctionExists)
include(CheckSymbolExists)
include(CheckLibraryExists)
#include(CheckTypeSize)
#include(CheckPrototypeExists)
#include(CheckCXXSourceCompiles)
#include(CheckCSourceCompiles)
include(ExternalProjectWithFilename)
include(CheckLinkType)

# external projects find path
if(NOT EP_BASE_DIR)
    set(EP_BASE_DIR "${CMAKE_BINARY_DIR}/CMakeExternals")
endif()
set_property(DIRECTORY PROPERTY EP_BASE ${EP_BASE_DIR})
set(EP_INSTALL_DIR "${EP_BASE_DIR}/Install")
list(APPEND CMAKE_FIND_ROOT_PATH ${EP_INSTALL_DIR})

# spawn new cmake to build deps
if (WITH_BUILD_DEPS AND ROOT_THREAD)
    set(CMAKE_RECURSE_ARGS "-DROOT_THREAD=FALSE")
    foreach(OPT ${RECURSE_OPTIONS_LIST})
        list(APPEND CMAKE_RECURSE_ARGS "-D${OPT}=${${OPT}}")
    endforeach()
    execute_process(COMMAND ${CMAKE_COMMAND} ${CMAKE_SOURCE_DIR} ${CMAKE_RECURSE_ARGS}
        RESULT_VARIABLE ERROR)
    if (ERROR)
        message(FATAL_ERROR "error, recursing loop returned error code: ${ERROR}")
    endif()
    message("** Making dependencies")
    execute_process(COMMAND ${CMAKE_MAKE_PROGRAM} "-j4" "-f${CMAKE_BINARY_DIR}/Makefile")
    message("** Configuring ${PROJECT_NAME}")
endif()

# prefer static packages if building static library
message("** Finding libraries")
if (WITH_PREFER_STATIC)
    set(BOOST_USE_STATIC_LIBS ON)
    # prefer static libs
    if(WIN32)
        set(CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
    else()
        set(CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
    endif()
endif()

# find libraries with cmake modules
set(REQUIRED_IF_ROOT_THREAD)
find_package(SimGear ${SIMGEAR_REQUIRED_VERSION} ${REQUIRED_IF_ROOT_THREAD} COMPONENTS io props xml structure misc debug magvar)
if (ROOT_THREAD)
    set(REQUIRED_IF_ROOT_THREAD "REQUIRED")
endif()
if (WITH_GUI AND WITH_ARKOSG)
    if (MSVC)
        set(Boost_USE_MULITHREADED ON)
        find_package(Boost ${BOOST_REQUIRED_VERSION} REQUIRED COMPONENTS thread system)
    elseif(MINGW)
        find_package(Boost ${BOOST_REQUIRED_VERSION} REQUIRED COMPONENTS thread_win32-mt system-mt)
    else()
        find_package(Boost ${BOOST_REQUIRED_VERSION} REQUIRED COMPONENTS thread-mt system-mt)
    endif()
    find_package(OpenGL REQUIRED)
    find_package(Qt4 ${QT_REQUIRED_VERSION} REQUIRED COMPONENTS QtOpenGL QtGui QtCore)
    find_package(OpenSceneGraph ${OPENSCENEGRAPH_REQUIRED_VERSION} REQUIRED COMPONENTS osgViewer osgUtil osgDB osgGA osgManipulator)
    find_package(OsgPlugin ${OPENSCENEGRAPH_REQUIRED_VERSION} REQUIRED COMPONENTS ac rgb)
    check_link_type(OSGPLUGIN ${OSGPLUGIN_AC})
    find_package(ArkOsg ${ARKOSG_REQUIRED_VERSION} ${REQUIRED_IF_ROOT_THREAD})
elseif(WITH_GUI)
    find_package(Qt4 ${QT_REQUIRED_VERSION} REQUIRED COMPONENTS QtGui QtCore REQUIRED)
endif()
if(WITH_ARKCOMM)
    find_package(ArkComm ${ARKCOMM_REQUIRED_VERSION} ${REQUIRED_IF_ROOT_THREAD})
endif()

# build dependencies
if (WITH_BUILD_DEPS AND (NOT ROOT_THREAD) )
    message("** Configuring dependencies")

    # add external projects

    set(CMAKE_EP_DEFAULT_ARGS
        -DEP_BASE_DIR=${EP_BASE_DIR}
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
        -DWITH_BUILD_DEPS=${WITH_BUILD_DEPS}
        )

    if (WITH_ARKCOMM  AND (NOT ARKCOMM_FOUND))
        ExternalProjectWithFilename_Add(arkcomm
            URL https://github.com/arktools/arkcomm/tarball/${ARKCOMM_REQUIRED_VERSION}
            URL_MD5 ${ARKCOMM_MD5}
            FILENAME arkcomm-${ARKCOMM_MD5}.tar.gz 
            CMAKE_ARGS ${CMAKE_EP_DEFAULT_ARGS}
            INSTALL_COMMAND ${CMAKE_MAKE_PROGRAM} DESTDIR=${EP_INSTALL_DIR} install
            INSTALL_DIR ${EP_INSTALL_DIR}
           )
    endif()

    if (WITH_ARKOSG AND WITH_GUI AND (NOT ARKOSG_FOUND))
        ExternalProjectWithFilename_Add(arkosg
            URL https://github.com/arktools/arkosg/tarball/${ARKOSG_REQUIRED_VERSION}
            URL_MD5 ${ARKOSG_MD5}
            FILENAME arkosg-${ARKOSG_MD5}.tar.gz
            CMAKE_ARGS ${CMAKE_EP_DEFAULT_ARGS}
            INSTALL_COMMAND ${CMAKE_MAKE_PROGRAM} DESTDIR=${EP_INSTALL_DIR} install
            INSTALL_DIR ${EP_INSTALL_DIR}
        )
    endif()

    if (NOT SIMGEAR_FOUND)
        ExternalProjectWithFilename_Add(simgear
            URL https://gitorious.org/fg/jgopperts-simgear/archive-tarball/mingw-fix
            URL_MD5 ${SIMGEAR_MD5}
            FILENAME jgoppert-jgopperts-simgear-mingw-fix.tar.gz
            CMAKE_ARGS ${CMAKE_EP_DEFAULT_ARGS} "-DSIMGEAR_HEADLESS=ON -DENABLE_TESTS=OFF"
            INSTALL_COMMAND ${CMAKE_MAKE_PROGRAM} DESTDIR=${EP_INSTALL_DIR} install
            INSTALL_DIR ${E_INSTALL_DIR}
           )
    endif()

    # terminate non root cmake thread
    return()
endif()

# configure
check_include_files(expat_config.h HAVE_EXPAT_CONFIG_H)
check_include_files(sys_types.h HAVE_SYS_TYPES_H)
check_include_files(sys_stat.h HAVE_SYS_STAT_H)
check_include_files(stdlib.h HAVE_STDLIB_H)
check_include_files(string.h HAVE_STRING_H)
check_include_files(inttypes.h HAVE_INTTYPES_H)
check_include_files(stdint.h HAVE_STDINT_H)
check_include_files(unistd.h HAVE_UNISTD_H)
check_function_exists(memcopy HAVE_MEMCOPY)
check_function_exists(memmove HAVE_MEMMOVE)
check_function_exists(bcopy HAVE_BCOPY)
#check_symbol_exists(LC_MESSAGES "locale.h" HAVE_LC_MESSAGES) 
#check_library_exists(arkosg  attachNode "" HAVE_ARKOSG)

# config files
configure_file(config.h.in config.h)
configure_file(gui/gui_config.h.in gui/gui_config.h)
install(FILES ${CMAKE_BINARY_DIR}/config.h DESTINATION include/${PROJECT_NAME} COMPONENT Dev)

# build settings
if (HAVE_EXPAT_CONFIG_H OR MSVC)
    add_definitions(-DHAVE_EXPAT_CONFIG_H)
endif()
if (HAVE_MEMMOVE)
    add_definitions(-DHAVE_MEMMOVE)
endif()
if (HAVE_BCOPY)
    add_definitions(-DHAVE_BCOPY)
endif()
if (HAVE_EXPAT_CONFIG_H)
    add_definitions(-DHAVE_EXPAT_CONFIG_H)
endif()
if (MSVC)
    add_definitions(-D_USE_MATH_DEFINES)
elseif(MINGW)
    add_definitions(-D_WIN32_WINNT=0x0501)
elseif(APPLE)
elseif(UNIX)
    add_definitions(-DUSE_X11)
else()
    message(ERROR "unknown operating system")
endif()
include_directories(src)

if (WITH_WARNINGS)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -pedantic")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wshadow -Wmissing-prototypes -Wdeclaration-after-statement")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused -Wfloat-equal -Wpointer-arith -Wwrite-strings -Wformat-security")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wmissing-format-attribute")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wshadow")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused -Wfloat-equal -Wpointer-arith -Wwrite-strings -Wformat-security")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wmissing-format-attribute")
endif()

# library
add_subdirectory(src)

# gui
if (WITH_GUI)
    add_subdirectory(gui)
endif()

# install data files
install(DIRECTORY "aircraft" DESTINATION share/${PROJECT_NAME} COMPONENT Runtime PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "data_output" DESTINATION share/${PROJECT_NAME} COMPONENT Runtime PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "data_plot" DESTINATION share/${PROJECT_NAME} COMPONENT Runtime PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "engine" DESTINATION share/${PROJECT_NAME} COMPONENT Runtime PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "scripts" DESTINATION share/${PROJECT_NAME} COMPONENT Runtime PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "engine" DESTINATION share/${PROJECT_NAME} COMPONENT Runtime PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "systems" DESTINATION share/${PROJECT_NAME} COMPONENT Runtime PATTERN "*.git*" EXCLUDE)

# packaging
if (WIN32)
    # set NSIS image, nsis bug requuires atleast one file with 4 slashes to install correctly
    set(CPACK_PACKAGE_ICON "${CMAKE_SOURCE_DIR}\\\\/cmake/jsbsim-nsis.bmp")
    set(CPACK_NSIS_MUI_ICON "${CMAKE_SOURCE_DIR}/cmake/jsbsim.ico")
    set(CPACK_NSIS_MUI_UNIICON "${CMAKE_SOURCE_DIR}/cmake/jsbsim.ico")
endif()
configure_file("COPYING" "COPYING.txt" COPYONLY) # adding file extensions
configure_file("README" "README.txt" COPYONLY)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_BINARY_DIR}/COPYING.txt")
set(CPACK_RESOURCE_FILE_README "${CMAKE_BINARY_DIR}/README.txt")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CPACK_RESOURCE_FILE_README}")
set(CPACK_RESOURCE_FILE_WELCOME "${CMAKE_SOURCE_DIR}/cmake/WELCOME.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "jsbsim simplex trimming branch")
set(CPACK_PACKAGE_VENDOR ${PROJECT_CONTACT_VENDOR})
set(CPACK_PACKAGE_CONTACT "${PROJECT_CONTACT_EMAIL}")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_SET_DESTDIR TRUE)
set(CPACK_COMPONENTS_GROUPING "ALL_COMPONENTS_IN_ONE")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libstdc++6, libgcc1, libc6")
if (WITH_GUI)
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libqtcore4, libqtgui4, libxcb1")
    if (WITH_ARKOSG)
        set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libqt4-opengl, libgl1-mesa-glx")
    endif()
endif()
set(CPACK_NSIS_MODIFY_PATH ON)
set(CPACK_PACKAGE_EXECUTABLES
    #"jsbsim-cmd" "Command Line"
    )
if (WITH_GUI)
    list(APPEND CPACK_PACKAGE_EXECUTABLES
        "jsbsim-gui" "Trimming Gui"
        )
    set(CPACK_CREATE_DESKTOP_LINKS "jsbsim-gui")
endif()
include(CPack)

# pkgconfig
configure_file(pc.in ${PROJECT_NAME}.pc)
install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc
    DESTINATION "lib/pkgconfig" COMPONENT Dev
    )
# vim:sw=4:ts=4:expandtab
