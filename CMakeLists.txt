project(jsbsim CXX C)
cmake_minimum_required(VERSION 2.8)
set(PROJECT_VERSION_MAJOR "1")
set(PROJECT_VERSION_MINOR "0")
set(PROJECT_VERSION_PATCH "0")
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
set(PROJECT_CONTACT_EMAIL james.goppert@gmail.com)
set(PROJECT_CONTACT_VENDOR JSBSim)
set(LIBRARY_VERSION ${APPLICATION_VERSION})
set(LIBRARY_SOVERSION "0.0.0")

set(ARKOSG_VERSION "e3685358553bd0a449ba22b452f5c7c2ee361c9b")
set(ARKCOMM_VERSION "3ebe1a8f224ceffda623361fdc876dfa7b2feb42")
set(MAVLINK_VERSION "3eb931ac38b94854aa47483739cb1e4492e9120a")

# variables
option(WITH_BUILD_DEPS "Build dependencies." ON)
option(WITH_BUILD_STATIC "Build statically." ON)
option(WITH_GUI "Build with GUI." OFF)
option(WITH_ARKOSG "Build arkosg graphics." OFF)
set(ROOT_THREAD TRUE CACHE INTERNAL "Is this the top level of the recursion?")
set(INSTALLED_TARGETS "")

# modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_SOURCE_DIR}/cmake/arkcmake)
include(BundleUtilities)
include(CheckIncludeFiles)
include(CheckFunctionExists)
include(CheckSymbolExists)
include(CheckLibraryExists)
#include(CheckTypeSize)
#include(CheckPrototypeExists)
#include(CheckCXXSourceCompiles)
#include(CheckCSourceCompiles)
include(ExternalProject)
include(FindARKOSG)
#include(FindMAVLINK)
#include(FindARKCOMM)

# spawn new cmake to build deps
if (WITH_BUILD_DEPS AND ROOT_THREAD)
    execute_process(COMMAND ${CMAKE_COMMAND} "${CMAKE_SOURCE_DIR}"
        "-DROOT_THREAD=FALSE"
        "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
        "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
        "-DWITH_BUILD_STATIC=${WITH_BUILD_STATIC}"
        "-DWITH_GUI=${WITH_GUI}"
        "-DWITH_ARKOSG=${WITH_ARKOSG}"
        RESULT_VARIABLE ERROR)
    if (ERROR)
        message(FATAL_ERROR "error, recursing loop returned error code: ${ERROR}")
    endif()
    message("** Making dependencies")
    execute_process(COMMAND ${CMAKE_MAKE_PROGRAM} "-j4" "-f${CMAKE_BINARY_DIR}/Makefile")
    message("** Configuring ${PROJECT_NAME}")
endif()

# external projects find path
if(NOT EP_BASE_DIR)
    set(EP_BASE_DIR "${CMAKE_BINARY_DIR}/CMakeExternals")
endif()
list(APPEND CMAKE_FIND_ROOT_PATH ${EP_BASE_DIR})

# prefer static packages if building static library
message("** Finding libraries")
if (WITH_BUILD_STATIC)
    set(BOOST_USE_STATIC_LIBS ON)
    # prefer static libs
    if(WIN32)
        set(CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
    else()
        set(CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
    endif()
endif()

# find libraries with cmake modules
if (WITH_GUI AND WITH_ARKOSG)
    if (WIN32)
        find_package(Boost 1.40.0 COMPONENTS thread_win32-mt system-mt REQUIRED)
    else()
        find_package(Boost 1.40.0 COMPONENTS thread-mt system-mt REQUIRED)
    endif()
    find_package(Qt4 COMPONENTS QtOpenGL QtGui QtCore REQUIRED)
    find_package(OpenSceneGraph 2.8.0 REQUIRED COMPONENTS osgViewer osgUtil osgDB osgGA osgManipulator)
    find_package(ARKOSG)
elseif(WITH_GUI)
    find_package(Qt4 COMPONENTS QtGui QtCore REQUIRED)
endif()

# build dependencies
if (WITH_BUILD_DEPS AND (NOT ROOT_THREAD) )
    message("** Configuring dependencies")

    # add external projects

    # arkosg
    if (WITH_ARKOSG AND WITH_GUI AND (NOT ARKOSG_FOUND))
        set(CMAKE_ARGS_ARKOSG
                -DEP_BASE_DIR=${EP_BASE_DIR}
                -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
                -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
            )
        build_arkosg(${ARKOSG_VERSION} ${EP_BASE_DIR} "${CMAKE_ARGS_ARKOSG}")
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
check_function_exists(bcopy HAVE_BCOPY)
#check_symbol_exists(LC_MESSAGES "locale.h" HAVE_LC_MESSAGES) 
check_library_exists(arkosg main "" HAVE_ARKOSG)

# config files
set(INSTALL_DATA_DIR "${CMAKE_INSTALL_PREFIX}/share/jsbsim")
configure_file(config.h.in config.h)
configure_file(gui/gui_config.h.in gui/gui_config.h)

# build settings
if (HAVE_EXPAT_CONFIG_H)
    add_definitions(-DHAVE_EXPAT_CONFIG_H)
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
include_directories(${CMAKE_SOURCE_DIR}/src)

# headers
set(JSBSIM_PUBLIC_HDRS
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
    src/math/LagrangeMultiplier.h
    
    src/simgear/compiler.h
    
    src/simgear/props/props.hxx
    
    src/simgear/magvar/coremag.hxx
    
    src/simgear/misc/stdint.hxx
    
    src/simgear/xml/utf8tab.h
    src/simgear/xml/internal.h
    src/simgear/xml/ascii.h
    src/simgear/xml/expat.h
    src/simgear/xml/nametab.h
    src/simgear/xml/winconfig.h
    #src/simgear/xml/xmltok_impl.h
    src/simgear/xml/easyxml.hxx
    src/simgear/xml/macconfig.h
    src/simgear/xml/expat_config.h
    src/simgear/xml/iasciitab.h
    src/simgear/xml/xmlrole.h
    src/simgear/xml/expat_external.h
    src/simgear/xml/latin1tab.h
    src/simgear/xml/asciitab.h
    src/simgear/xml/xmltok.h

    src/input_output/FGXMLElement.h
    src/input_output/FGXMLParse.h
    src/input_output/FGfdmSocket.h
    src/input_output/string_utilities.h
    src/input_output/FGXMLFileRead.h
    src/input_output/net_fdm.hxx
    src/input_output/FGScript.h
    src/input_output/FGGroundCallback.h
    src/input_output/FGPropertyManager.h

    src/models/FGAccelerations.h
    src/models/FGMassBalance.h
    src/models/FGFCS.h
    src/models/FGLGear.h
    src/models/FGInput.h
    src/models/FGAerodynamics.h
    src/models/FGAuxiliary.h
    src/models/FGBuoyantForces.h
    src/models/FGInertial.h
    src/models/FGPropulsion.h
    src/models/FGOutput.h
    src/models/FGExternalReactions.h
    src/models/FGAtmosphere.h
    src/models/atmosphere/FGMSIS.h
    src/models/atmosphere/FGMars.h
    src/models/FGAircraft.h
    src/models/FGExternalForce.h
    src/models/FGModel.h
    src/models/FGGasCell.h
    src/models/FGPropagate.h
    src/models/FGGroundReactions.h

    src/models/atmosphere/FGMSIS.h
    src/models/atmosphere/FGMars.h
    src/models/atmosphere/FGWinds.h
    src/models/atmosphere/FGStandardAtmosphere.h

    src/models/flight_control/FGAccelerometer.h
    src/models/flight_control/FGFCSFunction.h
    src/models/flight_control/FGDeadBand.h
    src/models/flight_control/FGSensorOrientation.h
    src/models/flight_control/FGGyro.h
    src/models/flight_control/FGGain.h
    src/models/flight_control/FGSwitch.h
    src/models/flight_control/FGFCSComponent.h
    src/models/flight_control/FGSummer.h
    src/models/flight_control/FGPID.h
    src/models/flight_control/FGActuator.h
    src/models/flight_control/FGSensor.h
    src/models/flight_control/FGFilter.h
    src/models/flight_control/FGMagnetometer.h
    src/models/flight_control/FGKinemat.h

    src/models/propulsion/FGElectric.h
    src/models/propulsion/FGTurbine.h
    src/models/propulsion/FGThruster.h
    src/models/propulsion/FGRocket.h
    src/models/propulsion/FGNozzle.h
    src/models/propulsion/FGTurboProp.h
    src/models/propulsion/FGForce.h
    src/models/propulsion/FGRotor.h
    src/models/propulsion/FGPiston.h
    src/models/propulsion/FGEngine.h
    src/models/propulsion/FGTank.h
    src/models/propulsion/FGPropeller.h

    src/initialization/FGInitialCondition.h
    src/initialization/FGSimplexTrim.h
    #src/initialization/FGTrimAnalysisControl.h
    #src/initialization/FGTrimAnalysis.h
    src/initialization/FGTrimAxis.h
    src/initialization/FGTrim.h
    src/initialization/FGTrimmer.h

    src/FGFDMExec.h
    src/FGJSBBase.h
    )

# sources
set(JSBSIM_SRCS
    src/input_output/FGfdmSocket.cpp
    src/input_output/FGXMLParse.cpp
    src/input_output/FGScript.cpp
    src/input_output/FGGroundCallback.cpp
    src/input_output/FGXMLElement.cpp
    src/input_output/FGPropertyManager.cpp

    #src/simgear/xml/xmltok_impl.c
    #src/simgear/xml/easyxml.cpp
    src/simgear/xml/xmlrole.c
    src/simgear/xml/easyxml.cxx
    src/simgear/xml/xmltok.c
    #src/simgear/xml/xmltok_ns.c
    src/simgear/xml/xmlparse.c
    src/simgear/props/props.cxx
    src/simgear/magvar/coremag.cxx

    src/math/FGQuaternion.cpp
    src/math/FGColumnVector3.cpp
    src/math/FGStateSpace.cpp
    src/math/FGPropertyValue.cpp
    src/math/FGRungeKutta.cpp
    src/math/FGRealValue.cpp
    src/math/FGModelFunctions.cpp
    src/math/FGFunction.cpp
    src/math/FGNelderMead.cpp
    src/math/FGTable.cpp
    src/math/FGLocation.cpp
    src/math/FGMatrix33.cpp
    src/math/FGCondition.cpp

    src/models/FGAccelerations.cpp
    src/models/flight_control/FGAccelerometer.cpp
    src/models/flight_control/FGSwitch.cpp
    src/models/flight_control/FGPID.cpp
    src/models/flight_control/FGFCSFunction.cpp
    src/models/flight_control/FGSummer.cpp
    src/models/flight_control/FGDeadBand.cpp
    src/models/flight_control/FGGyro.cpp
    src/models/flight_control/FGActuator.cpp
    src/models/flight_control/FGFCSComponent.cpp
    src/models/flight_control/FGMagnetometer.cpp
    src/models/flight_control/FGFilter.cpp
    src/models/flight_control/FGGain.cpp
    src/models/flight_control/FGSensor.cpp
    src/models/flight_control/FGKinemat.cpp
    src/models/FGBuoyantForces.cpp
    src/models/FGExternalForce.cpp
    src/models/FGLGear.cpp
    src/models/FGInput.cpp
    src/models/FGInertial.cpp
    src/models/FGFCS.cpp
    src/models/FGPropagate.cpp
    src/models/FGAuxiliary.cpp
    src/models/FGPropulsion.cpp
    src/models/FGOutput.cpp
    src/models/FGGasCell.cpp
    src/models/FGMassBalance.cpp
    src/models/FGAtmosphere.cpp
    src/models/atmosphere/FGStandardAtmosphere.cpp
    src/models/atmosphere/FGWinds.cpp
    src/models/atmosphere/FGMSISData.cpp
    src/models/atmosphere/FGMSIS.cpp
    src/models/atmosphere/FGMars.cpp
    src/models/FGAircraft.cpp
    src/models/FGExternalReactions.cpp
    src/models/FGGroundReactions.cpp
    src/models/propulsion/FGTank.cpp
    src/models/propulsion/FGForce.cpp
    src/models/propulsion/FGThruster.cpp
    src/models/propulsion/FGPropeller.cpp
    src/models/propulsion/FGTurbine.cpp
    src/models/propulsion/FGElectric.cpp
    src/models/propulsion/FGRotor.cpp
    src/models/propulsion/FGPiston.cpp
    src/models/propulsion/FGTurboProp.cpp
    src/models/propulsion/FGNozzle.cpp
    src/models/propulsion/FGRocket.cpp
    src/models/propulsion/FGEngine.cpp
    src/models/FGAerodynamics.cpp
    src/models/FGModel.cpp

    src/initialization/FGInitialCondition.cpp
    src/initialization/FGSimplexTrim.cpp
    #src/initialization/FGTrimAnalysisControl.cpp
    #src/initialization/FGTrimAnalysis.cpp
    src/initialization/FGTrimAxis.cpp
    src/initialization/FGTrim.cpp
    src/initialization/FGTrimmer.cpp

    src/FGFDMExec.cpp
    src/FGJSBBase.cpp
    )

# private headers
set(JSBSIM_PRVIATE_HDRS
    )

# gui
if (WITH_GUI)
    add_subdirectory(gui)
endif()

# set icns file containing icons
if (APPLE)
    # set how it shows up in Info.plist
    set(MACOSX_BUNDLE_CON_FILE jsbsim.icns)
    # set where in the bundle to put the icns file
    set_source_files_properties(jsbsim.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    # include the icns file in the target
    list(APPEND JSBSIM_SRCS jsbsim.icns)
endif(APPLE)

# library
if (WITH_BUILD_STATIC)
    add_library(jsbsim STATIC ${JSBSIM_SRCS} ${JSBSIM_PUBLIC_HDRS} ${JSBSIM_PRIVATE_HDRS})
else(WITH_BUILD_STATIC)
    add_library(jsbsim SHARED ${JSBSIM_SRCS} ${JSBSIM_PUBLIC_HDRS} ${JSBSIM_PRIVATE_HDRS})
endif(WITH_BUILD_STATIC)

# these properties tell mac how to structure the framework
set_property(TARGET jsbsim PROPERTY FRAMEWORK ON)
set_property(TARGET jsbsim PROPERTY PUBLIC_HEADER ${JSBSIM_HDRS})
set_property(TARGET jsbsim PROPERTY PRIVATE_HEADER ${JSBSIM_HDRS})

# executable linking to static library
add_executable(jsbsim-cmd MACOSX_BUNDLE WIN32 src/JSBSim.cpp)
target_link_libraries(jsbsim-cmd jsbsim)
list(APPEND INSTALLED_TARGETS jsbsim-cmd jsbsim)

# install
install(TARGETS ${INSTALLED_TARGETS} 
    RUNTIME DESTINATION "bin" COMPONENT Runtime
    LIBRARY DESTINATION "lib" COMPONENT Runtime
    ARCHIVE DESTINATION "lib" COMPONENT Runtime
    BUNDLE DESTINATION "/Applications" COMPONENT Runtime
    FRAMEWORK DESTINATION "/Library/Frameworks" COMPONENT Runtime
    # where to put files on non-mac computers, mac override and uses default
    PRIVATE_HEADER DESTINATION "include/jsbsim" COMPONENT Runtime
    PUBLIC_HEADER DESTINATION "include/jsbsim" COMPONENT Runtime
    RESOURCE DESTINATION "share/jsbsim" COMPONENT Runtime
    )

# install external projects, if built
if (EXISTS ${EP_BASE_DIR}/${CMAKE_INSTALL_PREFIX}/)
    install(DIRECTORY ${EP_BASE_DIR}/${CMAKE_INSTALL_PREFIX}/ DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

# packaging
set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
set(CPACK_SET_DESTDIR TRUE)
include(CPack)

# pkgconfig
configure_file(pc.in ${PROJECT_NAME}.pc)
install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc
    DESTINATION "lib/pkgconfig"
    )
