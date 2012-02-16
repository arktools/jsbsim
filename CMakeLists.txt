project (jsbsim)

# settings
cmake_minimum_required (VERSION 2.8.2)
set(APPLICATION_NAME ${PROJECT_NAME})
set(APPLICATION_VERSION_MAJOR "1")
set(APPLICATION_VERSION_MINOR "0")
set(APPLICATION_VERSION_PATCH "0.102")
set(APPLICATION_VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}.${APPLICATION_VERSION_PATCH}")
set(LIBRARY_SOVERSION "0.0.0")
set(ARKOSG_VERSION "e3685358553bd0a449ba22b452f5c7c2ee361c9b")
set(ARKCOMM_VERSION "2f03138a14c4e3dfe39ddcb6c6b1c37720df0416")
set(MAVLINK_VERSION "3eb931ac38b94854aa47483739cb1e4492e9120a")
set(PROJECT_CONTACT_EMAIL james.goppert@gmail.com)
set(PROJECT_CONTACT_VENDOR JSBSim)

# modules
set(CMAKE_MODULE_PATH
	${CMAKE_SOURCE_DIR}/cmake
	${CMAKE_SOURCE_DIR}/cmake/arkcmake
)
#include(FindMAVLINK)
#include(FindARKCOMM)

# use common setup
include(CommonSetup)
set(LIBRARY_VERSION ${APPLICATION_VERSION})
include(JSBSimCPackPackage)

# settings
option(WITH_BUILD_STATIC "Prefer static linking." ON)
option(WITH_GUI "Build the gui." OFF)
option(WITH_ARKOSG "Build the arkosg." OFF)

# static lib options
if (WITH_BUILD_STATIC)
    if (WITH_ARKOSG)
        set(BOOST_USE_STATIC_LIBS ON)
    endif()
    # prefer static libs
    if(WIN32)
        set(CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
    else()
        set(CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
    endif()
endif()

# enable languages
enable_language(C)
enable_language(CXX)

if (WIN32)
    add_definitions(-D_WIN32_WINNT=0x0501) # target xp
elseif(APPLE)
elseif(UNIX)
    add_definitions(-DUSE_X11)
else()
    message(ERROR "unknown operating system")
endif()

# find libraries with cmake modules
if (WITH_GUI)
    if (WITH_ARKOSG)
        if (WIN32)
            find_package(Boost 1.40.0 COMPONENTS thread_win32-mt system-mt REQUIRED)
        else()
            find_package(Boost 1.40.0 COMPONENTS thread-mt system-mt REQUIRED)
        endif()
        find_package(Qt4 COMPONENTS QtOpenGL QtGui QtCore REQUIRED)
        find_package(OpenGL REQUIRED)
        find_package(OpenSceneGraph 2.8.0 REQUIRED COMPONENTS osgViewer osgUtil osgDB osgGA osgManipulator)
        include(FindARKOSG)
        find_package(ARKOSG ${ARKOSG_VERSIOIN})
        if (NOT ARKOSG_FOUND)
            build_arkosg(${ARKOSG_VERSION} ${EP_BASE_DIR})
        endif()
    else()
        find_package(Qt4 COMPONENTS QtGui QtCore REQUIRED)
    endif()
endif()

#find_or_build_arkcomm(${ARKCOMM_VERSION} ${EP_BASE_DIR})

# install data files
install(DIRECTORY "${PROJECT_SOURCE_DIR}/aircraft" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/data_output" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/data_plot" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/engine" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/scripts" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/engine" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/systems" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)

# install required dependencies from external projects
if (WITH_GUI AND WITH_ARKOSG)
    install(DIRECTORY ${EP_BASE_DIR}/${CMAKE_INSTALL_PREFIX}/ DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

# pkgconfig
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pc.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc)
install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc
    DESTINATION lib/pkgconfig
    )

# config file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gui/gui_config.h.in ${CMAKE_CURRENT_BINARY_DIR}/gui/gui_config.h)

# build settings
include_directories(src src/simgear/xml)
add_definitions(-DHAVE_EXPAT_CONFIG_H)
if (MSVC)
    add_definitions(-D_USE_MATH_DEFINES)
endif()

install(FILES
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
    DESTINATION include/jsbsim/math
    )
install(FILES
    src/simgear/compiler.h
    DESTINATION include/jsbsim/simgear
    )
install(FILES
    src/simgear/props/props.hxx
    DESTINATION include/jsbsim/simgear/props
    )
install(FILES
    src/simgear/magvar/coremag.hxx
    DESTINATION include/jsbsim/simgear/magvar
    )
install(FILES
    src/simgear/misc/stdint.hxx
    DESTINATION include/jsbsim/simgear/misc
    )
install(FILES
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
    DESTINATION include/jsbsim/simgear/xml
    )
install(FILES
    src/input_output/FGXMLElement.h
    src/input_output/FGXMLParse.h
    src/input_output/FGfdmSocket.h
    src/input_output/string_utilities.h
    src/input_output/FGXMLFileRead.h
    src/input_output/net_fdm.hxx
    src/input_output/FGScript.h
    src/input_output/FGGroundCallback.h
    src/input_output/FGPropertyManager.h
    DESTINATION include/jsbsim/input_output
    )
install(FILES
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
    DESTINATION include/jsbsim/models
    )
install(FILES
    src/models/atmosphere/FGMSIS.h
    src/models/atmosphere/FGMars.h
    src/models/atmosphere/FGWinds.h
    src/models/atmosphere/FGStandardAtmosphere.h
    DESTINATION include/jsbsim/models/atmosphere
    )
install(FILES
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
    DESTINATION include/jsbsim/models/flight_control
    )
install(FILES
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
    DESTINATION include/jsbsim/models/propulsion
    )
install(FILES
    src/initialization/FGInitialCondition.h
    src/initialization/FGSimplexTrim.h
    #src/initialization/FGTrimAnalysisControl.h
    #src/initialization/FGTrimAnalysis.h
    src/initialization/FGTrimAxis.h
    src/initialization/FGTrim.h
    src/initialization/FGTrimmer.h
    DESTINATION include/jsbsim/initialization
    )
install(FILES
    src/FGFDMExec.h
    src/FGJSBBase.h
    DESTINATION include/jsbsim
    )

# jsbsim library
add_library(libjsbsim 
    src/input_output/FGfdmSocket.cpp
    src/input_output/FGXMLParse.cpp
    src/input_output/FGScript.cpp
    src/input_output/FGGroundCallback.cpp
    src/input_output/FGXMLElement.cpp
    src/input_output/FGPropertyManager.cpp

    #src/simgear/xml/xmltok_impl.c
    src/simgear/xml/easyxml.cpp
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

# Remove prefix 'lib' in any platform
set_target_properties(libjsbsim PROPERTIES PREFIX "")
if(NOT MSVC)
    target_link_libraries(libjsbsim m)
endif()
if(MINGW)
    find_library(WSOCK32 NAMES wsock32 ws2_32)
    target_link_libraries(libjsbsim ${WSOCK32})
endif()
install(TARGETS libjsbsim DESTINATION lib)

# jsbsim executable
add_executable(JSBSim
    src/JSBSim.cpp
    )
target_link_libraries(JSBSim libjsbsim)
install(TARGETS JSBSim DESTINATION bin)

# jsbsim gui
if (WITH_GUI)
    add_subdirectory(gui)
endif()

# vim:sw=4:ts=4:expandtab
