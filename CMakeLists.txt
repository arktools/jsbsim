project (jsbsim)

# Requried cmake version
cmake_minimum_required (VERSION 2.8.2)

# global needed variables
set(APPLICATION_NAME ${PROJECT_NAME})

# set these for release
set(APPLICATION_VERSION_MAJOR "1")
set(APPLICATION_VERSION_MINOR "100")
set(APPLICATION_VERSION_PATCH "0")
set(ARKOSG_VERSION "0.1.6")
set(ARKCOMM_VERSION "0.1.3")
set(MAVLINK_VERSION "0.9.0")

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
include(FindMAVLINK)

# cpack
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "arkTools scicoslab toolbox")
set(CPACK_PACKAGE_VENDOR "arkTools")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "james.goppert@gmail.com")
set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/COPYING")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README")
set(CPACK_PACKAGE_VERSION_MAJOR "${APPLICATION_VERSION_MAJOR}") 
set(CPACK_PACKAGE_VERSION_MINOR "${APPLICATION_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${APPLICATION_VERSION_PATCH}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
set(CPACK_SET_DESTDIR TRUE)
set(CPACK_SOURCE_IGNORE_FILES ${CPACK_SOURCE_IGNORE_FILES}
	/.git/;/build/;~$;.*\\\\.bin$;.*\\\\.swp$)
set(CPACK_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
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

# add make dist target
add_custom_target(dist COMMAND ${CMAKE_MAKE_PROGRAM} package_source)

# external projects
set(EP_BASE_DIR "${CMAKE_BINARY_DIR}/CMakeExternals")
set_property(DIRECTORY PROPERTY EP_BASE ${EP_BASE_DIR})
set(EP_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}-external)

# install/developer mode
if(DEV_MODE)
    message(STATUS "configuring for in source build")
    set(DATADIR ${PROJECT_SOURCE_DIR}/data)
    set(LIBDIR ${CMAKE_BINARY_DIR}/lib)
    set(BINDIR ${CMAKE_BINARY_DIR}/bin)
    set(EP_DATADIR ${EP_BASE_DIR}/${EP_INSTALL_PREFIX}/share)
else()
    message(STATUS "configuring for install build")
    set(DATADIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data)
    set(LIBDIR ${CMAKE_INSTALL_PREFIX}/lib)
    set(BINDIR ${CMAKE_INSTALL_PREFIX}/bin)
    set(EP_DATADIR ${EP_INSTALL_PREFIX}/share)
endif()

# find libraries with cmake modules
find_package(SCICOSLAB)
find_package(OpenSceneGraph 2.8.0 COMPONENTS osgGA osgDB osgUtil osgViewer)
find_package(ARKOSG)
find_package(ARKCOMM)
find_package(Boost 1.42 COMPONENTS thread-mt system-mt)
find_or_build_arkcomm(${ARKCOMM_VERSION} ${EP_BASE_DIR} ${EP_INSTALL_PREFIX} ${EP_DATADIR})
find_or_build_arkosg(${ARKOSG_VERSION} ${EP_BASE_DIR} ${EP_INSTALL_PREFIX} ${EP_DATADIR})
find_or_build_mavlink(${MAVLINK_VERSION} ${EP_BASE_DIR} ${EP_INSTALL_PREFIX} ${EP_DATADIR})

# install data files
install(DIRECTORY "${PROJECT_SOURCE_DIR}/aircraft" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/data_output" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/data_plot" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/engine" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/scripts" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/engine" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/scicoslab" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/systems" DESTINATION share/${PROJECT_NAME} PATTERN "*.git*" EXCLUDE)

# install required dependencies from external projects
install(DIRECTORY ${EP_BASE_DIR}/${EP_INSTALL_PREFIX} DESTINATION ${CMAKE_INSTALL_PREFIX})

# pkgconfig
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pc.cmake ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc)
install(FILES
	${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION lib/pkgconfig
	)

# build settings
include_directories(src src/simgear/xml)
add_definitions(-DHAVE_EXPAT_CONFIG_H)

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
	DESTINATION include/jsbsim/math
	)
install(FILES
	src/simgear/compiler.h
	DESTINATION include/jsbsim/simgear
	)
install(FILES
	src/simgear/props/SGSharedPtr.hxx
	src/simgear/props/props.hxx
	src/simgear/props/SGReferenced.hxx
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
	src/models/FGMassBalance.h
	src/models/FGFCS.h
	src/models/FGLGear.h
	src/models/FGInput.h
	src/models/FGAerodynamics.h
	src/models/FGAtmosphere.h
	src/models/FGAuxiliary.h
	src/models/FGBuoyantForces.h
	src/models/FGInertial.h
	src/models/FGPropulsion.h
	src/models/FGOutput.h
	src/models/FGExternalReactions.h
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
	DESTINATION include/jsbsim/models/atmosphere
	)
install(FILES
	src/models/flight_control/FGAccelerometer.h
	src/models/flight_control/FGFCSFunction.h
	src/models/flight_control/FGDeadBand.h
	src/models/flight_control/FGSensorOrientation.h
	src/models/flight_control/FGGyro.h
	src/models/flight_control/FGGain.h
	src/models/flight_control/FGGradient.h
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
    src/FGState.h
	DESTINATION include/jsbsim
    )

# jsbsim library
add_library(jsbsim 
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
	src/models/flight_control/FGGradient.cpp
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
	src/models/FGAtmosphere.cpp
	src/models/FGMassBalance.cpp
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
	src/FGState.cpp
	)
target_link_libraries(jsbsim m)
install(TARGETS jsbsim DESTINATION lib)

# jsbsim executable
add_executable(JSBSim
    src/JSBSim.cpp
	)
target_link_libraries(JSBSim jsbsim)
install(TARGETS JSBSim DESTINATION bin)

# jsbsim gui
# vim:sw=4:ts=4:expandtab
