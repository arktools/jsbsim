# headers
set(MATH_HDRS
    math/FGColumnVector3.h
    math/FGCondition.h
    math/FGFunction.h
    math/FGLocation.h
    math/FGMatrix33.h
    math/FGModelFunctions.h
    math/FGNelderMead.h
    math/FGParameter.h
    math/FGPropertyValue.h
    math/FGQuaternion.h
    math/FGRealValue.h
    math/FGRungeKutta.h
    math/FGStateSpace.h
    math/FGTable.h
    math/LagrangeMultiplier.h
    )
install(FILES ${MATH_HDRS} DESTINATION include/jsbsim/math COMPONENT Dev)

set(SIMGEAR_HDRS
    simgear/compiler.h
    )
install(FILES ${SIMGEAR_HDRS} DESTINATION include/jsbsim/simgear COMPONENT Dev)

set(SIMGEAR_PROPS_HDRS
    simgear/props/props.hxx
    )
install(FILES ${SIMGEAR_PROPS_HDRS} DESTINATION include/jsbsim/simgear/props COMPONENT Dev)

set(SIMGEAR_MAGVAR_HDRS
    simgear/magvar/coremag.hxx
    )
install(FILES ${SIMGEAR_MAGVAR_HDRS} DESTINATION include/jsbsim/simgear/magvar COMPONENT Dev)

set(SIMGEAR_MISC_HDRS
    simgear/misc/stdint.hxx
    )
install(FILES ${SIMGEAR_MISC_HDRS} DESTINATION include/jsbsim/simgear/misc COMPONENT Dev)

set(SIMGEAR_STRUCTURE_HDRS
    simgear/structure/SGReferenced.hxx
    simgear/structure/SGSharedPtr.hxx

    )
install(FILES ${SIMGEAR_STRUCTURE_HDRS} DESTINATION include/jsbsim/simgear/structure COMPONENT Dev)


set(SIMGEAR_XML_HDRS
    simgear/xml/utf8tab.h
    simgear/xml/internal.h
    simgear/xml/ascii.h
    simgear/xml/expat.h
    simgear/xml/nametab.h
    simgear/xml/winconfig.h
    #simgear/xml/xmltok_impl.h
    simgear/xml/easyxml.hxx
    simgear/xml/macconfig.h
    simgear/xml/expat_config.h
    simgear/xml/iasciitab.h
    simgear/xml/xmlrole.h
    simgear/xml/expat_external.h
    simgear/xml/latin1tab.h
    simgear/xml/asciitab.h
    simgear/xml/xmltok.h
    )
install(FILES ${SIMGEAR_XML_HDRS} DESTINATION include/jsbsim/simgear/xml COMPONENT Dev)

set(INPUT_OUTPUT_HDRS
    input_output/FGXMLElement.h
    input_output/FGXMLParse.h
    input_output/FGfdmSocket.h
    input_output/string_utilities.h
    input_output/FGXMLFileRead.h
    input_output/net_fdm.hxx
    input_output/FGScript.h
    input_output/FGGroundCallback.h
    input_output/FGPropertyManager.h
    )
install(FILES ${INPUT_OUTPUT_HDRS} DESTINATION include/jsbsim/input_output COMPONENT Dev)

set(MODELS_HDRS
    models/FGAccelerations.h
    models/FGMassBalance.h
    models/FGFCS.h
    models/FGLGear.h
    models/FGInput.h
    models/FGAerodynamics.h
    models/FGAuxiliary.h
    models/FGBuoyantForces.h
    models/FGInertial.h
    models/FGPropulsion.h
    models/FGOutput.h
    models/FGExternalReactions.h
    models/FGAtmosphere.h
    models/atmosphere/FGMSIS.h
    models/atmosphere/FGMars.h
    models/FGAircraft.h
    models/FGExternalForce.h
    models/FGModel.h
    models/FGGasCell.h
    models/FGPropagate.h
    models/FGGroundReactions.h
    )
install(FILES ${MODELS_HDRS} DESTINATION include/jsbsim/models COMPONENT Dev)

set(MODELS_ATMOSPHERE_HDRS
    models/atmosphere/FGMSIS.h
    models/atmosphere/FGMars.h
    models/atmosphere/FGWinds.h
    models/atmosphere/FGStandardAtmosphere.h
    )
install(FILES ${MODELS_ATMOSPHERE_HDRS} DESTINATION include/jsbsim/models/atmosphere COMPONENT Dev)

set(MODELS_FLIGHT_CONTROL_HDRS
    models/flight_control/FGAccelerometer.h
    models/flight_control/FGFCSFunction.h
    models/flight_control/FGDeadBand.h
    models/flight_control/FGSensorOrientation.h
    models/flight_control/FGGyro.h
    models/flight_control/FGGain.h
    models/flight_control/FGSwitch.h
    models/flight_control/FGFCSComponent.h
    models/flight_control/FGSummer.h
    models/flight_control/FGPID.h
    models/flight_control/FGActuator.h
    models/flight_control/FGSensor.h
    models/flight_control/FGFilter.h
    models/flight_control/FGMagnetometer.h
    models/flight_control/FGKinemat.h
    )
install(FILES ${MODELS_FLIGHT_CONTROL_HDRS} DESTINATION include/jsbsim/models/flight_control COMPONENT Dev)

set(MODELS_PROPULSION_HDRS
    models/propulsion/FGElectric.h
    models/propulsion/FGTurbine.h
    models/propulsion/FGThruster.h
    models/propulsion/FGRocket.h
    models/propulsion/FGNozzle.h
    models/propulsion/FGTurboProp.h
    models/propulsion/FGTransmission.h
    models/propulsion/FGForce.h
    models/propulsion/FGRotor.h
    models/propulsion/FGPiston.h
    models/propulsion/FGEngine.h
    models/propulsion/FGTank.h
    models/propulsion/FGPropeller.h
    )
install(FILES ${MODELS_PROPULSION_HDRS} DESTINATION include/jsbsim/models/propulsion COMPONENT Dev)

set(INITIALIZATION_HDRS
    initialization/FGInitialCondition.h
    initialization/FGSimplexTrim.h
    #initialization/FGTrimAnalysisControl.h
    #initialization/FGTrimAnalysis.h
    initialization/FGTrimAxis.h
    initialization/FGTrim.h
    initialization/FGTrimmer.h
    )
install(FILES ${INITIALIZATION_HDRS} DESTINATION include/jsbsim/initialization COMPONENT Dev)

set(TOP_HDRS
    FGFDMExec.h
    FGJSBBase.h
    )
install(FILES ${TOP_HDRS} DESTINATION include/jsbsim COMPONENT Dev)

# sources
set(JSBSIM_SRCS
    input_output/FGfdmSocket.cpp
    input_output/FGXMLParse.cpp
    input_output/FGScript.cpp
    input_output/FGGroundCallback.cpp
    input_output/FGXMLElement.cpp
    input_output/FGPropertyManager.cpp

    #simgear/xml/xmltok_impl.c
    #simgear/xml/easyxml.cpp
    simgear/xml/xmlrole.c
    simgear/xml/easyxml.cxx
    simgear/xml/xmltok.c
    #simgear/xml/xmltok_ns.c
    simgear/xml/xmlparse.c
    simgear/props/props.cxx
    simgear/magvar/coremag.cxx

    math/FGQuaternion.cpp
    math/FGColumnVector3.cpp
    math/FGStateSpace.cpp
    math/FGPropertyValue.cpp
    math/FGRungeKutta.cpp
    math/FGRealValue.cpp
    math/FGModelFunctions.cpp
    math/FGFunction.cpp
    math/FGNelderMead.cpp
    math/FGTable.cpp
    math/FGLocation.cpp
    math/FGMatrix33.cpp
    math/FGCondition.cpp

    models/FGAccelerations.cpp
    models/flight_control/FGAccelerometer.cpp
    models/flight_control/FGSwitch.cpp
    models/flight_control/FGPID.cpp
    models/flight_control/FGFCSFunction.cpp
    models/flight_control/FGSummer.cpp
    models/flight_control/FGDeadBand.cpp
    models/flight_control/FGGyro.cpp
    models/flight_control/FGActuator.cpp
    models/flight_control/FGFCSComponent.cpp
    models/flight_control/FGMagnetometer.cpp
    models/flight_control/FGFilter.cpp
    models/flight_control/FGGain.cpp
    models/flight_control/FGSensor.cpp
    models/flight_control/FGKinemat.cpp
    models/FGBuoyantForces.cpp
    models/FGExternalForce.cpp
    models/FGLGear.cpp
    models/FGInput.cpp
    models/FGInertial.cpp
    models/FGFCS.cpp
    models/FGPropagate.cpp
    models/FGAuxiliary.cpp
    models/FGPropulsion.cpp
    models/FGOutput.cpp
    models/FGGasCell.cpp
    models/FGMassBalance.cpp
    models/FGAtmosphere.cpp
    models/atmosphere/FGStandardAtmosphere.cpp
    models/atmosphere/FGWinds.cpp
    models/atmosphere/FGMSISData.cpp
    models/atmosphere/FGMSIS.cpp
    models/atmosphere/FGMars.cpp
    models/FGAircraft.cpp
    models/FGExternalReactions.cpp
    models/FGGroundReactions.cpp
    models/propulsion/FGTank.cpp
    models/propulsion/FGForce.cpp
    models/propulsion/FGThruster.cpp
    models/propulsion/FGPropeller.cpp
    models/propulsion/FGTurbine.cpp
    models/propulsion/FGElectric.cpp
    models/propulsion/FGRotor.cpp
    models/propulsion/FGPiston.cpp
    models/propulsion/FGTurboProp.cpp
    models/propulsion/FGTransmission.cpp
    models/propulsion/FGNozzle.cpp
    models/propulsion/FGRocket.cpp
    models/propulsion/FGEngine.cpp
    models/FGAerodynamics.cpp
    models/FGModel.cpp

    initialization/FGInitialCondition.cpp
    initialization/FGSimplexTrim.cpp
    #initialization/FGTrimAnalysisControl.cpp
    #initialization/FGTrimAnalysis.cpp
    initialization/FGTrimAxis.cpp
    initialization/FGTrim.cpp
    initialization/FGTrimmer.cpp

    FGFDMExec.cpp
    FGJSBBase.cpp
    )

# public headers
set(JSBSIM_PRIVATE_HDRS
    )

# private headers
set(JSBSIM_PRIVATE_HDRS
    )

# link libraries
set(JSBSIM_LINK_LIBRARIES
    )
if (MINGW)
    list(APPEND JSBSIM_LINK_LIBRARIES
        wsock32
        ws2_32
        )
endif()

# static library
add_library(jsbsimStatic STATIC ${JSBSIM_SRCS} ${JSBSIM_PUBLIC_HDRS} ${JSBSIM_PRIVATE_HDRS})
set(STATIC_LIBNAME_SUFFIX "")
if (MSVC)
    set(STATIC_LIBNAME_SUFFIX "static")
endif()
set_target_properties(jsbsimStatic PROPERTIES 
    OUTPUT_NAME jsbsim${STATIC_LIBNAME_SUFFIX}
    VERSION ${LIBRARY_VERSION} 
    PUBLIC_HEADER "${JSBSIM_PUBLIC_HDRS}"
    PRIVATE_HEADER "${JSBSIM_PRIVATE_HDRS}"
    )
target_link_libraries(jsbsimStatic ${JSBSIM_LINK_LIBRARIES})
install(TARGETS jsbsimStatic
    ARCHIVE DESTINATION "lib" COMPONENT Dev
    # where to put files on non-mac computers, mac overrides and uses default
    PRIVATE_HEADER DESTINATION "include/jsbsim" COMPONENT Runtime
    PUBLIC_HEADER DESTINATION "include/jsbsim" COMPONENT Runtime
    )

# shared library
if(WITH_BUILD_SHARED)
    add_library(jsbsimShared SHARED ${JSBSIM_SRCS} ${JSBSIM_PUBLIC_HDRS} ${JSBSIM_PRIVATE_HDRS})
    set_target_properties(jsbsimShared PROPERTIES 
        OUTPUT_NAME jsbsim
        VERSION ${LIBRARY_VERSION} 
        SOVERSION ${LIBRARY_SOVERSION}
        FRAMEWORK ON
        PUBLIC_HEADER "${JSBSIM_PUBLIC_HDRS}"
        PRIVATE_HEADER "${JSBSIM_PRIVATE_HDRS}"
        )
    target_link_libraries(jsbsimShared ${JSBSIM_LINK_LIBRARIES})
    install(TARGETS jsbsimShared 
        LIBRARY DESTINATION "lib" COMPONENT Runtime
        FRAMEWORK DESTINATION "/Library/Frameworks" COMPONENT Runtime
        # where to put files on non-mac computers, mac overrides and uses default
        PRIVATE_HEADER DESTINATION "include/jsbsim" COMPONENT Runtime
        PUBLIC_HEADER DESTINATION "include/jsbsim" COMPONENT Runtime
        )
endif()

# command line executable
add_executable(jsbsim-cmd JSBSim.cpp)
target_link_libraries(jsbsim-cmd jsbsimStatic)
install(TARGETS jsbsim-cmd
    RUNTIME DESTINATION "bin" COMPONENT Runtime
    )
# vim:sw=4:ts=4:expandtab
