include(${QT_USE_FILE})
add_definitions(-DUSE_QT4)

if (WITH_ARKOSG AND BUILD_STATIC)
    add_definitions(-DOSG_LIBRARY_STATIC)
endif()

link_directories(
    ${OPENSCENEGRAPH_LIBRARY_DIRS}
    ${QT_LIBRARY_DIRS}
    )

include_directories(. 
    ${ARKOSG_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
	${QT_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    )

set(JSBSimGui_moc_src
	MainWindow.hpp
	)

set(JSBSimGui_ui_src
    )

if (WITH_ARKOSG)
    list(APPEND JSBSimGui_ui_src
        MainWindow_OSG.ui
        )
else()
    list(APPEND JSBSimGui_ui_src
        MainWindow.ui
        )
endif()

set(JSBSimGui_res
    textures.qrc
	)

qt4_wrap_cpp(JSBSimGui_moc ${JSBSimGui_moc_src})
qt4_wrap_ui(JSBSimGui_ui ${JSBSimGui_ui_src})
qt4_add_resources(JSBSimGui_res ${JSBSimGui_res_src})

set(JSBSIMGUI_SRCS
	MainWindow.cpp
	main.cpp
	${JSBSimGui_moc}
	${JSBSimGui_ui}
	${JSBSimGui_res}
	)

if (WITH_ARKOSG)
    list(APPEND JSBSimGuiLinkLibs
        arkosg
        )
    add_dependencies(jsbsim-gui arkosg)
endif()

if (WIN32)
    if (WITH_BUILD_STATIC)
        list(APPEND JSBSimGuiLinkLibs
            lcms
            lzma
        )
    endif()
endif()

set(JSBSimGuiLinkLibs
    jsbsim
    ${QT_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    )

# set icns file containing icons
if (APPLE)
    # set how it shows up in Info.plist
    set(MACOSX_BUNDLE_CON_FILE jsbsim.icns)
    # set where in the bundle to put the icns file
    set_source_files_properties(jsbsim.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    # include the icns file in the target
    list(APPEND JSBSIMGUI_SRCS jsbsim.icns)
endif(APPLE)

add_executable(jsbsim-gui MACOSX_BUNDLE WIN32 ${JSBSIMGUI_SRCS})
target_link_libraries(jsbsim-gui ${JSBSimGuiLinkLibs})
list(APPEND INSTALLED_TARGETS jsbsim-gui)

# install
#install(TARGETS jsbsim-gui 
#    RUNTIME DESTINATION "bin" COMPONENT Runtime
#    LIBRARY DESTINATION "lib" COMPONENT Runtime
#    ARCHIVE DESTINATION "lib" COMPONENT Runtime
#    BUNDLE DESTINATION "/Applications" COMPONENT Runtime
#    FRAMEWORK DESTINATION "/Library/Frameworks" COMPONENT Runtime
#    # where to put files on non-mac computers, mac override and uses default
#    PRIVATE_HEADER DESTINATION "include/jsbsim" COMPONENT Runtime
#    PUBLIC_HEADER DESTINATION "include/jsbsim" COMPONENT Runtime
#    RESOURCE DESTINATION "share/jsbsim" COMPONENT Runtime
#    )
