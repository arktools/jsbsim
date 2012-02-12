include(${QT_USE_FILE})
add_definitions(-DUSE_QT4)

if (WIN32)
    # osg static build only works on 3.0.0, 
    # but stable linux is running 2.8
    add_definitions(-DOSG_LIBRARY_STATIC)
endif()

link_directories(
    ${OPENSCENEGRAPH_LIBRARY_DIRS}
    ${QT_LIBRARY_DIRS}
	#${Boost_LIBRARY_DIRS})
    )

include_directories(. 
    ${ARKOSG_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
	${QT_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_BINARY_DIR}
    )

set(JSBSimGui_moc_src
	MainWindow.hpp
	)

set(JSBSimGui_ui_src
	MainWindow.ui
	)

set(JSBSimGui_res
    textures.qrc
	)

qt4_wrap_cpp(JSBSimGui_moc ${JSBSimGui_moc_src})
qt4_wrap_ui(JSBSimGui_ui ${JSBSimGui_ui_src})
qt4_add_resources(JSBSimGui_res ${JSBSimGui_res_src})

set(JSBSimGui_src
	MainWindow.cpp
	main.cpp
	${JSBSimGui_moc}
	${JSBSimGui_ui}
	${JSBSimGui_res}
	)

set(JSBSimGuiLinkLibs
    libjsbsim
    ${ARKOSG_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES} 
    ${OPENGL_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
    pthread
    )

if (WIN32)
    if (NOT BUILD_SHARED_LIBS)
        list(APPEND JSBSimGuiLinkLibs
            lcms
            lzma
        )
    endif()
endif()

add_executable(JSBSimGui ${JSBSimGui_src})
add_test(JSBSimGui ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/JSBSimGui)
add_dependencies(JSBSimGui arkosg)
target_link_libraries(JSBSimGui ${JSBSimGuiLinkLibs})
install(TARGETS JSBSimGui DESTINATION bin)
