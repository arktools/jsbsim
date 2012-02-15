include(${QT_USE_FILE})
add_definitions(-DUSE_QT4)

if (WITH_ARKOSG AND (NOT BUILD_SHARED_LIBRARIES))
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

set(JSBSimGui_src
	MainWindow.cpp
	main.cpp
	${JSBSimGui_moc}
	${JSBSimGui_ui}
	${JSBSimGui_res}
	)

add_executable(JSBSimGui ${JSBSimGui_src})

set(JSBSimGuiLinkLibs
    libjsbsim
    ${QT_LIBRARIES}
    )

if (WITH_ARKOSG)
    list(APPEND JSBSimGuiLinkLibs
        ${ARKOSG_LIBRARIES}
        ${OPENSCENEGRAPH_LIBRARIES} 
        ${OPENGL_LIBRARIES}
        ${Boost_LIBRARIES}
        pthread
        )
    add_dependencies(JSBSimGui arkosg)
endif()

if (WIN32)
    if (NOT BUILD_SHARED_LIBS)
        list(APPEND JSBSimGuiLinkLibs
            lcms
            lzma
        )
    endif()
endif()

add_test(JSBSimGui ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/JSBSimGui)
target_link_libraries(JSBSimGui ${JSBSimGuiLinkLibs})
install(TARGETS JSBSimGui DESTINATION bin)
