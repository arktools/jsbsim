include( ${QT_USE_FILE} )

link_directories(${OPENSCENEGRAPH_LIBRARY_DIRS} ${QT_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS})

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

include_directories(. 
    ${ARKOSG_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${QT_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_BINARY_DIR}
    )

set(JSBSimGui_src
	MainWindow.cpp
	main.cpp
	${JSBSimGui_moc}
	${JSBSimGui_ui}
	${JSBSimGui_res}
	)

add_definitions(-DUSE_QT4)
add_executable(JSBSimGui ${JSBSimGui_src})
add_test(JSBSimGui ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/JSBSimGui)
add_dependencies(JSBSimGui arkosg)
target_link_libraries(JSBSimGui jsbsim ${OPENSCENEGRAPH_LIBRARIES} ${ARKOSG_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS JSBSimGui DESTINATION bin)
