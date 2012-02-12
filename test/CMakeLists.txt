include(${QT_USE_FILE})
add_definitions(-DUSE_QT4)

set(arkosgTest_moc_src
	MainWindow.hpp
	)

set(arkosgTest_ui_src
	MainWindow.ui
	)

set(arkosgTest_res
	)

qt4_wrap_cpp(arkosgTest_moc ${arkosgTest_moc_src})
qt4_wrap_ui(arkosgTest_ui ${arkosgTest_ui_src})
qt4_add_resources(arkosgTest_res ${arkosgTest_res_src})

set(arkosgTest_src
	MainWindow.cpp
	main.cpp
	${arkosgTest_moc}
	${arkosgTest_ui}
	${arkosgTest_res}
	)

include_directories(
    ${CMAKE_BINARY_DIR}
    ${QT_INCLUDES}
    ${ARKOSG_INCLUDE_DIRS}
    )

set(arkosgTestLinkLibs
    ${ARKOSG_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES} 
    ${QT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    pthread
    )

if (WIN32)
    if (NOT BUILD_SHARED_LIBS)
        list(APPEND arkosgTestLinkLibs
            lcms
            lzma
        )
    endif()
endif()

add_executable(arkosgTest ${arkosgTest_src})
add_test(arkosgTest ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/arkosgTest)
target_link_libraries(arkosgTest ${arkosgTestLinkLibs})

install(TARGETS arkosgTest DESTINATION bin)
