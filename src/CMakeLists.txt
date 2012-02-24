set(FDGPAK_SRCS
    fdgpak.cpp
    )

set(FDGPAK_PUBLIC_HDRS
    fdgpak.h
    )

set(FDGPAK_PRVIATE_HDRS
    private.h
    )

# set icns file containing icons
if (APPLE)
    # set how it shows up in Info.plist
    set(MACOSX_BUNDLE_CON_FILE fdgpak.icns)
    # set where in the bundle to put the icns file
    set_source_files_properties(fdgpak.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    # include the icns file in the target
    list(APPEND FDGPAK_SRCS fdgpak.icns)
endif()

# static library example
add_library(fdgpak_static STATIC ${FDGPAK_SRCS} ${FDGPAK_PUBLIC_HDRS} ${FDGPAK_PRIVATE_HDRS})

# dynamic library example
add_library(fdgpak_dynamic SHARED ${FDGPAK_SRCS} ${FDGPAK_PUBLIC_HDRS} ${FDGPAK_PRIVATE_HDRS})

# these properties tell mac how to structure the framework
set_property(TARGET fdgpak_dynamic PROPERTY FRAMEWORK ON)
set_property(TARGET fdgpak_dynamic PROPERTY PUBLIC_HEADER ${FDGPAK_HDRS})
set_property(TARGET fdgpak_dynamic PROPERTY PRIVATE_HEADER ${FDGPAK_HDRS})

# executable linking to static library
add_executable(fdgpak MACOSX_BUNDLE WIN32 main.cpp)
target_link_libraries(fdgpak fdgpak_static)

install(TARGETS fdgpak fdgpak_dynamic fdgpak_static
    RUNTIME DESTINATION "\$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/bin" COMPONENT Runtime
    LIBRARY DESTINATION "\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib" COMPONENT Runtime
    ARCHIVE DESTINATION "\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib" COMPONENT Runtime
    BUNDLE DESTINATION  "\$ENV{DESTDIR}/Applications" COMPONENT Runtime
    FRAMEWORK DESTINATION "\$ENV{DESTDIR}/Library/Frameworks" COMPONENT Runtime
    # where to put files on non-mac computers, mac override and uses default
    PRIVATE_HEADER DESTINATION "\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/include/fdgpak" COMPONENT Runtime
    PUBLIC_HEADER DESTINATION "\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/include/fdgpak" COMPONENT Runtime
    RESOURCE DESTINATION "\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/fdgpak" COMPONENT Runtime
    )
