if BUILD_GUI

include $(top_srcdir)/autotroll.mk

# initiliaze to null
bin_PROGRAMS = 
BUILT_SOURCES =

bin_PROGRAMS += trim
trim_SOURCES = main.cpp MainWindow.cpp MainWindow.hpp MainWindow.moc.cpp MainWindow.ui.h \
			   osgUtils.cpp osgUtils.hpp Viewer.cpp Viewer.hpp QOSGAdapterWidget.cpp \
			   QOSGAdapterWidget.hpp $(top_srcdir)/src/math/FGNelderMead.cpp \
			   ${top_srcdir}/src/FGFDMExec.cpp ${top_srcdir}/src/FGJSBBase.cpp \
			   ${top_srcdir}/src/FGFDMExec.h ${top_srcdir}/src/FGJSBBase.h
trim_CXXFLAGS = $(QT_CXXFLAGS) $(AM_CXXFLAGS)
trim_CPPFLAGS = $(QT_CPPFLAGS) $(AM_CPPFLAGS) $(OSG_CFLAGS) -DUSE_QT4 -DDATADIR=\"$(abs_top_srcdir)\" -I$(top_srcdir)/src
trim_LDFLAGS  = $(QT_LDFLAGS) $(LDFLAGS)
trim_LDADD    = $(QT_LIBS) $(LDADD) $(OSG_LIBS) \
	$(top_srcdir)/src/initialization/libInit.a \
	$(top_srcdir)/src/models/atmosphere/libAtmosphere.a \
	$(top_srcdir)/src/models/libModels.a \
	$(top_srcdir)/src/models/flight_control/libFlightControl.a \
	$(top_srcdir)/src/models/propulsion/libPropulsion.a \
	$(top_srcdir)/src/input_output/libInputOutput.a \
	$(top_srcdir)/src/math/libMath.a \
	$(top_srcdir)/src/simgear/props/libProperties.a \
	$(top_srcdir)/src/simgear/xml/libExpat.a \
	$(top_srcdir)/src/simgear/magvar/libcoremag.a

BUILT_SOURCES += MainWindow.moc.cpp MainWindow.ui.h

endif
