include $(top_srcdir)/autotroll.mk

# initiliaze to null
bin_PROGRAMS = 
BUILT_SOURCES =

bin_PROGRAMS += trim
trim_SOURCES = main.cpp MainWindow.cpp MainWindow.hpp MainWindow.ui MainWindow.ui.h \
			   osgUtils.cpp osgUtils.hpp Viewer.cpp Viewer.hpp QOSGAdapterWidget.cpp \
			   QOSGAdapterWidget.hpp \
			   $(top_srcdir)/src/math/FGNelderMead.cpp
trim_CXXFLAGS = $(QT_CXXFLAGS) $(AM_CXXFLAGS)
trim_CPPFLAGS = $(QT_CPPFLAGS) $(AM_CPPFLAGS) -DUSE_QT4 -DDATADIR=\"$(top_srcdir)/data\" -I$(top_srcdir)/src
trim_LDFLAGS  = $(QT_LDFLAGS) $(LDFLAGS)
trim_LDADD    = $(QT_LIBS) $(LDADD)
BUILT_SOURCES += MainWindow.moc.cpp MainWindow.ui.h
