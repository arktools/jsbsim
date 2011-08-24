EXTRA_DIST = datafile.cpp datafile.h plotXMLVisitor.cpp plotXMLVisitor.h main.cpp prep_plot.cpp post_process.sh

SUBDIRS = aeromatic

noinst_PROGRAMS = prep_plot

prep_plot_CPPFLAGS = -I ${top_srcdir}/src/input_output
prep_plot_SOURCES = prep_plot.cpp ../simgear/xml/easyxml.cxx plotXMLVisitor.cpp
prep_plot_LDADD = -lexpat
