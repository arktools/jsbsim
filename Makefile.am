ACLOCAL_AMFLAGS = -I m4

SUBDIRS	= src gui systems aircraft engine scripts data_output data_plot check_cases

dist-hook:
	(cd $(top_srcdir))

#
# Rule to build RPM distribution package
#
rpm: dist
	rpm -ta $(PACKAGE)-$(VERSION).tar.gz

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = jsbsim.pc

EXTRA_DIST = \
			 autogen.sh JSBSim.xsd JSBSim.xsl JSBSim.vcproj \
			 JSBSim.dox updateAutomake.sh jsbsim.pc.in


