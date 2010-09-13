EXTRA_DIST=

if BUILD_SCICOSLAB
sci_gateway/libjsbsim_sci_gateway.so: $(top_builddir)/src/libJSBSim.la sci_gateway/*.cpp
	find . -regex ".*\.\(o\|lo\|a\|la\|so\)" -exec rm -f {} \;
	scicoslab -args -nouserstartup -nwni -f autobuild.sce
clean-scicos:
	find . -regex ".*\.\(o\|lo\|a\|la\|so\)" -exec rm -f {} \;
clean-local: clean-scicos
all-local: sci_gateway/libjsbsim_sci_gateway.so
endif


scicoslabdir=$(pkgdatadir)/scicoslab
scicoslab_DATA= \
	builder.sce \
	loader.sce
EXTRA_DIST+= $(scicoslab_DATA)

scicoslabdemosdir=$(scicoslabdir)/demos
scicoslabdemos_DATA= \
	demos/loader.sce \
	demos/EasyStarClassic.cos \
	demos/EasyStar_lin.sce

EXTRA_DIST+= $(scicoslabdemos_DATA)

scicoslabincludesdir=$(scicoslabdir)/includes
scicoslabincludes_DATA= \
	includes/definitions.hpp
EXTRA_DIST+= $(scicoslabincludes_DATA)

scicoslabsci_gatewaydir=$(scicoslabdir)/sci_gateway
scicoslabsci_gateway_DATA= \
	sci_gateway/builder.sce \
	sci_gateway/loader.sce \
	sci_gateway/sci_jsbsimComm.cpp \
	sci_gateway/sci_jsbsimTrim.cpp
EXTRA_DIST+= $(scicoslabsci_gateway_DATA)

scicoslabscicosdir=$(scicoslabdir)/scicos
scicoslabscicos_DATA= \
	builder.sce \
	loader.sce
EXTRA_DIST+= $(scicoslabscicos_DATA)

scicoslabscicosjsbsimdir=$(scicoslabscicosdir)/jsbsim
scicoslabscicosjsbsim_DATA= \
	scicos/jsbsim/jsbsimComm.sci
EXTRA_DIST+= $(scicoslabscicosoooark_DATA)

scicoslabscriptsdir=$(scicoslabdir)/scripts
scicoslabscripts_DATA= \
	scripts/fgrun.sh
EXTRA_DIST+= $(scicoslabscripts_DATA)

.PHONY: clean-scicos
