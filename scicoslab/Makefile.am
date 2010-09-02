EXTRA_DIST=

scicoslabdir=$(pkgdatadir)/scicoslab
scicoslab_DATA= \
	builder.sce \
	loader.sce
EXTRA_DIST+= $(scicoslab_DATA)

scicoslabdemosdir=$(scicoslabdir)/demos
scicoslabdemos_DATA= \
	demos/loader.sce \
	demos/autopilotBackside.cosf \
	demos/autopilotBacksideOld.cosf \
	demos/autopilotHybrid.cosf \
	demos/invertedPendulum.cosf \
	demos/invertedPendulum.sce \
	demos/jsbsimDynamics.sce \
	demos/jsbsimPorts.cosf \
	demos/loader.sce \
	demos/trimTest.cosf

EXTRA_DIST+= $(scicoslabdemos_DATA)

scicoslabincludesdir=$(scicoslabdir)/includes
scicoslabincludes_DATA= \
	includes/definitions.hpp
EXTRA_DIST+= $(scicoslabincludes_DATA)

scicoslabsci_gatewaydir=$(scicoslabdir)/sci_gateway
scicoslabsci_gateway_DATA= \
	sci_gateway/builder.sce \
	sci_gateway/sci_flightGearComm.cpp
EXTRA_DIST+= $(scicoslabsci_gateway_DATA)

scicoslabscicosdir=$(scicoslabdir)/scicos
scicoslabscicos_DATA= \
	scicos/builder.sce \
	scicos/loader.sce
EXTRA_DIST+= $(scicoslabscicos_DATA)

scicoslabscicosoooarkdir=$(scicoslabscicosdir)/oooark
scicoslabscicosoooark_DATA= \
	scicos/oooark/flightGearComm.sci
EXTRA_DIST+= $(scicoslabscicosoooark_DATA)

scicoslabscriptsdir=$(scicoslabdir)/scripts
scicoslabscripts_DATA= \
	scripts/fgrun.sh
EXTRA_DIST+= $(scicoslabscripts_DATA)
