#!/usr/bin/python

import subprocess # for check_call()

installed = [
	"/usr/local/include/JSBSim",
	"/usr/local/include/arkcomm",
	"/usr/local/include/arkmath",
	"/usr/local/include/arkosg",
	"/usr/local/include/arkscicos",
	"/usr/local/lib/arkcomm",
	"/usr/local/lib/arkmath",
	"/usr/local/lib/arkosg",
	"/usr/local/lib/libJSBSim.a",
	"/usr/local/lib/libJSBSim.la",
	"/usr/local/lib/libarkcomm.a",
	"/usr/local/lib/libarkmath.a",
	"/usr/local/lib/libarkosg.a",
	"/usr/local/lib/libarkscicos.0.0.0.so",
	"/usr/local/lib/libarkscicos.so",
	"/usr/local/lib/pkgconfig/arkcomm.pc",
	"/usr/local/lib/pkgconfig/arkmath.pc",
	"/usr/local/lib/pkgconfig/arkosg.pc",
	"/usr/local/lib/pkgconfig/arkscicos.pc",
	"/usr/local/lib/pkgconfig/jsbsim.pc",
	"/usr/local/share/arkmath",
	"/usr/local/share/arkosg",
	"/usr/local/share/jsbsim",
	"/Applications/ScicosLabGtk.app/Contents/Resources/scicoslab-gtk-4.4.1/contrib/arkscicos-toolbox",
]

print "Removing arkscicos..."
for path in installed:
	subprocess.check_call(["sudo", "rm", "-rf", path])
	print "Removed '%s'" % path
print "Finished removing arkscicos."
exit(0)
