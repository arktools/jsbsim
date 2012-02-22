#!/usr/bin/python
# Builds jsbsim binaries for Linux, Mac OS X, and Windows (via mingw)
# Lenna X. Peterson

import os # for os stuff
import sys # for determining os
import inspect # for finding current directory
import subprocess # for calling processes

# apparently __file__ doesn't work consistently on windows
scriptDir = os.path.abspath(os.path.split(inspect.getfile(inspect.currentframe()))[0])

buildDir = os.path.normpath(os.path.join(scriptDir, "../build"))
#print buildDir
if not os.path.exists(buildDir):
    print "Creating build directory"
    os.mkdir(buildDir)
try:
    os.chdir(buildDir)
except OSError:
    print "Could not move to build directory."
    print "This script is probably in the wrong directory."
    raise SystemExit

cmakeArgs = [".."] 

def callCMake(*args):
    _cmakeCall = list(cmakeArgs)
    # Sets DWITH_GUI default to y
    if "-DWITH_GUI=n" not in args:
        _cmakeCall.append("-DWITH_GUI=y")
    for a in args:
        _cmakeCall.append(a)
    _cmakeCall.insert(0, "cmake")
    #print _cmakeCall
    subprocess.call("rm -f CMakeCache.txt", shell=True)
    subprocess.call(_cmakeCall) 
    subprocess.call(["make"]) 

def callCPack(*args):
    _cpackCall = ["cpack"]
    for a in args:
        _cpackCall.append(a)
    print _cpackCall
    subprocess.call(_cpackCall)
 
if sys.platform.startswith('darwin'):
    print "Building for Darwin"
    # WITH_OSXBUNDLE defaults to ON
    callCMake()
    callCPack()
    print "OS X app bundle built"
    callCMake("-DWITH_OSXBUNDLE=n")
    callCPack()
    print "OS X pkg installer built" 
elif sys.platform.startswith('linux'):
    print "Building for Linux"
    callCMake()
    callCPack()
    print "Linux Debian package built"
    callCPack("-G","STGZ")

    print "Generic Linux package built"
else:
    print "Nothing to do on %s" % sys.platform
    raise SystemExit
