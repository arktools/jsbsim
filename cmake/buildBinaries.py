#!/usr/bin/python
# Builds jsbsim binaries for Linux, Mac OS X, and Windows (via mingw)
# Lenna X. Peterson

import os # for os stuff
import sys # for determining os
import inspect # for finding current directory
import subprocess # for calling processes

# apparently __file__ doesn't work consistently on windows
#print os.path.dirname(os.path.abspath(__file__))
scriptDir = os.path.abspath(os.path.split(inspect.getfile(inspect.currentframe()))[0])
#subDir = os.path.join(scriptDir, "arkcmake")
#if os.path.exists(subDir):
    #sys.path.append(subDir)

#try:
    #from get_build_path import get_build_path
#except ImportError:
    #print "Could not find 'get_build_path.py'"
    #print "This module is required."
    #raise SystemExit

#build_path = get_build_path()
#if build_path:
    #os.chdir(build_path)
#else:
    #print "The script was unable to find a build directory."
    #raise SystemExit

buildDir = os.path.normpath(os.path.join(scriptDir, "../build"))
print buildDir
#print os.uname()[0]
if sys.platform.startswith('darwin'):
    os.system("rm -f CMakeCache.txt")
    #os.system("cmake -DWITH_GUI=y -DWITH_OSXBUNDLE=n ..; make; cpack")
    #os.system("rm -f CMakeCache.txt")
    #os.system("cmake -DWITH_GUI=y -DWITH_OSXBUNDLE=y ..; make; cpack")
elif sys.platform.startswith('linux'):
    pass
else:
    print "Nothing to do on %s" % sys.platform
    raise SystemExit
