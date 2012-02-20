#!/bin/bash
# APPLE ONLY # 
# This is intended to be CPACK_BUNDLE_STARTUP_COMMAND
# (copied to $BUNDLE/Contents/MacOS/ and `exec`s the binary)
# `mdls $FILE` lists all metadata searchable by mdfind

# Finds app bundle
#BUNDLE=`mdfind "kMDItemKind == Application && kMDItemFSName == jsbsim.app" | head -1`
# CMake's method for finding this path (requires script to be run from $BUNDLE/Contents/MacOS/)
BUNDLE=`echo "$0" | sed -e 's:/Contents/MacOS/.*::'`
#echo $BUNDLE
f16xml=`mdfind -onlyin $BUNDLE "kMDItemFSName == f16.xml" | sed -e 's:share/jsbsim/aircraft/f16/f16.xml::'`
# Outputs absolute path to location of bin/ share/ etc. 
#echo $f16xml
exec `$f16xml/bin/JSBSimGui`
