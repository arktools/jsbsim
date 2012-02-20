#!/bin/bash
# APPLE ONLY # 
# This is intended to be CPACK_BUNDLE_STARTUP_COMMAND
# (copied to $BUNDLE/Contents/Resources/MacOS/ and `exec`s the binary)
# `mdls $FILE` lists all metadata searchable by mdfind
jsbsimTop=`mdfind "kMDItemKind == Application && kMDItemFSName == jsbsim.app" | head -1`
echo $jsbsimTop
f16xml=`mdfind -onlyin $jsbsimTop "kMDItemFSName == f16.xml"`
echo $f16xml
