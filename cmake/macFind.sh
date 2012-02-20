#!/bin/bash
jsbsimTop=`mdfind "kMDItemKind == Application && kMDItemFSName == jsbsim.app" | head -1`
echo $jsbsimTop
f16xml=`mdfind -onlyin $jsbsimTop "kMDItemFSName == f16.xml"`
echo $f16xml
