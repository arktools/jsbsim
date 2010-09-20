#!/bin/bash
#--native-ctrls=socket,out,119,,5501,udp \
#--native-ctrls=socket,in,120,,5502,udp \
if [[ $# != 2 ]]
then
	echo usage: $0 ip-address num
	exit
fi
ip=$1
num=$2
port=$((5000+$num))
callsign=f-$num
echo callsign: $callsign
echo port: $port
portFdm=5500+$num
#--fdm=external \
#--native-fdm=socket,in,120,,portFdm,udp \
fgfs \
--callsign=$callsign \
--aircraft=T38 \
--geometry=400x300 \
--multiplay=in,10,$ip,$port \
--multiplay=out,10,mpserver05.flightgear.org,5000 \
--airport=KSFO \
--runway=28R
#--shading-flat \
#--fog-disable \
#--disable-textures \
#--disable-specular-highlight \
#--disable-skyblend \
#--disable-random-objects \
#--disable-panel \
#--disable-horizon-effect \
#--disable-clouds \
#--disable-anti-alias-hud
