#!/bin/bash

if [[ $# == 0 || $# > 5 ]]; then
	echo program for communicating with jsbsimcomm
	echo usage: $0 aircraft ip-address mpPortStart mpServer number
	fgfs --show-aircraft
	exit
fi

aircraft=c172p
ip=192.168.1.1
mpPortStart=5000
mpServer=mpserver05.flightgear.org	
number=1

echo num args: $#
if [[ $# > 0 ]]; then aircraft=$1; fi
if [[ $# > 1 ]]; then ip=$2; fi
if [[ $# > 2 ]]; then mpPortStart=$3; fi
if [[ $# > 3 ]]; then mpServer=$4; fi
if [[ $# > 4 ]]; then number=$5; fi

echo input
echo aircraft: $aircraft
echo ip: $ip
echo mpPortStart: $mpPortStart
echo mpServer: $mpServer
echo number: $number

function fgRunExtPlane()
{
num=$1
callsign=hsl-$num
portMulti=$(($num+$mpPortStart))
portFdm=$(($num+6000))
echo num: $num
echo callsign: $callsign
echo port multi: $portMulti
echo port fdm: $portFdm
fgfs \
--fdm=external \
--callsign=$callsign \
--aircraft=$aircraft \
--multiplay=in,10,$ip,$portMulti \
--multiplay=out,10,$mpServer,5000 \
--native-fdm=socket,in,10,,$portFdm,udp \
--prop:/sim/frame-rate-throttle-hz=10 \
--timeofday=noon \
--disable-clouds3d \
--geometry=400x300 \
--vc=30 \
--altitude=1000 \
--heading=90 \
--roll=0 \
--pitch=0 \
--wind=0@0 \
--turbulence=0.0 \
--timeofday=noon \
--shading-flat \
--notrim \
--fog-fastest \
--enable-ai-models \
--disable-specular-highlight \
--disable-skyblend \
--disable-random-objects \
--disable-panel \
--disable-horizon-effect \
--disable-clouds \
--disable-anti-alias-hud \

}
for ((num=1; num<=$number; num++))
do
	fgRunExtPlane $num &
done
