#!/bin/bash
if [ $# != 3 ]
then
	echo program for communicating with jsbsimcomm
	echo usage: $0 ip-address number aircraft
	fgfs --show-aircraft
	exit
fi

ip=$1
number=$2
aircraft=$3

function fgRunExtPlane()
{
num=$1
callsign=hsl-$num
portMulti=$(($num+5000))
portFdm=$(($num+6000))
echo callsign: $callsign
echo port multi: $portMulti
echo port fdm: $portFdm
fgfs \
--fdm=external \
--callsign=$callsign \
--aircraft=$aircraft \
--multiplay=in,10,$ip,$portMulti \
--multiplay=out,10,mpserver05.flightgear.org,5000 \
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
