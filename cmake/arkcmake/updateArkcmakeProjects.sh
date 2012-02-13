#!/bin/bash
dirList="
arkcomm
arkmath
arkosg
arkscicos
arkhangar
"
topDir=$PWD
for dir in $dirList
do
    echo $dir
    cd $topDir/$dir/cmake && ./arkcmake/updateArkcmake.py
    cd $topDir/$dir && git add . && git pull origin master && git commit -m "updated arkcmake" && git push origin master
    cd $topDir
done
