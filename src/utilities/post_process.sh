#!/bin/bash

PREP_PLOT=./src/utilities/prep_plot

# prepare plots from data
for file in $(ls *.csv | sed s/\.csv//g)
do
    echo $file post processing
    if [ -f data_plot/$file.xml ]
    then
        echo -e "\tusing xml style: $file.xml"
        $PREP_PLOT $file.csv --plot=data_plot/$file.xml | gnuplot
    else
        echo -e "\tusing comprehensive plot"
        $PREP_PLOT $file.csv --comp | gnuplot
    fi
done

# convert ps to pdf
for file in $(ls *.ps | sed s/\.ps//g)
do
    echo -e "\tconverting to pdf: $file"
    gs -q -dNOPAUSE -dBATCH -sDEVICE=pdfwrite -sOutputFile=$file.pdf $file.ps
    rm $file.ps
done
# vim:ts=4:sw=4:expandtab
