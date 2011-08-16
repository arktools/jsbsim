#!/bin/bash

# prepare plots from data
for file in $(ls *.csv)
do
    echo $file 
    if [ -f data_plot/$file:r.xml ]
    then
        echo preparing plot for: $file
        prep_plot $file --plot=data_plot/$file:r.xml | gnuplot
    fi
done

# convert ps to pdf
for file in $(ls *.ps)
do
    gs -q -dNOPAUSE -dBATCH -sDEVICE=pdfwrite -sOutputFile=$file.pdf $file
done
# vim:ts=4:sw=4:expandtab
