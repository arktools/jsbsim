#!/bin/bash

PREP_PLOT=./src/utilities/prep_plot

function usage {
    echo usage: $0 plotDirective
}

if [ $# == 1 ]
then
    plotDirective=$1 
elif [ $# == 0 ]
then
    plotDirective=""
else
    echo incorrect usage
    usage
    exit 1
fi

# prepare plots from data
for file in $(ls *.csv | sed s/\.csv//g)
do
    echo $file post processing
    if [ "$plotDirective" = "" ]
    then
        if [ -f data_plot/$file.xml ]
        then
            echo -e "\tusing xml style: $file.xml"
            $PREP_PLOT $file.csv --plot=data_plot/$file.xml | gnuplot
        else
            while [ 1 ]
            do
                echo -e -n "\t no plot directive found or given, perform
                comprehensive plot? [y/n] "
                read answer
                if [ "$answer" = "y" ]
                then
                    echo -e "\tusing comprehensive plot"
                    $PREP_PLOT $file.csv --comp | gnuplot
                    break
                elif [ "$answer" = "n" ]
                then
                    exit 0
                fi
            done
        fi
    else
            echo -e "\tusing plot directive: $plotDirective"
            $PREP_PLOT $file.csv --plot=$plotDirective | gnuplot
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
