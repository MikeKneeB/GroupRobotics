#!/bin/bash

COMM="plot"
COMM_EPS="plot"
#echo $COMM
for item in $@
do
  COMM="$COMM '$item' using 'ep':'re' with lines title '$item' noenhanced, '$item' using 'ep':'rol_re' with lines title '$item rolling' noenhanced, "
  COMM_EPS="$COMM_EPS '$item' using 'ep':'epsilon' with lines title '$item' noenhanced, "
done

#echo $COMM

gnuplot -p -e "set grid; set key bmargin; $COMM"
gnuplot -p -e "set grid; set key bmargin; $COMM_EPS"
