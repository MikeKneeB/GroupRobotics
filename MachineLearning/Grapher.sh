#!/bin/bash

# Grapher script for gnuplot.
# To use ensure data file is tab delimited, with headers ep for epochs, re for
# reward, rol_re for rolling reward and epsilon for noise factor. Noise factor
# column can safely be left as zeroes or all blank.

# Ask Mike for help or if things in this file need changing, please don't do
# edits to this script without asking me.

COMM="plot"
COMM_EPS="plot"
#echo $COMM
for item in $@
do
  COMM="$COMM '$item' using 'ep':'re' with lines title '$item' noenhanced, '$item' using 'ep':'rol_re' with lines title '$item rolling' noenhanced, "
  COMM_EPS="$COMM_EPS '$item' using 'ep':'epsilon' with lines title '$item' noenhanced, "
done

#echo $COMM

gnuplot -p -e "set grid; set xlabel 'Epoch'; set ylabel 'Reward'; set key bmargin; $COMM"
gnuplot -p -e "set grid; set key bmargin; $COMM_EPS"
