#!/bin/bash

COMM="plot"
echo $COMM
for item in $@
do
  COMM="$COMM '$item' using 'ep':'re' with lines title '$item', "
done

echo $COMM

gnuplot -p -e "set grid; $COMM"
