#!/bin/bash

COMM="plot"
echo $COMM
for item in $@
do
  COMM="$COMM '$item' using 'ep':'re' with lines title '$item', '$item' using 'ep':'rol_re' with lines title '$item rolling', "
done

echo $COMM

gnuplot -p -e "set grid; $COMM"
