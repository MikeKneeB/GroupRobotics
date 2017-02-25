#!/bin/bash

# Grapher script for gnuplot.
# To use: ensure data file is tab delimited, with headers ep for epochs, re for
# reward, rol_re for rolling reward and epsilon for noise factor. Noise factor
# column can safely be left as zeroes or all blank.

# Ask Mike for help or if things in this file need changing, please don't do
# edits to this script without asking me.

PLOTS=()
COMM_LIST=()

function help() {
  echo "Ask Mike for help..."
}

while [ $# -gt 0 ]
do
  case "$1" in
    -p) PLOTS+=($2); shift;;
    -h) help; break;;
    *) break;;
  esac
  shift
done

#echo "${PLOTS[@]}"

IFS=";"

for plt in "${PLOTS[@]}"
do
  read -r -a AX_LIST <<< "$plt"
  #echo "${AX_LIST[@]}"
  COMM="plot"
  #echo $COMM
  for fname in $@
  do
    for axis in "${AX_LIST[@]}":
    do
      if [ ${axis: -1} == ":" ]
      then
        axis=${axis::-1}
      fi
      COMM="$COMM '$fname' using $axis with lines title '$fname' noenhanced, "
      #echo $COMM
      #COMM_EPS="$COMM_EPS '$item' using 'ep':'epsilon' with lines title '$item' noenhanced, "
    done
  done
  COMM_LIST+=("$COMM")
done

#echo $COMM

for __COMM in "${COMM_LIST[@]}"
do
  #echo $__COMM; echo
  gnuplot -p -e "set grid; set xlabel 'Epoch'; set ylabel 'Reward'; set key bmargin; $__COMM"
done
