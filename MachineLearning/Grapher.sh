#!/bin/bash

# Grapher script for gnuplot.
# To use:
# - ensure data file is tab delimited.
# - Input columns to plot with switch -p followed by the columns to plot in
#   form: "'x1':'y1';'x2':'y2';...;'xn':'yn'". All of these columns will be
#   plotted on one graph. Multiple -p switches (followed by column lists) can
#   be passed to the script.
# - After one or more -p column lists have been input one or more filenames
#   should be passed to the script.
#
# Ask Mike for help or if things in this file need changing, please don't
# edit this script without asking me.

PLOTS=()
LABELS=()
COMM_LIST=()

function help() {
  echo "Ask Mike for help..."
}

while [ $# -gt 0 ]
do
  case "$1" in
    -p) PLOTS+=($2); LABELS+=($3); shift; shift;;
    -h) help; break;;
    *) break;;
  esac
  shift
done

#echo "${PLOTS[@]}"
#echo "${LABELS[@]}"

IFS=";"

i=0
#for plt in "${PLOTS[@]}"
while [ $i -lt ${#PLOTS[*]} ]
do
  read -r -a AX_LIST <<< "${PLOTS[$i]}"
  read -r -a LA_LIST <<< "${LABELS[$i]}"
  #echo "${AX_LIST[@]}"
  COMM="set xlabel ${LA_LIST[0]}; set ylabel ${LA_LIST[1]}; plot"
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
  i=$(( $i + 1));
done

#echo $COMM

for __COMM in "${COMM_LIST[@]}"
do
  #echo $__COMM; echo
  gnuplot -p -e "set grid; set key bmargin; $__COMM"
done
