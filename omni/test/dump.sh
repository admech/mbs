#!/bin/bash

# show currently simulating
if [[ $1 == "--current" || $1 == "-c" ]]
then
  echo `date`;
  echo "Current simulations";
  ps auxww | grep "dump.*Test .*" 

# show existing last time points, file names and sizes
elif [[ $1 == "--peek" || $1 == "-p" ]]
then
  echo `date`;
  echo "Existing results:";
  for f in `ls -R ~/dump/out/**/*.csv`; do ls -laSh $f; tail -1 $f | grep -o "^[^,]\+,"; done;

# copy one file to another keeping the first couple lines, then each nth, then a tail of a few hundred
elif [[ $1 == "--throttle" || $1 == "-t" ]]
then
  if [[ $2 && $3 && $4 ]]
  then
    echo `date`;
    echo "Taking every $2th line and a tail of 500:";
    awk "NR <= 1 || NR % $2 == 0" $3 > $4;
    tail -500 $3 >> $4;
    echo "Done";
    date;
  else
    echo "Please specify n to take each nth line, from and to files as args 2-4";
  fi

# show disk and ram usage
elif [[ $1 == "--diag" || $1 == "-d" ]]
then
  echo "";
  date;
  echo "";
  echo "HDD ==========";
  df -h | grep "nvme\|Use";
  echo "";
  echo "RAM ==========";
  free -m | sed "s/      //" ;
  echo "";
  echo "OMC ==========";
  ps aww --sort=-pcpu -o pid,pcpu,trs,args | grep "[M]bs[A-Z,a-z,0-9,.]\+$\|omc .*" | fold -w `tput cols` 
  echo "";

else
  echo "Usage: --current|-c -- dump current sims, --peek|-p -- show csv sizes and last time point; -t n f1 f2 puts every nth line from f1 to f2; -d shows diag";
  exit -1;
fi

echo "Done";

