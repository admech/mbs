#!/bin/bash

if [[ $1 == "--current" || $1 == "-c" ]]
then
  echo `date`;
  echo "Current simulations";
  ps auxww | grep "dump.*Test .*" 
elif [[ $1 == "--peek" || $1 == "-p" ]]
then
  echo `date`;
  echo "Existing results:";
  for f in `ls -R ~/dump/out/**/*.csv`; do ls -laSh $f; tail -1 $f | grep -o "^[^,]\+,"; done;
elif [[ $1 == "--throttle" || $1 == "-t" ]]
then
  if [[ $2 && $3 && $4 ]]
  then
    echo `date`;
    echo "Taking every $2th line:";
    awk "NR <= 1 || NR % $2 == 0" $3 > $4;
  else
    echo "Please specify n to take each nth line, from and to files as args 2-4";
  fi
elif [[ $1 == "--diag" || $1 == "-d" ]]
then
  date;
  echo "HDD ==========";
  df -h | grep "nvme\|Use";
  echo "RAM ==========";
  free -m;
else
  echo "Usage: --current|-c -- dump current sims, --peek|-p -- show csv sizes and last time point; -t n f1 f2 puts every nth line from f1 to f2; -d shows diag";
  exit -1;
fi

echo "Done";

