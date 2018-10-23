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
else
  echo "Usage: --current|-c -- dump current sims, --peek|-p -- show csv sizes and last time point";
  exit -1;
fi

echo "Done";

