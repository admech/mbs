#!/bin/bash

# HDD and RAM
if [[ $1 == "--resources" || $1 == "-r" ]]
then
  watch -n 600 "./dump.sh -d"

elif [[ $1 == "--simulation" || $1 == "-s" ]]
then
  watch -n 1 "date && ./dump.sh -p | grep -o \"[a-z,A-Z,0-9]\+_res.*\|^[0-9].*\|[0-9][^ ]\+ Oct\""

else
  echo "Usage: --resources|-r -- watch HDD and RAM; --simulation|-s -- current simulation time of existing out files";
  exit -1;
fi

echo "Done";

