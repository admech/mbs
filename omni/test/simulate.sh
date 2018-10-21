#!/bin/bash

if [[ $1 ]]
then
  echo "Going to run $1";
else
  echo "Please pass model name as first arg";
  exit -1;
fi

if [[ $2 ]]
then
  if [[ -d $2 ]] 
  then
    echo "dir $2 already exists, please either remove it or specify another one";
    exit -1;
  else
    echo "Going to put stuff to $2";
  fi
else
  echo "Please pass target dir name within ~/dump/out/ as second arg";
  exit -1;
fi

if [[ $3 ]]
then
  echo "Using parameter overrides: $3";
  parameter_overrides=", override=\"$3\"";
else
  echo "just in case: you could have specified some parameter overrides in the form: paramName=paramValue,other=oneMore as the third argument";
  parameter_overrides="";
fi

tmp_dir_name="/home/ubuntu/dump/out/$2";
tmp_file_name="$tmp_dir_name/simulate_$1.mos";

cp "test.mos" $tmp_file_name;

sed -i "s/SPECIFY_MODEL_NAME/$1/g" $tmp_file_name;
sed -i "s/SPECIFY_PARAMETER_OVERRIDES/$parameter_overrides/g" $tmp_file_name;

echo "Created run script:"
echo "";
echo "$tmp_file_name";
echo "";
echo "----------------------------------------";
echo "----------------------------------------";
echo "            RUNNING MODELICA            ";
echo "----------------------------------------";
echo "----------------------------------------";
date;

cd $tmp_dir_name && \
omc \
    -d=initialization,evaluateAllParameters \
    --indexReductionMethod=dummyDerivatives \
    --numProcs=6 \
    $tmp_file_name \
  | grep -v "Warning.*Connector.*\(KinematicPort\|WrenchPort\) is not balanced" \
  > run.log && \
cd -;

echo "----------------------------------------";
echo "----------------------------------------";
echo "----------------------------------------";
echo "----------------------------------------";
echo "----------------------------------------";
date;
echo "";

echo "Done";

