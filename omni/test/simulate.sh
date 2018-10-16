#!/bin/bash

if [[ $1 ]]
then
  echo "Going to run $1";
else
  echo "Please pass model name as first arg";
  exit -1;
fi

tmp_file_name="out/simulate_$1.mos";

cp "test.mos" $tmp_file_name;

sed -i "s/SPECIFY_MODEL_NAME/$1/g" $tmp_file_name;

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

omc \
    -d=initialization,evaluateAllParameters \
    --indexReductionMethod=dummyDerivatives \
    $tmp_file_name \
  | grep -v "Warning.*Connector.*\(KinematicPort\|WrenchPort\) is not balanced" \
  > run.log;

echo "----------------------------------------";
echo "----------------------------------------";
echo "----------------------------------------";
echo "----------------------------------------";
echo "----------------------------------------";
date;
echo "";

echo "Done";

