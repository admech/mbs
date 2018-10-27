#!/bin/bash

if [[ $1 ]]
then
  model_name="$1";
  echo "Going to run $model_name";
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

if [[ $3 == "--dry" ]]
then
  is_dry_run="true";
else
  is_dry_run="false";
fi

tmp_dir_name="/home/ubuntu/dump/out/$2";
tmp_file_name="$tmp_dir_name/simulate_$model_name.mos";

cp "test.mos" $tmp_file_name;

sed -i "s/SPECIFY_MODEL_NAME/$model_name/g" $tmp_file_name;

sed -i "s/SPECIFY_DRY_RUN/$is_dry_run/g" $tmp_file_name;

echo "";
echo "";
echo "VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV";
echo "--------------------------------";
echo "--------------------------------";
echo "";
echo "Created run script:"
echo "";
echo "$tmp_file_name";
echo "";
echo "--------------------------------";
echo "--------------------------------";
echo "        RUNNING MODELICA        ";
echo "--------------------------------";
echo "--------------------------------";
echo "";
echo "";
echo "Started simulating";
echo "";
echo "$model_name"; 
echo "";
echo "at";
date;
echo "";
echo "Please find results in";
echo "";
echo "$tmp_dir_name";
echo "";
echo "";
echo "--------------------------------";

# --indexReductionMethod=dummyDerivatives \
cd $tmp_dir_name && \
omc \
    -d=initialization,evaluateAllParameters \
    $tmp_file_name \
    --maxSizeLinearTearing=5000 \
    --maxSizeNonlinearTearing=5000 \
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

