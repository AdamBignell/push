#!/bin/bash

cd
cd Documents/USRA/push

BASE="./push -r 100 -b 500 -z 0.5 -s 0.5 -t C -y H -g 50"

# Produce simple square results
CIRCLE="circle_tightrad"
cd Results_Replays
mkdir $CIRCLE
cd ..
cd Results
mkdir $CIRCLE
cd ..
for i in {0..19}
do
	$BASE -c 0 -d 0 -o $CIRCLE/$CIRCLE$i -x
done

# Produce all square results
squaresarray=( "square" "squareflare_preprime" "squaredrag" )
QUALIFIER="_tightrad"
for i in "${squaresarray[@]}"
do
	NAME=$i$QUALIFIER
	ADDITIONAL="-c 0 "
	cd Results_Replays
	mkdir $NAME
	cd ..
	cd Results
	mkdir $NAME
	cd ..
	if [ $i == "square" ]
	then
		ADDITIONAL+="-d 0"
	fi
	if [ $i == "squareflare_preprime" ]
	then
		ADDITIONAL+="-d 0 -f 1.5"
	fi
	if [ $i == "squaredrag" ]
	then
		ADDITIONAL+="-d 0.25"
	fi
	for j in {0..19}
	do
		$BASE $ADDITIONAL -p shapes/square.txt -o $NAME/$NAME$j -x
	done
done

echo "Complete!"
