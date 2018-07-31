#!/bin/bash

cd
cd Documents/USRA/push

BASE="./push -r 200 -b 500 -z 0.5 -s 0.5 -t C -y H -g 50"
BASESMALL="200Robots500Boxes"

# Produce all square results
squaresarray=( "square_tight" )
circleswitcharray=( "0" "1" )
for i in "${squaresarray[@]}"
do
	for l in "${circleswitcharray[@]}"
	do	
		QUALIFIER=""
		if [ $l == "0" ]
		then
			QUALIFIER=""
		fi
		if [ $l == "1" ]
		then
			QUALIFIER="_switch"
		fi
		NAME=$i$QUALIFIER
		cd Results_Replays
		cd $BASESMALL
		mkdir $NAME
		cd ..
		cd ..
		cd Results
		cd $BASESMALL
		mkdir $NAME
		cd ..
		cd ..
		if [ $i == "square_tight" ]
		then
			ADDITIONAL=" -d 0"
		fi
		if [ $i == "squareflare_preprime_tight" ]
		then
			ADDITIONAL=" -d 0 -f 1.5"
		fi
		if [ $i == "squaredrag_tight" ]
		then
			ADDITIONAL=" -d 0.25"
		fi
		for j in {40..99}
		do
			$BASE $ADDITIONAL -c $l -p shapes/square.txt -o $BASESMALL/$NAME/$NAME$j -x
		done
	done
done

echo "Complete!"
