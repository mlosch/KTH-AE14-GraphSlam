#!/bin/bash

indir="bags/"
outdir="dats/"

mkdir -p outdir

for f in $indir*.bag
do
	command="./print-bag.py $f > $outdir`basename $f .bag`.dat"
	echo $command
	eval $command
done
