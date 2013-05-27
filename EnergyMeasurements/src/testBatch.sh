#!/bin/bash

FRAMES=2000
LOG=allmeasurements.log

rm -f $LOG

for RES in 3; do #1 2 3; do
	if [ "$RES" -eq 1 ]; then
		XRES=320
		YRES=240
	fi
	if [ "$RES" -eq 2 ]; then
		XRES=800
		YRES=480
	fi
	if [ "$RES" -eq 3 ]; then
		XRES=1280
		YRES=720
	fi

  	for TEST in 0 1 2; do # test case 3 would be empty scene
        for LOD in 0 1 2 3 4; do
            #iterations
            for ITER in 1 2 3 4 5 6 7 8 9 10; do #2 3 4 5 6 7 8 9 10; do
                LD_LIBRARY_PATH=./ ./test $TEST $LOD $XRES $YRES 0 20 >> $LOG
            done # iterations
        done
    done
done

python ../tools/parselog.py $LOG >$LOG.parsed

