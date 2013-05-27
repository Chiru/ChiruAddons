#!/bin/bash
pwd=`pwd`
DIR="/home/vatjjar/projects/Tundra-scenes/oulu3d-128"
cd $DIR/assets
echo "Converting material refs into DDS"
for i in *.material; do sed -i "s#jpg#dds#" $i; done
for i in *.material; do sed -i "s#pkm#dds#" $i; done
echo "Done!"
cd $pwd
./run-oulu3d-128.sh "$@"

