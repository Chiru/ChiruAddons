#!/bin/bash
pwd=`pwd`
DIR="/home/vatjjar/projects/Tundra-scenes/oulu3d-128"
cd $DIR/assets
echo "Converting material refs into JPG"
for i in *.material; do sed -i "s#dds#jpg#" $i; done
for i in *.material; do sed -i "s#pkm#jpg#" $i; done
echo "Done!"
cd $pwd
./run-oulu3d-128.sh "$@"

