#!/bin/bash
pwd=`pwd`
DIR="/home/vatjjar/projects/Tundra-scenes/oulu3d-128"
cd $DIR/assets
echo "Converting material refs into PKM/ETC1"
for i in *.material; do sed -i "s#dds#pkm#" $i; done
for i in *.material; do sed -i "s#jpg#pkm#" $i; done
echo "Done!"
cd $pwd
./run-oulu3d-128.sh "$@"

