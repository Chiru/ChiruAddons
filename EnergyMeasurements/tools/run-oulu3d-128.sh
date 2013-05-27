#!/bin/bash
rm -f libNeocortex.so
ln -s ../src/libNeocortex.so libNeocortex.so

DIR="/home/vatjjar/projects/Tundra-scenes"
if [ ! -d "$DIR" ]; then
        echo "Assuming scenes in $DIR, but it does not exist. Abort!"
        exit 0;
fi

PYTHONPATH=../src/ python TXMLLoader.py -t $DIR/oulu3d-128/oulu3d-neocortex2.txml -p $DIR/oulu3d-128/assets "$@"
