#!/bin/bash
set -e
set -x

# This scripts assumes it's run from under src/ChiruAddons in the Tundra tree.

# Note:
# Remember to run build-ubuntu-deps.bash script before executing this script.
# Script it self wont check if given files are already downloaded or compiled,
# so running this script twice will cause possible errors.

RAPTOR=raptor2-2.0.7
RASQAL=rasqal-0.9.28
REDLAND=redland-1.0.15

viewer=$(dirname $(readlink -f $0))/../..
deps=$viewer/../naali-deps
prefix=$deps/install
build=$deps/build
tarballs=$deps/tarballs

# Set enviroment variables for pkg-config and gcc/g++ paths.
export PKG_CONFIG_PATH=$deps/install/lib/pkgconfig
export LD_LIBRARY_PATH=$deps/install/lib:LD_LIBRARY_PATH
export C_INCLUDE_PATH=$deps/install/include:C_INCLUDE_PATH
export CPLUS_INCLUDE_PATH=$deps/install/include:CPLUS_INCLUDE_PATH

cd $build
wget -P $tarballs http://download.librdf.org/source/$RAPTOR.tar.gz
tar zxf $tarballs/$RAPTOR.tar.gz
cd $build/$RAPTOR
./configure --prefix=$prefix
make
make install

cd $build
wget -P $tarballs http://download.librdf.org/source/$RASQAL.tar.gz
tar zxf $tarballs/$RASQAL.tar.gz
cd $build/$RASQAL
./configure --prefix=$prefix
make
make install

cd $build
wget -P $tarballs http://download.librdf.org/source/$REDLAND.tar.gz
tar zxf $tarballs/$REDLAND.tar.gz
cd $build/$REDLAND
./configure --prefix=$prefix
make
make install
