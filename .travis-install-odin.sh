#!/bin/bash
VERSION="systemtest"
ODIN="odin-data-${VERSION}"
ODIN_PREFIX=$HOME/$ODIN
git clone https://github.com/dls-controls/odin-data.git
cd odin-data
git checkout $VERSION
mkdir -p build;
cd build;
cmake -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON -DCMAKE_INSTALL_PREFIX=$ODIN_PREFIX -DHDF5_ROOT=$HDF5_ROOT -DBLOSC_ROOT_DIR=$BLOSC_ROOT -DKAFKA_ROOT_DIR=$KAFKA_ROOT ..
make -j 8 VERBOSE=1
make install
