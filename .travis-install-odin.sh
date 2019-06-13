#!/bin/bash
VERSION="systemtest"
ODIN="odin-data-${VERSION}"
ODIN_PREFIX=$HOME/$ODIN
wget https://github.com/dls-controls/odin-data/archive/${VERSION}.tar.gz
tar xvfz ${VERSION}.tar.gz
cd "$ODIN"
mkdir build;
cd build;
cmake -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON -DCMAKE_INSTALL_PREFIX=$ODIN_PREFIX -DHDF5_ROOT=$HDF5_ROOT ..
make -j 4 VERBOSE=1
make install
