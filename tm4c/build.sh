#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

mkdir -p dist/
mkdir -p build/
rm -f dist/*
cd build

CORES=$(nproc)
echo "Running build with $CORES threads."

rm -fr ./*
cmake ..
make VERBOSE=1
#make -j$CORES
mv sdrx* ../dist

cd ..
rm -fr build

exit 0
