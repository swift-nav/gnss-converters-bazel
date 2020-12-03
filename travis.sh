#!/bin/bash

# Run Travis setup

set -e
set -x
set -o errexit
set -o pipefail

function build_haskell () {
    cd haskell
    stack build --test
    cd ..
}

function build_rust () {
  cargo build --release -vv
  cargo build --release -vv --features nov2sbp --bin nov2sbp
}

function build_c() {
    cd c
    mkdir build
    cd build
    /usr/bin/cmake -DCMAKE_INSTALL_PREFIX=test_install -Dnov2sbp_BUILD=true ..
    make -j8 VERBOSE=1
    make do-all-tests
    make run_test_novatel_parser
    make install
    cd ../..
}

function build_codecov() {
    cd c
    mkdir build
    cd build
    /usr/bin/cmake -DCODE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug -Dnov2sbp_BUILD=true .. 2>&1 >cmake.log
    tail cmake.log
    make -j8 ccov-all 2>&1 >ccov.log
    tail ccov.log
    cd ../..
}

if [ "$TESTENV" == "stack" ]; then
  build_haskell
elif [ "$TESTENV" == "codecov" ]; then
  build_codecov
elif [ "$TESTENV" == "rust" ]; then
  build_rust
else
  build_c
fi
