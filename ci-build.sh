#!/bin/bash

# Run CI setup

set -e
set -x
set -o errexit
set -o pipefail

function build_rust () {
  VERBOSE=1 cargo build --bin rtcm-converter --all-features --all-targets --release -vv
  VERBOSE=1 cargo test --all-features --all-targets --release -vv
  VERBOSE=1 cargo build --package rtcm --no-default-features --features sbp_master -vv
  VERBOSE=1 cargo build --all-features --all-targets --release -vv
}

function build_c() {
    cd c
    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=test_install -Dnov2sbp_BUILD=true ..
    make -j8 VERBOSE=1
    make do-all-tests
    make run_test_novatel_parser
    make install
    cd ../..
}

function build_codecov() {

    bazel run //:refresh_compile_commands
    bazel coverage --collect_code_coverage --combined_report=lcov --coverage_report_generator=@bazel_sonarqube//:sonarqube_coverage_generator //...

    if [[ -z "${SONAR_SCANNER_VERSION}" ]]; then
	echo "Error: SONAR_SCANNER_VERSION must be configured" >&2
	exit 1
    fi

    export SONAR_SCANNER_HOME="${HOME}/.sonar/sonar-scanner-${SONAR_SCANNER_VERSION}-linux"
    export SONAR_SCANNER_OPTS="-server"

    # Run sonar scanner

    SONAR_OTHER_ARGS="\
	-Dsonar.projectVersion=1.0 \
	-Dsonar.sources=. \
	-Dsonar.coverageReportPaths=./bazel-out/_coverage/_coverage_report.dat \
        -Dsonar.cfamily.compile-commands=./compile_commands.json \
	-Dsonar.cfamily.threads=1 \
	-Dsonar.cfamily.cache.enabled=false \
	-Dsonar.sourceEncoding=UTF-8"

    # shellcheck disable=SC2086
    sonar-scanner \
      "-Dsonar.login=${SONAR_TOKEN}" \
      "-Dsonar.host.url=https://sonarcloud.io" \
      "-Dsonar.projectName=gnss-converters-bazel" \
      "-Dsonar.projectKey=swift-nav_gnss-converters-bazel" \
      ${SONAR_OTHER_ARGS}
}

# Assumes that we've already run cargo and stack!
function package () {
  if [[ "$OS_NAME" == "windows" ]]; then
    cd ./target/release
    strip.exe rtcm3tosbp.exe sbp2rtcm.exe ubx2sbp.exe ubx2json.exe ixcom2sbp.exe nov2sbp.exe rtcm32json.exe json2rtcm3.exe mrtjs2rtjs.exe rtcm-converter.exe;
    7z a -tzip ../../gnss_converters_windows.zip rtcm3tosbp.exe sbp2rtcm.exe ubx2sbp.exe ubx2json.exe ixcom2sbp.exe nov2sbp.exe rtcm32json.exe json2rtcm3.exe mrtjs2rtjs.exe rtcm-converter.exe;
    cd ../..;
    git status; git diff --exit-code --quiet
    VERSION="$(git describe --always --tags --dirty)";
    BUILD_TRIPLET="$($CC -dumpmachine)";
    mv gnss_converters_windows.zip "gnss_converters-${VERSION}-windows-${BUILD_TRIPLET}.zip";
    echo "gnss_converters-${VERSION}-windows-${BUILD_TRIPLET}.zip" >release-archive.filename;
    ls -l;
  else
    (cd target/release; strip rtcm3tosbp sbp2rtcm ubx2sbp ubx2json ixcom2sbp nov2sbp rtcm32json json2rtcm3 mrtjs2rtjs rtcm-converter);
    tar -C "target/release" -czf gnss_converters.tar.gz rtcm3tosbp sbp2rtcm ubx2sbp ubx2json ixcom2sbp nov2sbp rtcm32json json2rtcm3 mrtjs2rtjs rtcm-converter;
    git status; git diff --exit-code --quiet
    VERSION="$(git describe --always --tags --dirty)";
    BUILD_TRIPLET="$($CC -dumpmachine)";
    mv gnss_converters.tar.gz "gnss_converters-${VERSION}-${BUILD_TRIPLET}.tar.gz";
    echo "gnss_converters-${VERSION}-${BUILD_TRIPLET}.tar.gz" >release-archive.filename;
    ls -l;
  fi
}

if [ "$TESTENV" == "codecov" ]; then
  build_codecov
elif [ "$TESTENV" == "rust" ]; then
  build_rust
elif [ "$TESTENV" == "package" ]; then
  package
else
  build_c
fi
