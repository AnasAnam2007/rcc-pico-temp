#!/usr/bin/env bash

# Build the static version of the library.

here=$(pwd)
thisscript=$(basename $0)
workdir=$here/xdata/$thisscript
argumentumgit="file://$(realpath ../../../argumentum)"
builddir=out/build

if [ -d $workdir ]; then
   rm -rf $workdir
fi

if [ ! -d $workdir ]; then
   mkdir -p $workdir
fi

create_test_dir() {
   cd $workdir

   git -c "protocol.file.allow=always" clone $argumentumgit argumentum

   cd $workdir/argumentum
   mkdir -p $builddir
}

configure() {
   cd $workdir/argumentum
   # local debug=--debug-output
   cmake -S . -B $builddir -D ARGUMENTUM_BUILD_STATIC_LIBS=ON $debug
}

build() {
   cd $workdir/argumentum
   # local debug=--debug-output
   cmake --build $builddir $debug
}

create_test_dir
configure
build

