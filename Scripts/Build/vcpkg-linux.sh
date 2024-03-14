#!/bin/bash

if [[ ! -d "${UE_ENGINE}" ]]; then
	echo ERROR: UE_ENGINE envvar not set or specifies non-existant location
	exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
DIR=$DIR/vcpkg

# this is a tag in the vcpkg repository
VCPKG_VERSION=2024.02.14

# enable manifest mode
VCPKG_FEATURE_FLAGS=manifests

# deduce vcpkg nomenclature for this system
if [ `uname` == "Linux" ]; then
	VCPKG_SYSTEM=Linux
else
	echo Error: Linux libraries can only be built on Linux.
	exit 1
fi

VCPKG_ROOT=$DIR/tmp/vcpkg-${VCPKG_SYSTEM}-${VCPKG_VERSION}

echo
echo === Checking out vcpkg to ${VCPKG_ROOT}===
git clone --single-branch --branch $VCPKG_VERSION -- https://github.com/microsoft/vcpkg.git $VCPKG_ROOT

echo
echo === Bootstrapping vcpkg ===
${VCPKG_ROOT}/bootstrap-vcpkg.sh -disableMetrics

echo
echo === Making Linux artifacts writeable ===
chmod -R u+w $DIR/Linux


echo
echo === Copying vcpkg.json ===
mkdir -p $DIR/Linux/x86_64-unknown-linux-gnu
cp $DIR/vcpkg.json $DIR/Linux/x86_64-unknown-linux-gnu/vcpkg.json

echo
echo === Running vcpkg in manifest mode ===
# --x-manifest-root tells it to consume build directives in vcpkg.json
# --overlay-triplets tells it to resolve a named triplet via additional paths outside vcpkg/, PWD relative
# --triplet names the triplet to configure the build with, our custom triplet file w/o .cmake extentions
# --debug will provide extra information to stdout
${VCPKG_ROOT}/vcpkg install \
	--overlay-ports=$DIR/overlay-ports \
	--overlay-triplets=$DIR/overlay-triplets \
	--x-manifest-root=$DIR/Linux/x86_64-unknown-linux-gnu \
	--x-packages-root=$DIR/Linux/x86_64-unknown-linux-gnu \
	--triplet=x86_64-unknown-linux-gnu

echo
echo === Replacing symlinks with actual files ===
for f in $(find $DIR/Linux -type l);
do
	rsync `realpath $f` $f
done

