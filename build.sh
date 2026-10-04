#!/usr/bin/env bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

set -e
set -x

# debug=true
debug=false
gcc=false
ue_root="${UE_ROOT:-}"
ue_toolchain="${UE_TOOLCHAIN:-}"
UE_CMAKE_ARGS=()
# Parse command line arguments
while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
    --debug)
        debug=true
        shift # past argument
        ;;
    --gcc)
        gcc=true
        shift # past argument
        ;;
    --ue-root)
        ue_root="${2:?--ue-root needs the folder that contains Engine/}"
        shift # past argument
        shift # past value
        ;;
    --ue-toolchain)
        ue_toolchain="${2:?--ue-toolchain needs the x86_64-unknown-linux-gnu folder of the toolchain}"
        shift # past argument
        shift # past value
        ;;
    *)
        echo "ERROR: unknown argument: $key"
        exit 1
        ;;
    esac

done

function version_less_than_equal_to() { test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"; }

# check for rpclib
RPC_VERSION_FOLDER="rpclib-2.3.1"
if [ ! -d "./external/rpclib/$RPC_VERSION_FOLDER" ]; then
    echo "ERROR: new version of AirSim requires newer rpclib."
    echo "please run setup.sh first and then run build.sh again."
    exit 1
fi

# check for local cmake build created by setup.sh
if [ -d "./cmake_build" ]; then
    if [ "$(uname)" == "Darwin" ]; then
        CMAKE="$(greadlink -f cmake_build/bin/cmake)"
    else
        CMAKE="$(readlink -f cmake_build/bin/cmake)"
    fi
else
    CMAKE=$(which cmake)
fi

# variable for build output
if $debug; then
    build_dir=build_debug
else
    build_dir=build_release
fi 
if [ "$(uname)" == "Darwin" ]; then
    # llvm v8 is too old for Big Sur see
    # https://github.com/microsoft/AirSim/issues/3691
    #export CC=/usr/local/opt/llvm@8/bin/clang
    #export CXX=/usr/local/opt/llvm@8/bin/clang++
    #now pick up whatever setup.sh installs
    export CC="$(brew --prefix)/opt/llvm/bin/clang"
    export CXX="$(brew --prefix)/opt/llvm/bin/clang++"
elif [[ -n "$ue_root" || -n "$ue_toolchain" ]]; then
    # Build with Unreal Engine's bundled clang, sysroot and libc++, the same toolchain the
    # plugin is built with. Needed on Ubuntu 24.04+, which has no clang-12 and whose newer
    # glibc emits symbols (e.g. __isoc23_strtol) that UE's older sysroot does not provide.
    # --ue-toolchain takes the toolchain folder directly, e.g. Epic's native Linux toolchain
    # for an engine installed on Windows, which has no HostLinux toolchain.
    if $gcc; then
        echo "ERROR: --ue-root/--ue-toolchain and --gcc are mutually exclusive."
        exit 1
    fi
    if [[ -z "$ue_toolchain" ]]; then
        ue_toolchains=("$ue_root"/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/*/x86_64-unknown-linux-gnu)
        if [[ ${#ue_toolchains[@]} -ne 1 || ! -x "${ue_toolchains[0]}/bin/clang++" ]]; then
            echo "ERROR: expected one Unreal Engine Linux toolchain matching"
            echo "  $ue_root/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/*/x86_64-unknown-linux-gnu"
            echo "Pass the folder that contains Engine/ (run Unreal's Setup.sh first for a source build),"
            echo "or the toolchain's x86_64-unknown-linux-gnu folder with --ue-toolchain."
            exit 1
        fi
        ue_toolchain="${ue_toolchains[0]}"
    elif [[ ! -x "$ue_toolchain/bin/clang++" ]]; then
        echo "ERROR: $ue_toolchain/bin/clang++ not found; --ue-toolchain needs the x86_64-unknown-linux-gnu folder."
        exit 1
    fi
    # UE 5.2 keeps libc++ in Engine/Source/ThirdParty/Unix/LibCxx; UE 5.5+ ships it in the toolchain.
    ue_libcxx_include="$ue_root/Engine/Source/ThirdParty/Unix/LibCxx/include/c++/v1"
    ue_libcxx_lib="$ue_root/Engine/Source/ThirdParty/Unix/LibCxx/lib/Unix/x86_64-unknown-linux-gnu"
    if [[ -z "$ue_root" || ! -f "$ue_libcxx_include/__config" ]]; then
        ue_libcxx_include="$ue_toolchain/include/c++/v1"
        ue_libcxx_lib="$ue_toolchain/lib64"
    fi
    if [[ ! -f "$ue_libcxx_include/__config" || ! -f "$ue_libcxx_lib/libc++.a" || ! -f "$ue_libcxx_lib/libc++abi.a" ]]; then
        echo "ERROR: Unreal Engine's libc++ headers or static libraries not found"
        echo "  (looked in Engine/Source/ThirdParty/Unix/LibCxx and $ue_toolchain)"
        exit 1
    fi
    echo "Using Unreal Engine's bundled Linux toolchain at $ue_toolchain"
    export CC="$ue_toolchain/bin/clang"
    export CXX="$ue_toolchain/bin/clang++"
    UE_CMAKE_ARGS=(
        "-DCMAKE_SYSROOT=$ue_toolchain"
        -DUSING_UE_TOOLCHAIN=ON
        "-DUE_LIBCXX_INCLUDE_DIR=$ue_libcxx_include"
        -DCMAKE_EXE_LINKER_FLAGS=-nostdlib++
        "-DCMAKE_CXX_STANDARD_LIBRARIES=-Wl,--start-group \"$ue_libcxx_lib/libc++.a\" \"$ue_libcxx_lib/libc++abi.a\" -Wl,--end-group -lpthread -ldl -lm"
    )
else
    VERSION=$(lsb_release -rs | cut -d. -f1)
    if $gcc; then
        # GCC 12 where installed, otherwise the distribution's GCC (e.g. Ubuntu 26.04)
        if command -v g++-12 >/dev/null; then
            export CC="gcc-12"
            export CXX="g++-12"
        else
            export CC="gcc"
            export CXX="g++"
        fi
    else
        if ! command -v clang++-12 >/dev/null; then
            echo "ERROR: clang++-12 not found. On Ubuntu 24.04 and newer, build with Unreal Engine's"
            echo "toolchain instead: ./build.sh --ue-root /path/to/UnrealEngine (or set UE_ROOT)."
            exit 1
        fi
        export CC="clang-12"
        export CXX="clang++-12"
    fi
fi

#install EIGEN library
if [[ ! -d "./AirLib/deps/eigen3/Eigen" ]]; then
    echo "### Eigen is not installed. Please run setup.sh first."
    exit 1
fi

echo "putting build in $build_dir folder, to clean, just delete the directory..."

# this ensures the cmake files will be built in our $build_dir instead.
if [[ -f "./cmake/CMakeCache.txt" ]]; then
    rm "./cmake/CMakeCache.txt"
fi
if [[ -d "./cmake/CMakeFiles" ]]; then
    rm -rf "./cmake/CMakeFiles"
fi



if [[ ! -d $build_dir ]]; then
    mkdir -p $build_dir
fi

# Fix for Unreal/Unity using x86_64 (Rosetta) on Apple Silicon hardware.
# Also: modern CMake (>=4.0) dropped support for the very old
# cmake_minimum_required() versions declared by some subprojects
# (e.g. MavLinkTest), so pin a policy floor to let them configure anyway.
CMAKE_VARS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
if [ "$(uname)" == "Darwin" ]; then
    CMAKE_VARS="$CMAKE_VARS -DCMAKE_APPLE_SILICON_PROCESSOR=x86_64"
fi

pushd $build_dir  >/dev/null
if $debug; then
    folder_name="Debug"
    "$CMAKE" ../cmake -DCMAKE_BUILD_TYPE=Debug $CMAKE_VARS "${UE_CMAKE_ARGS[@]}" \
        || (popd && rm -r $build_dir && exit 1)
else
    folder_name="Release"
    "$CMAKE" ../cmake -DCMAKE_BUILD_TYPE=Release $CMAKE_VARS "${UE_CMAKE_ARGS[@]}" \
        || (popd && rm -r $build_dir && exit 1)
fi
popd >/dev/null


pushd $build_dir  >/dev/null
# final linking of the binaries can fail due to a missing libc++abi library
# (happens on Fedora, see https://bugzilla.redhat.com/show_bug.cgi?id=1332306).
# So we only build the libraries here for now
make -j"$(nproc)"
popd >/dev/null

mkdir -p AirLib/lib/x64/$folder_name
mkdir -p AirLib/deps/rpclib/lib
mkdir -p AirLib/deps/MavLinkCom/lib
cp $build_dir/output/lib/libAirLib.a AirLib/lib
cp $build_dir/output/lib/libMavLinkCom.a AirLib/deps/MavLinkCom/lib
cp $build_dir/output/lib/librpc.a AirLib/deps/rpclib/lib/librpc.a

# Update AirLib/lib, AirLib/deps, Plugins folders with new binaries
rsync -a --delete $build_dir/output/lib/ AirLib/lib/x64/$folder_name
rsync -a --delete external/rpclib/$RPC_VERSION_FOLDER/include AirLib/deps/rpclib
rsync -a --delete MavLinkCom/include AirLib/deps/MavLinkCom
rsync -a --delete AirLib Unreal/Plugins/AirSim/Source
rm -rf Unreal/Plugins/AirSim/Source/AirLib/src

set +x

echo ""
echo ""
echo "==============================="
echo " Cosys-AirSim plugin is built! "
echo "==============================="
echo ""
echo "For further info see for installation see:"
echo "https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/docs/install_linux.md"
echo "=================================================================="

popd >/dev/null
