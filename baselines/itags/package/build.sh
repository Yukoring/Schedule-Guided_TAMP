#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
upstream_sha=e779ca8b6c9877ca51a5ffa5b4550dcdb3e0cd38
if [[ ! -d upstream/.git ]]; then
  git clone https://github.com/gneville6/D-ITAGS.git upstream
  git -C upstream checkout --detach "$upstream_sha"
fi
[[ "$(git -C upstream rev-parse HEAD)" == "$upstream_sha" ]]
if git -C upstream apply --reverse --check ../upstream_cpsat.patch 2>/dev/null; then
  :
else
  git -C upstream apply --check ../upstream_cpsat.patch
  git -C upstream apply ../upstream_cpsat.patch
fi
ortools_path="${ORTOOLS_ROOT:-$PWD/deps/or-tools_x86_64_Ubuntu-24.04_cpp_v9.14.6206}"
if [[ ! -f "$ortools_path/lib/cmake/ortools/ortoolsConfig.cmake" ]]; then
  if [[ -n "${ORTOOLS_ROOT:-}" ]]; then
    echo 'ORTOOLS_ROOT does not contain the expected CMake package.' >&2
    exit 2
  fi
  mkdir -p deps
  curl -fL --retry 2 \
    https://github.com/google/or-tools/releases/download/v9.14/or-tools_amd64_ubuntu-24.04_cpp_v9.14.6206.tar.gz \
    -o deps/ortools.tar.gz
  tar --no-same-owner -xzf deps/ortools.tar.gz -C deps
fi
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="$ortools_path"
cmake --build build -j 4
