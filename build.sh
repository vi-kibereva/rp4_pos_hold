#!/usr/bin/env bash
set -euo pipefail

# ---- defaults ----
BUILD_TYPE="${BUILD_TYPE:-Release}" # or Debug
BUILD_DIR="${BUILD_DIR:-build}"
TARGET="${TARGET:-main_gr}"
JOBS="${JOBS:-$(
  (command -v sysctl >/dev/null && sysctl -n hw.ncpu) ||
    (command -v nproc >/dev/null && nproc) ||
    echo 4
)}"
GEN_COMPILE_COMMANDS="${GEN_COMPILE_COMMANDS:-1}" # 1 = symlink compile_commands.json to repo root
TOOLCHAIN_FILE="${TOOLCHAIN_FILE:-}"              # e.g., toolchains/rpi.cmake
EXTRA_CMAKE_FLAGS=()                              # keep as array

usage() {
  cat <<'EOF'
Usage: ./build.sh [options]

Options:
  -d, --debug            Build Debug (default Release)
  -r, --release          Build Release
  -b, --build-dir DIR    Build directory (default: build)
  -t, --target NAME      Target to build (default: main_gr)
  -j, --jobs N           Parallel jobs (default: autodetect)
  -c, --clean            Delete build dir before configuring
  --asan                 Enable AddressSanitizer
  --ubsan                Enable UndefinedBehaviorSanitizer
  --toolchain FILE       Pass a CMake toolchain file
  --flag "CMAKE_FLAG"    Append raw CMake flag (can repeat)
  -h, --help             Show this help
EOF
}

CLEAN=0
ASAN=0
UBSAN=0

# ---- parse args ----
while [[ $# -gt 0 ]]; do
  case "$1" in
  -d | --debug)
    BUILD_TYPE="Debug"
    shift
    ;;
  -r | --release)
    BUILD_TYPE="Release"
    shift
    ;;
  -b | --build-dir)
    [[ $# -ge 2 ]] || {
      echo "Missing arg for $1"
      exit 1
    }
    BUILD_DIR="$2"
    shift 2
    ;;
  -t | --target)
    [[ $# -ge 2 ]] || {
      echo "Missing arg for $1"
      exit 1
    }
    TARGET="$2"
    shift 2
    ;;
  -j | --jobs)
    [[ $# -ge 2 ]] || {
      echo "Missing arg for $1"
      exit 1
    }
    JOBS="$2"
    shift 2
    ;;
  -c | --clean)
    CLEAN=1
    shift
    ;;
  --asan)
    ASAN=1
    shift
    ;;
  --ubsan)
    UBSAN=1
    shift
    ;;
  --toolchain)
    [[ $# -ge 2 ]] || {
      echo "Missing arg for $1"
      exit 1
    }
    TOOLCHAIN_FILE="$2"
    shift 2
    ;;
  --flag)
    [[ $# -ge 2 ]] || {
      echo "Missing arg for $1"
      exit 1
    }
    EXTRA_CMAKE_FLAGS+=("$2")
    shift 2
    ;;
  -h | --help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown arg: $1"
    usage
    exit 1
    ;;
  esac
done

# ---- prepare build dir ----
if [[ "$CLEAN" -eq 1 && -d "$BUILD_DIR" ]]; then
  echo "[clean] removing $BUILD_DIR"
  rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"

# ---- sanitizer flags (clang/gcc) ----
SAN_FLAGS=""
[[ "$ASAN" -eq 1 ]] && SAN_FLAGS+=" -fsanitize=address"
[[ "$UBSAN" -eq 1 ]] && SAN_FLAGS+=" -fsanitize=undefined"
if [[ -n "$SAN_FLAGS" ]]; then
  EXTRA_CMAKE_FLAGS+=("-DCMAKE_CXX_FLAGS=${SAN_FLAGS} -fno-omit-frame-pointer")
fi

# ---- toolchain ----
[[ -n "$TOOLCHAIN_FILE" ]] && EXTRA_CMAKE_FLAGS+=("-DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}")

echo "[configure] type=${BUILD_TYPE} dir=${BUILD_DIR}"
# Guard empty-array expansion for Bash 3.2 + set -u
if ((${#EXTRA_CMAKE_FLAGS[@]})); then
  cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    "${EXTRA_CMAKE_FLAGS[@]}"
else
  cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi

echo "[build] target=${TARGET} jobs=${JOBS}"
cmake --build "$BUILD_DIR" --target "$TARGET" -j "$JOBS"

# ---- compile_commands.json symlink for clangd ----
if [[ "$GEN_COMPILE_COMMANDS" -eq 1 && -f "$BUILD_DIR/compile_commands.json" ]]; then
  ln -sf "$BUILD_DIR/compile_commands.json" compile_commands.json
fi

echo "[done] Built $TARGET in $BUILD_DIR"
echo "Run it with: ./$BUILD_DIR/$TARGET"
