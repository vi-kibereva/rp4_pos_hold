#!/bin/sh
set -eu

# Default values
BUILD_DIR="${BUILD_DIR:-build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

# Parse command line arguments
for arg in "$@"; do
  case $arg in
    --debug)
      BUILD_TYPE="Debug"
      shift
      ;;
    --release)
      BUILD_TYPE="Release"
      shift
      ;;
    --help)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --debug     Build with debug symbols and no optimizations"
      echo "  --release   Build with optimizations (default)"
      echo "  --help      Show this help message"
      echo ""
      echo "Environment variables:"
      echo "  BUILD_DIR   Build directory (default: build)"
      exit 0
      ;;
    *)
      # Unknown option
      ;;
  esac
done

echo "Building in $BUILD_TYPE mode..."

cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$BUILD_DIR" --parallel
