#!/bin/bash

TARGET_DIRS=(
    "/root/code/datasets/fastlivo2/output/Log/Colmap/images"
    "/root/code/datasets/fastlivo2/output/Log/Colmap/sparse/0"
)

for dir in "${TARGET_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        rm -rf "$dir"
        echo "Removed: $dir"
    else
        echo "Not found: $dir"
    fi
done

for dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo "Created: $dir"
    else
        echo "Exists: $dir"
    fi
done

