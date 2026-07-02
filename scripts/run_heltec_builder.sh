#!/bin/bash

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
REPO_DIR=$(dirname "$SCRIPT_DIR")
WORK_DIR=$(dirname "$REPO_DIR")

echo $WORK_DIR/LoraMacLink
ls $WORK_DIR/LoraMacLink

cd $REPO_DIR
podman run --rm \
    -v $(pwd):/project:Z \
    -v $WORK_DIR/LoraMacLink:/project/lib/LoraMacLink:Z \
    -v heltec-pio-cache:/root/.platformio \
    heltec-builder
