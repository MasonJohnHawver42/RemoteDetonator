#!/bin/bash

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
REPO_DIR=$(dirname "$SCRIPT_DIR")
WORK_DIR=$(dirname "$REPO_DIR")

echo $WORK_DIR/lora-maclink
ls $WORK_DIR/lora-maclink

cd $REPO_DIR
podman run --rm \
    -v $(pwd):/project:Z \
    -v $WORK_DIR/lora-maclink:/project/lib/lora-maclink:Z \
    heltec-builder
