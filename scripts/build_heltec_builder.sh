#!/bin/bash

SCRIPT_PATH="${BASH_SOURCE[0]}"
SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
REPO_DIR=$(dirname "$SCRIPT_DIR")

cd $REPO_DIR
podman build -t heltec-builder .

