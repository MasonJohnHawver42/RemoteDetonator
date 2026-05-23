#!/bin/bash
set -e

SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
REPO_DIR=$(dirname "$SCRIPT_DIR")

cd "$REPO_DIR"
sudo env PATH=$PATH podman build -f Containerfile.deploy -t heltec-deployer .
