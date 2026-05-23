#!/bin/bash
set -e

SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
REPO_DIR=$(dirname "$SCRIPT_DIR")

TARGET="${1:-}"

if [[ -z "$TARGET" ]]; then
    echo "ERROR: Target required. Usage:" >&2
    echo "  $0 master [port]" >&2
    echo "  $0 slave  [port]" >&2
    exit 1
fi

if [[ "$TARGET" != "master" && "$TARGET" != "slave" ]]; then
    echo "ERROR: Target must be 'master' or 'slave', got: $TARGET" >&2
    exit 1
fi

BUILD_DIR="$REPO_DIR/.pio/build/$TARGET"

PORT="${2:-${HELTEC_PORT:-}}"

if [[ -z "$PORT" ]]; then
    for candidate in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
        if [[ -e "$candidate" ]]; then
            PORT="$candidate"
            echo "Auto-detected port: $PORT"
            break
        fi
    done
fi

if [[ -z "$PORT" ]]; then
    echo "ERROR: No serial port found. Plug in the device or pass the port explicitly:" >&2
    echo "  $0 $TARGET /dev/ttyACM0" >&2
    exit 1
fi

for f in "$BUILD_DIR/bootloader.bin" "$BUILD_DIR/partitions.bin" "$BUILD_DIR/firmware.bin"; do
    if [[ ! -f "$f" ]]; then
        echo "ERROR: Missing build artifact: $f" >&2
        echo "Run scripts/run_heltec_builder.sh first." >&2
        exit 1
    fi
done

echo "Flashing $TARGET on $PORT..."
sudo env PATH=$PATH podman run --rm \
    --privileged \
    --device "$PORT" \
    -v "$REPO_DIR:/project:Z" \
    heltec-deployer \
    --chip esp32s3 \
    --port "$PORT" \
    --baud 921600 \
    --before default-reset \
    --after hard-reset \
    write-flash \
    0x0     .pio/build/$TARGET/bootloader.bin \
    0x8000  .pio/build/$TARGET/partitions.bin \
    0x10000 .pio/build/$TARGET/firmware.bin
