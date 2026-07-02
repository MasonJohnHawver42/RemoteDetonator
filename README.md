# RemoteDetonator

Two-board firmware demo running over the [LML](../LoraMacLink/) MAC layer on
the Heltec WiFi LoRa 32 V3 (SX1262 + SSD1306 OLED).

Today it runs a placeholder request/response loop: the **master** sends an
8-bit `n` to the **slave**, the slave computes `fib(n)` as a `uint64_t`, and
the master displays the round-trip result on its OLED. Both peers use the
identical LML stack — the master/slave roles live entirely in the app layer.

---

## Repository layout

```
RemoteDetonator/
├── platformio.ini           # master + slave envs share [common]
├── Containerfile.build      # PlatformIO build image
├── Containerfile.deploy     # esptool flash image
├── scripts/
│   ├── build_heltec_builder.sh   # build the PIO image
│   ├── run_heltec_builder.sh     # compile both firmwares
│   ├── build_heltec_deployer.sh  # build the flasher image
│   └── run_heltec_deployer.sh    # flash one device
└── src/
    ├── main_master.cpp      # request loop + display
    └── main_slave.cpp       # fib service + display
```

The LML library is consumed from a sibling checkout, mounted into the build
container at `/project/lib/lora-maclink`. The `platformio.ini` `lib_deps`
points at that mount via `symlink:///project/lib/lora-maclink`, so the
container path matters.

---

## Task structure

Both firmwares pin three FreeRTOS tasks the same way:

| Task | Core | Priority | Job |
|------|------|----------|-----|
| `protocolTask` | 0 | 3 | owns the radio, calls `mac.poll()` in a tight loop |
| `appTask` | 1 | 2 | blocks on `mac.rx_queue`, sends via `mac.queue_tx()` |
| `displayTask` | any | 1 | renders two text lines on the OLED |

A shared `displayMutex` guards the two display lines; the app task writes,
the display task reads.

The master kicks off by queueing the first `n` and then loops on `rx_queue`,
retrying on `Event::Error` or unexpected reply length. The slave runs purely
reactively — wait for a message, compute, queue the reply, repeat.

---

## Build & flash (containerised)

The build runs in a PlatformIO container so the toolchain stays out of the
host. Both scripts use `podman`.

```sh
# 1. Build the PIO image (cached after the first run)
./scripts/build_heltec_builder.sh

# 2. Compile both master and slave firmware
./scripts/run_heltec_builder.sh

# 3. Build the esptool image (one-time)
./scripts/build_heltec_deployer.sh

# 4. Flash a board (auto-detects /dev/ttyACM*, /dev/ttyUSB*)
./scripts/run_heltec_deployer.sh master            # uses first detected port
./scripts/run_heltec_deployer.sh slave /dev/ttyACM1
HELTEC_PORT=/dev/ttyUSB0 ./scripts/run_heltec_deployer.sh master
```

`run_heltec_builder.sh` expects a sibling `lora-maclink` checkout at
`../lora-maclink` relative to the repo root and mounts it into the container.

The deploy script requires `sudo` for USB access and writes three artefacts:

```
0x0      bootloader.bin
0x8000   partitions.bin
0x10000  firmware.bin
```

---

## Building without containers

`platformio.ini` is a normal PIO project. From the repo root with the LML
sources symlinked into `lib/lora-maclink`:

```sh
pio run -e master
pio run -e slave
pio run -e master -t upload
pio run -e slave  -t upload
```

---

## Hardware

Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + SSD1306). Pin mapping is hard-coded
in both `main_*.cpp` files; see the LML README for the table.

---

## Roadmap

Today the message payload is raw bytes (`uint8_t n` one way, `uint64_t fib`
the other). Next: replace the ad-hoc payloads with a protobuf/nanopb message
envelope so the app speaks structured commands (arm, disarm, fire, status)
instead of byte-spans.
