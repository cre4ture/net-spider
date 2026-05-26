# net-spider

Rust firmware for the Waveshare RP2040-ETH board.

The firmware configures the onboard CH9120 for `TCP Server` mode and then listens
for ASCII commands arriving over the Ethernet socket. Four unused header pins are
exposed as tri-state channels:

- `P1` -> `GP2`
- `P2` -> `GP3`
- `P3` -> `GP4`
- `P4` -> `GP5`

`ON` drives the pin high, `OFF` drives the pin low, and `NEUTRAL` makes the pin
high impedance again.

## Default network settings

- IP: `192.168.1.200`
- Subnet mask: `255.255.255.0`
- Gateway: `192.168.1.1`
- TCP port: `5000`
- UART transport baud: `115200`

Edit `src/main.rs` if your network needs different values.

## Commands

The parser accepts either direct commands or a `SET` prefix:

```text
HELP
STATUS
P1 ON
P2 OFF
P3 NEUTRAL
SET P4 ON
ALL OFF
ALL Z
```

Aliases:

- Pin selectors: `P1`..`P4`, `1`..`4`, `GP2`..`GP5`
- High: `ON`, `HIGH`, `1`
- Low: `OFF`, `LOW`, `0`
- High impedance: `NEUTRAL`, `FLOAT`, `Z`, `HI-Z`

## Build

```bash
rustup target add thumbv6m-none-eabi
cargo build --release
```

To flash over USB boot mode, `elf2uf2-rs` is convenient:

```bash
cargo install elf2uf2-rs --locked
elf2uf2-rs target/thumbv6m-none-eabi/release/net-spider
```
