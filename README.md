# net-spider

Rust firmware for the Waveshare RP2040-ETH board.

The firmware configures the onboard CH9120 for `TCP Server` mode and then listens
for ASCII commands arriving over the Ethernet socket. Eight unused header pins are
exposed as tri-state channels:

- `P1` -> `GP2`
- `P2` -> `GP3`
- `P3` -> `GP4`
- `P4` -> `GP5`
- `P5` -> `GP6`
- `P6` -> `GP7`
- `P7` -> `GP8`
- `P8` -> `GP9`

`HIGH` drives the pin high, `LOW` drives the pin low, and `HI-Z` makes the pin
high impedance again.

## MCP23017 expansion

The firmware now supports up to eight external MCP23017 I2C expanders on the
same bus and exposes their GPIOs as additional tri-state channels.

### Wiring

- `GP26` -> all MCP23017 `SDA`
- `GP27` -> all MCP23017 `SCL`
- `3V3(OUT)` -> all MCP23017 `VCC`
- `GND` -> all MCP23017 `GND`

Every MCP23017 on the bus needs a unique address via `A0`, `A1`, and `A2`.
The firmware scans `0x20..0x27` at boot and enables every responding device it
finds.

The supported command mapping is:

- `P9`..`P24` -> first MCP23017 (`E1`)
- `P25`..`P40` -> second MCP23017 (`E2`)
- ...
- `P121`..`P136` -> eighth MCP23017 (`E8`)
- `E1X1`..`E8X16` -> explicit per-expander pin aliases
- `E1GPA0`..`E8GPB7` -> explicit bank/bit aliases
- `X1`..`X16`, `M1`..`M16`, `GPA0`..`GPB7` -> legacy aliases for the first expander only

`HI-Z` maps to input mode without pull-up on the MCP23017, so the line is no
longer driven by the expander.

Important: power the MCP23017 module from `3.3V` when it shares the bus directly
with the RP2040. Many breakout boards pull `SDA` and `SCL` up to their supply rail.

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
P1 HIGH
P5 LOW
P8 HI-Z
P12 HIGH
P25 LOW
X3 LOW
E2X3 LOW
GPA7 HIGH
E2GPB0 HI-Z
SET P4 H
ALL Z
```

Preferred state names are `HIGH`, `LOW`, and `HI-Z`. The parser also accepts
these aliases:

- Pin selectors:
  - Local: `P1`..`P8`, `1`..`8`, `GP2`..`GP9`
  - Global expander slots: `P9`..`P136`, `9`..`136`
  - First expander aliases: `X1`..`X16`, `M1`..`M16`, `GPA0`..`GPA7`, `GPB0`..`GPB7`
  - Explicit expander aliases: `E1X1`..`E8X16`, `E1GPA0`..`E8GPA7`, `E1GPB0`..`E8GPB7`
- High: `H`, `ON`, `1`
- Low: `L`, `OFF`, `0`
- High impedance: `Z`, `NEUTRAL`, `FLOAT`

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
