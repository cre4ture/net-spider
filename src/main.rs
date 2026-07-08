#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

mod ch9120;
mod mcp23017;

use core::fmt::Write;

use ch9120::{Ch9120, NetworkConfig, NetworkMode};
use cortex_m::delay::Delay;
use mcp23017::Mcp23017;
#[cfg(not(test))]
use panic_halt as _;
use rp2040_hal::Clock;
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::fugit::{ExtU32, RateExtU32};
use rp2040_hal::gpio::Pins;
use rp2040_hal::i2c::I2C;
use rp2040_hal::pac;
use rp2040_hal::sio::Sio;
use rp2040_hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral};
use rp2040_hal::watchdog::Watchdog;

const XTAL_FREQ_HZ: u32 = 12_000_000;
const COMMAND_BUFFER_LEN: usize = 64;
const LOCAL_SLOT_COUNT: usize = 8;
const MAX_EXPANDERS: usize = 8;
const EXPANDER_SLOT_COUNT: usize = 16;
const TOTAL_SLOT_CAPACITY: usize = LOCAL_SLOT_COUNT + MAX_EXPANDERS * EXPANDER_SLOT_COUNT;
const MCP23017_BASE_ADDRESS: u8 = 0x20;
const CONTROL_PINS: [u8; LOCAL_SLOT_COUNT] = [2, 3, 4, 5, 6, 7, 8, 9];
const MCP23017_I2C_FREQ_HZ: u32 = 100_000;
const MCP23017_SDA_PIN: u8 = 26;
const MCP23017_SCL_PIN: u8 = 27;
const WATCHDOG_TIMEOUT_MS: u32 = 8_000;
const NETWORK_CONFIG: NetworkConfig = NetworkConfig {
    mode: NetworkMode::TcpServer,
    local_ip: [192, 168, 1, 200],
    subnet_mask: [255, 255, 255, 0],
    gateway: [192, 168, 1, 1],
    local_port: 5_000,
    target_ip: [192, 168, 1, 10],
    target_port: 5_000,
    transport_baud: 115_200,
};

#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[derive(Clone, Copy)]
enum DriveState {
    High,
    Low,
    HiZ,
}

impl DriveState {
    const fn label(self) -> &'static str {
        match self {
            Self::High => "HIGH",
            Self::Low => "LOW",
            Self::HiZ => "HI-Z",
        }
    }
}

enum ParsedCommand {
    Help,
    Status,
    Scan,
    SetOne { slot: usize, state: DriveState },
    SetAll(DriveState),
}

const fn expander_slot_range(index: usize) -> (usize, usize) {
    let start = LOCAL_SLOT_COUNT + index * EXPANDER_SLOT_COUNT + 1;
    (start, start + EXPANDER_SLOT_COUNT - 1)
}

struct LocalPins {
    pin_numbers: [u8; LOCAL_SLOT_COUNT],
    states: [DriveState; LOCAL_SLOT_COUNT],
}

impl LocalPins {
    fn new(pin_numbers: [u8; LOCAL_SLOT_COUNT]) -> Self {
        let controller = Self {
            pin_numbers,
            states: [DriveState::HiZ; LOCAL_SLOT_COUNT],
        };

        controller.apply_mask_state(controller.mask(), DriveState::HiZ);
        controller
    }

    fn set(&mut self, slot: usize, state: DriveState) {
        self.apply_state(self.pin_numbers[slot], state);
        self.states[slot] = state;
    }

    fn set_all(&mut self, state: DriveState) {
        for slot in 0..self.pin_numbers.len() {
            self.set(slot, state);
        }
    }

    fn mask(&self) -> u32 {
        self.pin_numbers
            .iter()
            .fold(0u32, |mask, pin| mask | (1u32 << pin))
    }

    // Real SIO register access is skipped under `cfg(test)`: host unit tests
    // run on x86_64, where `pac::SIO::ptr()` is not a valid address and
    // dereferencing it would crash the test binary instead of exercising
    // firmware logic.
    #[cfg(not(test))]
    fn apply_mask_state(&self, mask: u32, state: DriveState) {
        let sio = unsafe { &*pac::SIO::ptr() };

        match state {
            DriveState::High => {
                sio.gpio_out_set().write(|w| unsafe { w.bits(mask) });
                sio.gpio_oe_set().write(|w| unsafe { w.bits(mask) });
            }
            DriveState::Low => {
                sio.gpio_out_clr().write(|w| unsafe { w.bits(mask) });
                sio.gpio_oe_set().write(|w| unsafe { w.bits(mask) });
            }
            DriveState::HiZ => {
                sio.gpio_oe_clr().write(|w| unsafe { w.bits(mask) });
            }
        }
    }

    #[cfg(test)]
    fn apply_mask_state(&self, _mask: u32, _state: DriveState) {}

    fn apply_state(&self, pin: u8, state: DriveState) {
        self.apply_mask_state(1u32 << pin, state);
    }
}

// `ExpanderBank`/`ExpanderState` intentionally never own the I2C bus handle.
// Detection used to run unconditionally at boot, before the UART/TCP command
// loop started; on a board without a populated (or correctly wired) MCP23017
// bus, `rp2040-hal`'s blocking I2C driver has no timeout and can spin forever
// inside `detect_expanders`, which permanently starved the loop that answers
// commands over the CH9120's TCP port. Detection is now opt-in via the SCAN
// command (see `CommandPins::scan`), so boot can never block on I2C, and the
// bus handle lives on `CommandPins` so it can be retried after a failed scan.
struct ExpanderBank {
    devices: [Option<Mcp23017>; MAX_EXPANDERS],
}

impl ExpanderBank {
    fn new() -> Self {
        Self {
            devices: [None; MAX_EXPANDERS],
        }
    }

    fn insert(&mut self, device: Mcp23017) {
        let index = usize::from(device.address() - MCP23017_BASE_ADDRESS);
        self.devices[index] = Some(device);
    }

    fn count(&self) -> usize {
        self.devices
            .iter()
            .filter(|device| device.is_some())
            .count()
    }

    fn device(&self, index: usize) -> Option<&Mcp23017> {
        self.devices.get(index)?.as_ref()
    }

    fn set<I2C>(
        &mut self,
        i2c: &mut I2C,
        slot: usize,
        state: DriveState,
    ) -> Result<(), &'static str>
    where
        I2C: embedded_hal::i2c::I2c,
    {
        let device_index = slot / EXPANDER_SLOT_COUNT;
        let pin_index = slot % EXPANDER_SLOT_COUNT;

        let Some(device) = self.devices.get_mut(device_index).and_then(Option::as_mut) else {
            return Err("target pin belongs to an unavailable mcp23017 address block");
        };

        device
            .set_pin_state(i2c, pin_index, state)
            .map_err(|_| "mcp23017 i2c write failed")
    }

    fn set_all<I2C>(&mut self, i2c: &mut I2C, state: DriveState) -> Result<(), &'static str>
    where
        I2C: embedded_hal::i2c::I2c,
    {
        for device in self.devices.iter_mut().flatten() {
            device
                .set_all_state(i2c, state)
                .map_err(|_| "mcp23017 i2c write failed")?;
        }

        Ok(())
    }
}

enum ExpanderState {
    Uninitialized,
    Ready(ExpanderBank),
    NotFound,
    Fault,
}

fn detect_expanders<I2C>(i2c: &mut I2C) -> ExpanderState
where
    I2C: embedded_hal::i2c::I2c,
{
    let mut expander = ExpanderBank::new();

    for address in MCP23017_BASE_ADDRESS..=(MCP23017_BASE_ADDRESS + MAX_EXPANDERS as u8 - 1) {
        match Mcp23017::probe(i2c, address) {
            Ok(true) => match Mcp23017::new(i2c, address) {
                Ok(device) => expander.insert(device),
                Err(_) => return ExpanderState::Fault,
            },
            Ok(false) => {}
            Err(_) => return ExpanderState::Fault,
        }
    }

    if expander.count() == 0 {
        ExpanderState::NotFound
    } else {
        ExpanderState::Ready(expander)
    }
}

struct CommandPins<I2C> {
    local: LocalPins,
    i2c: I2C,
    expander: ExpanderState,
}

impl<I2C> CommandPins<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    fn new(pin_numbers: [u8; LOCAL_SLOT_COUNT], i2c: I2C) -> Self {
        Self {
            local: LocalPins::new(pin_numbers),
            i2c,
            expander: ExpanderState::Uninitialized,
        }
    }

    /// Runs (or re-runs) MCP23017 detection. The only place that touches the
    /// I2C bus outside of an explicit expander pin write — never called
    /// implicitly at boot or from `set`/`set_all`.
    fn scan(&mut self) {
        self.expander = detect_expanders(&mut self.i2c);
    }

    fn set(&mut self, slot: usize, state: DriveState) -> Result<(), &'static str> {
        if slot < LOCAL_SLOT_COUNT {
            self.local.set(slot, state);
            return Ok(());
        }

        let expander_slot = slot - LOCAL_SLOT_COUNT;

        match &mut self.expander {
            ExpanderState::Ready(expander) => expander.set(&mut self.i2c, expander_slot, state),
            ExpanderState::Uninitialized => Err("mcp23017 not scanned yet; send SCAN"),
            ExpanderState::NotFound => Err("mcp23017 not detected on GP26/GP27"),
            ExpanderState::Fault => Err("mcp23017 init failed; recheck wiring and power"),
        }
    }

    fn set_all(&mut self, state: DriveState) -> Result<(), &'static str> {
        self.local.set_all(state);

        if let ExpanderState::Ready(expander) = &mut self.expander {
            expander.set_all(&mut self.i2c, state)?;
        }

        Ok(())
    }
}

#[cfg(not(test))]
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().expect("RP2040 peripherals can only be taken once");
    let core = cortex_m::Peripherals::take().expect("core peripherals can only be taken once");

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .expect("clock tree should be initializable");

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let _tcpcs = pins.gpio17.into_floating_input();
    let cfg_pin = pins.gpio18.into_push_pull_output();
    let rst_pin = pins.gpio19.into_push_pull_output();
    let uart_pins = (pins.gpio20.into_function(), pins.gpio21.into_function());

    let _gp2 = pins.gpio2.into_floating_input();
    let _gp3 = pins.gpio3.into_floating_input();
    let _gp4 = pins.gpio4.into_floating_input();
    let _gp5 = pins.gpio5.into_floating_input();
    let _gp6 = pins.gpio6.into_floating_input();
    let _gp7 = pins.gpio7.into_floating_input();
    let _gp8 = pins.gpio8.into_floating_input();
    let _gp9 = pins.gpio9.into_floating_input();

    let i2c = I2C::i2c1(
        pac.I2C1,
        pins.gpio26.reconfigure(),
        pins.gpio27.reconfigure(),
        MCP23017_I2C_FREQ_HZ.Hz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let config_uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(
                ch9120::CONFIG_BAUD.Hz(),
                DataBits::Eight,
                None,
                StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .expect("CH9120 config UART should be initializable");

    delay.delay_ms(1_000);

    let mut ch9120 = Ch9120::new(cfg_pin, rst_pin);
    let mut uart = ch9120.configure(
        config_uart,
        &mut delay,
        clocks.peripheral_clock.freq(),
        NETWORK_CONFIG,
    );

    // MCP23017 detection is intentionally NOT run here. It used to block boot
    // (see the comment on `ExpanderBank`); it's now opt-in via the SCAN
    // command so a missing/misbehaving I2C bus can never stop the board from
    // answering on the CH9120's TCP port.
    let mut controlled_pins = CommandPins::new(CONTROL_PINS, i2c);
    let mut line_buffer = [0u8; COMMAND_BUFFER_LEN];
    let mut line_len = 0usize;

    write_startup_banner(&controlled_pins, &mut uart);

    // Backstop for the one place that can still block on I2C at runtime: the
    // SCAN command and expander pin writes. If either wedges on a bad bus,
    // the watchdog resets the board instead of leaving it silently hung.
    watchdog.start(WATCHDOG_TIMEOUT_MS.millis());

    loop {
        watchdog.feed();

        let mut rx = [0u8; 32];

        match uart.read_raw(&mut rx) {
            Ok(count) => {
                for &byte in &rx[..count] {
                    ingest_byte(
                        byte,
                        &mut line_buffer,
                        &mut line_len,
                        &mut controlled_pins,
                        &mut uart,
                    );
                }
            }
            Err(nb::Error::WouldBlock) => {}
            Err(nb::Error::Other(_)) => {
                let _ = write!(uart, "ERR uart read failure\r\n");
            }
        }
    }
}

fn ingest_byte<I2C, P>(
    byte: u8,
    line_buffer: &mut [u8; COMMAND_BUFFER_LEN],
    line_len: &mut usize,
    controlled_pins: &mut CommandPins<I2C>,
    uart: &mut UartPeripheral<rp2040_hal::uart::Enabled, pac::UART1, P>,
) where
    I2C: embedded_hal::i2c::I2c,
    P: rp2040_hal::uart::ValidUartPinout<pac::UART1>,
{
    match byte {
        b'\r' | b'\n' => {
            if *line_len == 0 {
                return;
            }

            process_line(&line_buffer[..*line_len], controlled_pins, uart);
            *line_len = 0;
        }
        0x08 | 0x7F => {
            *line_len = line_len.saturating_sub(1);
        }
        b if b.is_ascii_graphic() || b == b' ' => {
            if *line_len == line_buffer.len() {
                *line_len = 0;
                let _ = write!(uart, "ERR command too long\r\n");
            } else {
                line_buffer[*line_len] = b;
                *line_len += 1;
            }
        }
        _ => {}
    }
}

fn process_line<I2C, P>(
    raw_line: &[u8],
    controlled_pins: &mut CommandPins<I2C>,
    uart: &mut UartPeripheral<rp2040_hal::uart::Enabled, pac::UART1, P>,
) where
    I2C: embedded_hal::i2c::I2c,
    P: rp2040_hal::uart::ValidUartPinout<pac::UART1>,
{
    let Ok(line) = core::str::from_utf8(raw_line) else {
        let _ = write!(uart, "ERR ASCII commands only\r\n");
        return;
    };

    match parse_command(line) {
        Ok(ParsedCommand::Help) => write_help(controlled_pins, uart),
        Ok(ParsedCommand::Status) => write_status(controlled_pins, uart),
        Ok(ParsedCommand::Scan) => {
            controlled_pins.scan();
            write_scan_result(controlled_pins, uart);
        }
        Ok(ParsedCommand::SetOne { slot, state }) => match controlled_pins.set(slot, state) {
            Ok(()) => {
                let _ = write!(uart, "OK ");
                write_pin_label(slot, uart);
                let _ = write!(uart, " {}\r\n", state.label());
            }
            Err(message) => {
                let _ = write!(uart, "ERR {message}\r\n");
            }
        },
        Ok(ParsedCommand::SetAll(state)) => match controlled_pins.set_all(state) {
            Ok(()) => {
                let _ = write!(uart, "OK ALL {}\r\n", state.label());
            }
            Err(message) => {
                let _ = write!(uart, "ERR {message}\r\n");
            }
        },
        Err(message) => {
            let _ = write!(uart, "ERR {message}\r\n");
        }
    }
}

fn write_startup_banner<I2C, P>(
    controlled_pins: &CommandPins<I2C>,
    uart: &mut UartPeripheral<rp2040_hal::uart::Enabled, pac::UART1, P>,
) where
    I2C: embedded_hal::i2c::I2c,
    P: rp2040_hal::uart::ValidUartPinout<pac::UART1>,
{
    let _ = write!(
        uart,
        "\r\nRP2040-ETH ready at {}.{}.{}.{}:{}\r\n\
LOCAL P1=GP2 P2=GP3 P3=GP4 P4=GP5 P5=GP6 P6=GP7 P7=GP8 P8=GP9\r\n",
        NETWORK_CONFIG.local_ip[0],
        NETWORK_CONFIG.local_ip[1],
        NETWORK_CONFIG.local_ip[2],
        NETWORK_CONFIG.local_ip[3],
        NETWORK_CONFIG.local_port,
    );

    match &controlled_pins.expander {
        ExpanderState::Uninitialized => {
            let _ = write!(
                uart,
                "MCP23017 not scanned yet on GP{}(SDA)/GP{}(SCL); send SCAN to detect, only P1..P8 available until then\r\n",
                MCP23017_SDA_PIN, MCP23017_SCL_PIN,
            );
        }
        ExpanderState::Ready(expander) => {
            let _ = write!(
                uart,
                "MCP23017 count={} on GP{}(SDA)/GP{}(SCL): fixed mapping P9..P136\r\n",
                expander.count(),
                MCP23017_SDA_PIN,
                MCP23017_SCL_PIN,
            );

            let _ = write!(uart, "EXPANDERS ");
            for index in 0..MAX_EXPANDERS {
                if let Some(device) = expander.device(index) {
                    let (start, end) = expander_slot_range(index);
                    let separator = if index + 1 == MAX_EXPANDERS
                        || (index + 1..MAX_EXPANDERS).all(|next| expander.device(next).is_none())
                    {
                        "\r\n"
                    } else {
                        " "
                    };
                    let _ = write!(
                        uart,
                        "E{}=0x{:02X}[P{}..P{}]{}",
                        index + 1,
                        device.address(),
                        start,
                        end,
                        separator,
                    );
                }
            }
        }
        ExpanderState::NotFound => {
            let _ = write!(
                uart,
                "No MCP23017 detected on GP{}(SDA)/GP{}(SCL); only P1..P8 available\r\n",
                MCP23017_SDA_PIN, MCP23017_SCL_PIN,
            );
        }
        ExpanderState::Fault => {
            let _ = write!(
                uart,
                "MCP23017 probe failed on GP{}(SDA)/GP{}(SCL); recheck wiring and supply voltage\r\n",
                MCP23017_SDA_PIN, MCP23017_SCL_PIN,
            );
        }
    }

    let _ = write!(
        uart,
        "Try HELP, STATUS, SCAN, P1 HIGH, E2X3 LOW, or ALL HI-Z\r\n"
    );
}

fn write_help<I2C, P>(
    controlled_pins: &CommandPins<I2C>,
    uart: &mut UartPeripheral<rp2040_hal::uart::Enabled, pac::UART1, P>,
) where
    I2C: embedded_hal::i2c::I2c,
    P: rp2040_hal::uart::ValidUartPinout<pac::UART1>,
{
    match &controlled_pins.expander {
        ExpanderState::Ready(_) => {
            let _ = write!(
                uart,
                "OK commands: HELP, STATUS, SCAN, P1..P136 HIGH|LOW|HI-Z, E1X1..E8X16, E1GPA0..E8GPB7, ALL HIGH|LOW|HI-Z; STATUS lists detected address blocks\r\n"
            );
        }
        ExpanderState::Uninitialized | ExpanderState::NotFound | ExpanderState::Fault => {
            let _ = write!(
                uart,
                "OK commands: HELP, STATUS, SCAN, P1..P8 HIGH|LOW|HI-Z, P9..P136 reserved for MCP23017 address blocks 0x20..0x27 (run SCAN to detect), ALL HIGH|LOW|HI-Z\r\n"
            );
        }
    }
}

fn write_scan_result<I2C, P>(
    controlled_pins: &CommandPins<I2C>,
    uart: &mut UartPeripheral<rp2040_hal::uart::Enabled, pac::UART1, P>,
) where
    I2C: embedded_hal::i2c::I2c,
    P: rp2040_hal::uart::ValidUartPinout<pac::UART1>,
{
    match &controlled_pins.expander {
        ExpanderState::Ready(expander) => {
            let _ = write!(uart, "OK SCAN count={}\r\n", expander.count());
        }
        ExpanderState::NotFound => {
            let _ = write!(uart, "OK SCAN count=0\r\n");
        }
        ExpanderState::Fault => {
            let _ = write!(
                uart,
                "ERR mcp23017 scan failed; recheck wiring and power\r\n"
            );
        }
        ExpanderState::Uninitialized => {
            let _ = write!(uart, "ERR scan did not complete\r\n");
        }
    }
}

fn write_status<I2C, P>(
    controlled_pins: &CommandPins<I2C>,
    uart: &mut UartPeripheral<rp2040_hal::uart::Enabled, pac::UART1, P>,
) where
    I2C: embedded_hal::i2c::I2c,
    P: rp2040_hal::uart::ValidUartPinout<pac::UART1>,
{
    let _ = write!(uart, "STATUS LOCAL ");
    for (slot, state) in controlled_pins.local.states.iter().enumerate() {
        let separator = if slot + 1 == LOCAL_SLOT_COUNT {
            "\r\n"
        } else {
            " "
        };
        let _ = write!(uart, "P{}={}{}", slot + 1, state.label(), separator);
    }

    match &controlled_pins.expander {
        ExpanderState::Ready(expander) => {
            let _ = write!(
                uart,
                "STATUS MCP23017 count={} SDA=GP{} SCL=GP{}\r\n",
                expander.count(),
                MCP23017_SDA_PIN,
                MCP23017_SCL_PIN,
            );

            for device_index in 0..MAX_EXPANDERS {
                let Some(device) = expander.device(device_index) else {
                    continue;
                };

                let (start, end) = expander_slot_range(device_index);
                let _ = write!(
                    uart,
                    "STATUS E{} addr=0x{:02X} slots=P{}..P{}\r\n",
                    device_index + 1,
                    device.address(),
                    start,
                    end,
                );

                let _ = write!(uart, "STATUS E{}A ", device_index + 1);
                for (pin, state) in device.states()[..8].iter().enumerate() {
                    let separator = if pin == 7 { "\r\n" } else { " " };
                    let _ = write!(
                        uart,
                        "E{}X{}={}{}",
                        device_index + 1,
                        pin + 1,
                        state.label(),
                        separator,
                    );
                }

                let _ = write!(uart, "STATUS E{}B ", device_index + 1);
                for (pin, state) in device.states()[8..].iter().enumerate() {
                    let separator = if pin == 7 { "\r\n" } else { " " };
                    let _ = write!(
                        uart,
                        "E{}X{}={}{}",
                        device_index + 1,
                        pin + 9,
                        state.label(),
                        separator,
                    );
                }
            }
        }
        ExpanderState::Uninitialized => {
            let _ = write!(uart, "STATUS MCP23017=not-scanned; send SCAN\r\n");
        }
        ExpanderState::NotFound => {
            let _ = write!(
                uart,
                "STATUS MCP23017=not-detected SDA=GP{} SCL=GP{}\r\n",
                MCP23017_SDA_PIN, MCP23017_SCL_PIN,
            );
        }
        ExpanderState::Fault => {
            let _ = write!(uart, "STATUS MCP23017=probe-failed\r\n");
        }
    }
}

fn write_pin_label<P>(
    slot: usize,
    uart: &mut UartPeripheral<rp2040_hal::uart::Enabled, pac::UART1, P>,
) where
    P: rp2040_hal::uart::ValidUartPinout<pac::UART1>,
{
    if slot < LOCAL_SLOT_COUNT {
        let _ = write!(uart, "GP{}(P{})", CONTROL_PINS[slot], slot + 1);
        return;
    }

    let expander_slot = slot - LOCAL_SLOT_COUNT;
    let expander_index = expander_slot / EXPANDER_SLOT_COUNT;
    let device_pin = expander_slot % EXPANDER_SLOT_COUNT;
    let bank = if device_pin < 8 { 'A' } else { 'B' };
    let bit = device_pin % 8;

    let _ = write!(
        uart,
        "E{}.GP{}{}(P{}/E{}X{})",
        expander_index + 1,
        bank,
        bit,
        slot + 1,
        expander_index + 1,
        device_pin + 1,
    );
}

fn parse_command(line: &str) -> Result<ParsedCommand, &'static str> {
    let mut parts = line.split_ascii_whitespace();
    let first = parts.next().ok_or("empty command")?;

    if first.eq_ignore_ascii_case("HELP") && parts.next().is_none() {
        return Ok(ParsedCommand::Help);
    }

    if first.eq_ignore_ascii_case("STATUS") && parts.next().is_none() {
        return Ok(ParsedCommand::Status);
    }

    if first.eq_ignore_ascii_case("SCAN") && parts.next().is_none() {
        return Ok(ParsedCommand::Scan);
    }

    if first.eq_ignore_ascii_case("ALL") {
        let state = parts.next().ok_or("missing state for ALL")?;
        if parts.next().is_some() {
            return Err("too many arguments");
        }
        return Ok(ParsedCommand::SetAll(parse_state(state)?));
    }

    if first.eq_ignore_ascii_case("SET") {
        let target = parts.next().ok_or("missing pin after SET")?;
        let state = parts.next().ok_or("missing state after pin")?;
        if parts.next().is_some() {
            return Err("too many arguments");
        }
        return Ok(ParsedCommand::SetOne {
            slot: parse_slot(target)?,
            state: parse_state(state)?,
        });
    }

    let state = parts.next().ok_or("missing state")?;
    if parts.next().is_some() {
        return Err("too many arguments");
    }

    Ok(ParsedCommand::SetOne {
        slot: parse_slot(first)?,
        state: parse_state(state)?,
    })
}

fn parse_slot(token: &str) -> Result<usize, &'static str> {
    if let Some(slot) = parse_prefixed_1_based(token, "P", TOTAL_SLOT_CAPACITY) {
        return Ok(slot - 1);
    }

    if let Ok(slot) = token.parse::<usize>()
        && (1..=TOTAL_SLOT_CAPACITY).contains(&slot)
    {
        return Ok(slot - 1);
    }

    if let Some(pin) = parse_prefixed_1_based(token, "GP", 9)
        && (2..=9).contains(&pin)
    {
        return Ok(pin - 2);
    }

    if let Some(slot) = parse_expander_scoped_slot(token) {
        return Ok(slot);
    }

    Err("unknown pin, use P1..P136, GP2..GP9, E1X1..E8X16, or E1GPA0..E8GPB7")
}

fn parse_expander_scoped_slot(token: &str) -> Option<usize> {
    let suffix = strip_ascii_prefix(token, "E")?;
    let digits_len = suffix
        .bytes()
        .take_while(|byte| byte.is_ascii_digit())
        .count();

    if digits_len == 0 {
        return None;
    }

    let (expander_text, pin_text) = suffix.split_at(digits_len);
    let expander_index = expander_text.parse::<usize>().ok()?;

    if !(1..=MAX_EXPANDERS).contains(&expander_index) {
        return None;
    }

    let base = LOCAL_SLOT_COUNT + (expander_index - 1) * EXPANDER_SLOT_COUNT;

    if let Some(slot) = parse_prefixed_1_based(pin_text, "X", EXPANDER_SLOT_COUNT) {
        return Some(base + slot - 1);
    }

    if let Some(bit) = parse_prefixed_0_based(pin_text, "GPA", 8) {
        return Some(base + bit);
    }

    if let Some(bit) = parse_prefixed_0_based(pin_text, "GPB", 8) {
        return Some(base + 8 + bit);
    }

    None
}

fn parse_prefixed_1_based(token: &str, prefix: &str, max: usize) -> Option<usize> {
    let suffix = strip_ascii_prefix(token, prefix)?;
    let value = suffix.parse::<usize>().ok()?;
    (1..=max).contains(&value).then_some(value)
}

fn parse_prefixed_0_based(token: &str, prefix: &str, max: usize) -> Option<usize> {
    let suffix = strip_ascii_prefix(token, prefix)?;
    let value = suffix.parse::<usize>().ok()?;
    (value < max).then_some(value)
}

fn strip_ascii_prefix<'a>(token: &'a str, prefix: &str) -> Option<&'a str> {
    let head = token.get(..prefix.len())?;
    head.eq_ignore_ascii_case(prefix)
        .then_some(&token[prefix.len()..])
}

fn parse_state(token: &str) -> Result<DriveState, &'static str> {
    if token.eq_ignore_ascii_case("HIGH")
        || token.eq_ignore_ascii_case("H")
        || token.eq_ignore_ascii_case("ON")
        || token.eq_ignore_ascii_case("1")
    {
        return Ok(DriveState::High);
    }

    if token.eq_ignore_ascii_case("LOW")
        || token.eq_ignore_ascii_case("L")
        || token.eq_ignore_ascii_case("OFF")
        || token.eq_ignore_ascii_case("0")
    {
        return Ok(DriveState::Low);
    }

    if token.eq_ignore_ascii_case("HI-Z")
        || token.eq_ignore_ascii_case("Z")
        || token.eq_ignore_ascii_case("NEUTRAL")
        || token.eq_ignore_ascii_case("FLOAT")
    {
        return Ok(DriveState::HiZ);
    }

    Err("unknown state, use HIGH, LOW, or HI-Z")
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{
        Error as _, ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress,
    };

    // Regression coverage for the boot hang fixed by making MCP23017
    // detection opt-in (see the comment on `ExpanderBank`): before that fix,
    // `detect_expanders` ran unconditionally at boot using a blocking I2C
    // driver with no timeout, so a bad/absent bus could permanently starve
    // the UART/TCP command loop. These tests assert the bus is only ever
    // touched by an explicit `scan()`, never by construction or by commands
    // that don't need it.

    #[derive(Debug)]
    struct FakeI2cError;

    impl embedded_hal::i2c::Error for FakeI2cError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
        }
    }

    /// Minimal fake I2C bus: NACKs every address except (optionally) one
    /// "present" device address, and counts how many transactions ran.
    struct FakeI2c {
        present_address: Option<u8>,
        calls: usize,
    }

    impl FakeI2c {
        fn no_devices() -> Self {
            Self {
                present_address: None,
                calls: 0,
            }
        }

        fn with_device_at(address: u8) -> Self {
            Self {
                present_address: Some(address),
                calls: 0,
            }
        }
    }

    impl ErrorType for FakeI2c {
        type Error = FakeI2cError;
    }

    impl I2c<SevenBitAddress> for FakeI2c {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            self.calls += 1;

            if self.present_address != Some(address) {
                return Err(FakeI2cError);
            }

            for operation in operations {
                if let Operation::Read(buffer) = operation {
                    buffer.fill(0);
                }
            }

            Ok(())
        }
    }

    #[test]
    fn boot_never_touches_i2c_bus() {
        let pins = CommandPins::new(CONTROL_PINS, FakeI2c::no_devices());

        assert_eq!(pins.i2c.calls, 0, "constructing CommandPins must not scan");
        assert!(matches!(pins.expander, ExpanderState::Uninitialized));
    }

    #[test]
    fn expander_slot_before_scan_errors_without_touching_i2c() {
        let mut pins = CommandPins::new(CONTROL_PINS, FakeI2c::no_devices());

        let result = pins.set(LOCAL_SLOT_COUNT, DriveState::High);

        assert!(result.is_err());
        assert_eq!(pins.i2c.calls, 0);
    }

    #[test]
    fn local_pin_commands_never_touch_i2c_even_after_a_failed_scan() {
        let mut pins = CommandPins::new(CONTROL_PINS, FakeI2c::no_devices());

        pins.scan();
        assert!(matches!(pins.expander, ExpanderState::NotFound));
        let calls_after_scan = pins.i2c.calls;
        assert!(calls_after_scan > 0, "scan should have probed the bus");

        assert!(pins.set(0, DriveState::High).is_ok());
        assert_eq!(
            pins.i2c.calls, calls_after_scan,
            "local pin writes must never touch the I2C bus"
        );
    }

    #[test]
    fn scan_detects_device_and_allows_expander_writes() {
        let mut pins =
            CommandPins::new(CONTROL_PINS, FakeI2c::with_device_at(MCP23017_BASE_ADDRESS));

        pins.scan();

        match &pins.expander {
            ExpanderState::Ready(bank) => assert_eq!(bank.count(), 1),
            _ => panic!("expected a detected expander, got a different state"),
        }

        assert!(pins.set(LOCAL_SLOT_COUNT, DriveState::High).is_ok());
    }

    #[test]
    fn parses_scan_command_case_insensitively() {
        assert!(matches!(parse_command("scan"), Ok(ParsedCommand::Scan)));
        assert!(matches!(parse_command("SCAN"), Ok(ParsedCommand::Scan)));
        assert!(parse_command("SCAN extra").is_err());
    }
}
