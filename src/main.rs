#![no_std]
#![no_main]

mod ch9120;

use core::fmt::Write;

use ch9120::{Ch9120, NetworkConfig, NetworkMode};
use cortex_m::delay::Delay;
use panic_halt as _;
use rp2040_hal::Clock;
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::gpio::Pins;
use rp2040_hal::pac;
use rp2040_hal::sio::Sio;
use rp2040_hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral};
use rp2040_hal::watchdog::Watchdog;

const XTAL_FREQ_HZ: u32 = 12_000_000;
const COMMAND_BUFFER_LEN: usize = 64;
const CONTROL_PINS: [u8; 8] = [2, 3, 4, 5, 6, 7, 8, 9];
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
    On,
    Off,
    Neutral,
}

impl DriveState {
    const fn label(self) -> &'static str {
        match self {
            Self::On => "ON",
            Self::Off => "OFF",
            Self::Neutral => "NEUTRAL",
        }
    }
}

enum ParsedCommand {
    Help,
    Status,
    SetOne { slot: usize, state: DriveState },
    SetAll(DriveState),
}

struct CommandPins {
    pin_numbers: [u8; 8],
    states: [DriveState; 8],
}

impl CommandPins {
    fn new(pin_numbers: [u8; 8]) -> Self {
        let controller = Self {
            pin_numbers,
            states: [DriveState::Neutral; 8],
        };

        controller.apply_mask_state(controller.mask(), DriveState::Neutral);
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

    fn apply_mask_state(&self, mask: u32, state: DriveState) {
        let sio = unsafe { &*pac::SIO::ptr() };

        match state {
            DriveState::On => {
                sio.gpio_out_set().write(|w| unsafe { w.bits(mask) });
                sio.gpio_oe_set().write(|w| unsafe { w.bits(mask) });
            }
            DriveState::Off => {
                sio.gpio_out_clr().write(|w| unsafe { w.bits(mask) });
                sio.gpio_oe_set().write(|w| unsafe { w.bits(mask) });
            }
            DriveState::Neutral => {
                sio.gpio_oe_clr().write(|w| unsafe { w.bits(mask) });
            }
        }
    }

    fn apply_state(&self, pin: u8, state: DriveState) {
        self.apply_mask_state(1u32 << pin, state);
    }
}

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
    .ok()
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

    let mut controlled_pins = CommandPins::new(CONTROL_PINS);
    let mut line_buffer = [0u8; COMMAND_BUFFER_LEN];
    let mut line_len = 0usize;

    let _ = write!(
        uart,
        "\r\nRP2040-ETH ready at {}.{}.{}.{}:{}\r\n\
P1=GP2 P2=GP3 P3=GP4 P4=GP5 P5=GP6 P6=GP7 P7=GP8 P8=GP9\r\n\
Try HELP, STATUS, P1 ON, or ALL NEUTRAL\r\n",
        NETWORK_CONFIG.local_ip[0],
        NETWORK_CONFIG.local_ip[1],
        NETWORK_CONFIG.local_ip[2],
        NETWORK_CONFIG.local_ip[3],
        NETWORK_CONFIG.local_port,
    );

    loop {
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

fn ingest_byte<P>(
    byte: u8,
    line_buffer: &mut [u8; COMMAND_BUFFER_LEN],
    line_len: &mut usize,
    controlled_pins: &mut CommandPins,
    uart: &mut UartPeripheral<rp2040_hal::uart::Enabled, pac::UART1, P>,
) where
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

fn process_line<P>(
    raw_line: &[u8],
    controlled_pins: &mut CommandPins,
    uart: &mut UartPeripheral<rp2040_hal::uart::Enabled, pac::UART1, P>,
) where
    P: rp2040_hal::uart::ValidUartPinout<pac::UART1>,
{
    let Ok(line) = core::str::from_utf8(raw_line) else {
        let _ = write!(uart, "ERR ASCII commands only\r\n");
        return;
    };

    match parse_command(line) {
        Ok(ParsedCommand::Help) => {
            let _ = write!(
                uart,
                "OK commands: HELP, STATUS, P1..P8 ON|OFF|NEUTRAL, ALL ON|OFF|NEUTRAL\r\n"
            );
        }
        Ok(ParsedCommand::Status) => {
            let _ = write!(
                uart,
                "STATUS GP2={} GP3={} GP4={} GP5={} GP6={} GP7={} GP8={} GP9={}\r\n",
                controlled_pins.states[0].label(),
                controlled_pins.states[1].label(),
                controlled_pins.states[2].label(),
                controlled_pins.states[3].label(),
                controlled_pins.states[4].label(),
                controlled_pins.states[5].label(),
                controlled_pins.states[6].label(),
                controlled_pins.states[7].label(),
            );
        }
        Ok(ParsedCommand::SetOne { slot, state }) => {
            controlled_pins.set(slot, state);
            let _ = write!(
                uart,
                "OK {} {}\r\n",
                pin_label(slot),
                controlled_pins.states[slot].label(),
            );
        }
        Ok(ParsedCommand::SetAll(state)) => {
            controlled_pins.set_all(state);
            let _ = write!(uart, "OK ALL {}\r\n", state.label());
        }
        Err(message) => {
            let _ = write!(uart, "ERR {message}\r\n");
        }
    }
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
    if token.eq_ignore_ascii_case("P1")
        || token.eq_ignore_ascii_case("1")
        || token.eq_ignore_ascii_case("GP2")
    {
        return Ok(0);
    }

    if token.eq_ignore_ascii_case("P2")
        || token.eq_ignore_ascii_case("2")
        || token.eq_ignore_ascii_case("GP3")
    {
        return Ok(1);
    }

    if token.eq_ignore_ascii_case("P3")
        || token.eq_ignore_ascii_case("3")
        || token.eq_ignore_ascii_case("GP4")
    {
        return Ok(2);
    }

    if token.eq_ignore_ascii_case("P4")
        || token.eq_ignore_ascii_case("4")
        || token.eq_ignore_ascii_case("GP5")
    {
        return Ok(3);
    }

    if token.eq_ignore_ascii_case("P5")
        || token.eq_ignore_ascii_case("5")
        || token.eq_ignore_ascii_case("GP6")
    {
        return Ok(4);
    }

    if token.eq_ignore_ascii_case("P6")
        || token.eq_ignore_ascii_case("6")
        || token.eq_ignore_ascii_case("GP7")
    {
        return Ok(5);
    }

    if token.eq_ignore_ascii_case("P7")
        || token.eq_ignore_ascii_case("7")
        || token.eq_ignore_ascii_case("GP8")
    {
        return Ok(6);
    }

    if token.eq_ignore_ascii_case("P8")
        || token.eq_ignore_ascii_case("8")
        || token.eq_ignore_ascii_case("GP9")
    {
        return Ok(7);
    }

    Err("unknown pin, use P1..P8 or GP2..GP9")
}

fn parse_state(token: &str) -> Result<DriveState, &'static str> {
    if token.eq_ignore_ascii_case("ON")
        || token.eq_ignore_ascii_case("HIGH")
        || token.eq_ignore_ascii_case("1")
    {
        return Ok(DriveState::On);
    }

    if token.eq_ignore_ascii_case("OFF")
        || token.eq_ignore_ascii_case("LOW")
        || token.eq_ignore_ascii_case("0")
    {
        return Ok(DriveState::Off);
    }

    if token.eq_ignore_ascii_case("NEUTRAL")
        || token.eq_ignore_ascii_case("FLOAT")
        || token.eq_ignore_ascii_case("Z")
        || token.eq_ignore_ascii_case("HI-Z")
    {
        return Ok(DriveState::Neutral);
    }

    Err("unknown state, use ON, OFF, or NEUTRAL")
}

fn pin_label(slot: usize) -> &'static str {
    match slot {
        0 => "GP2",
        1 => "GP3",
        2 => "GP4",
        3 => "GP5",
        4 => "GP6",
        5 => "GP7",
        6 => "GP8",
        7 => "GP9",
        _ => "UNKNOWN",
    }
}
