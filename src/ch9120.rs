use cortex_m::delay::Delay;
use embedded_hal::digital::OutputPin;
use rp2040_hal::fugit::{HertzU32, RateExtU32};
use rp2040_hal::pac::UART1;
use rp2040_hal::uart::{DataBits, Enabled, StopBits, UartConfig, UartPeripheral, ValidUartPinout};

const FRAME_PREFIX: [u8; 2] = [0x57, 0xAB];
const CMD_MODE: u8 = 0x10;
const CMD_LOCAL_IP: u8 = 0x11;
const CMD_SUBNET_MASK: u8 = 0x12;
const CMD_GATEWAY: u8 = 0x13;
const CMD_LOCAL_PORT: u8 = 0x14;
const CMD_TARGET_IP: u8 = 0x15;
const CMD_TARGET_PORT: u8 = 0x16;
const CMD_UART1_BAUD: u8 = 0x21;
const CMD_SAVE: u8 = 0x0D;
const CMD_APPLY: u8 = 0x0E;
const CMD_RESTART: u8 = 0x5E;

pub const CONFIG_BAUD: u32 = 9_600;

#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum NetworkMode {
    TcpServer,
    TcpClient,
    UdpServer,
    UdpClient,
}

impl NetworkMode {
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::TcpServer => 0,
            Self::TcpClient => 1,
            Self::UdpServer => 2,
            Self::UdpClient => 3,
        }
    }

    pub const fn uses_target_endpoint(self) -> bool {
        matches!(self, Self::TcpClient | Self::UdpClient)
    }
}

#[derive(Clone, Copy)]
pub struct NetworkConfig {
    pub mode: NetworkMode,
    pub local_ip: [u8; 4],
    pub subnet_mask: [u8; 4],
    pub gateway: [u8; 4],
    pub local_port: u16,
    pub target_ip: [u8; 4],
    pub target_port: u16,
    pub transport_baud: u32,
}

pub struct Ch9120<CfgPin, ResetPin> {
    cfg_pin: CfgPin,
    reset_pin: ResetPin,
}

impl<CfgPin, ResetPin> Ch9120<CfgPin, ResetPin>
where
    CfgPin: OutputPin,
    ResetPin: OutputPin,
{
    pub fn new(mut cfg_pin: CfgPin, mut reset_pin: ResetPin) -> Self {
        let _ = cfg_pin.set_high();
        let _ = reset_pin.set_high();
        Self { cfg_pin, reset_pin }
    }

    pub fn configure<P>(
        &mut self,
        uart: UartPeripheral<Enabled, UART1, P>,
        delay: &mut Delay,
        peripheral_clock: HertzU32,
        config: NetworkConfig,
    ) -> UartPeripheral<Enabled, UART1, P>
    where
        P: ValidUartPinout<UART1>,
    {
        self.enter_config(delay);

        self.write_u8(&uart, delay, CMD_MODE, config.mode.as_u8());
        self.write_bytes(&uart, delay, CMD_LOCAL_IP, &config.local_ip);
        self.write_bytes(&uart, delay, CMD_SUBNET_MASK, &config.subnet_mask);
        self.write_bytes(&uart, delay, CMD_GATEWAY, &config.gateway);
        self.write_u16(&uart, delay, CMD_LOCAL_PORT, config.local_port);

        if config.mode.uses_target_endpoint() {
            self.write_bytes(&uart, delay, CMD_TARGET_IP, &config.target_ip);
            self.write_u16(&uart, delay, CMD_TARGET_PORT, config.target_port);
        }

        self.write_u32(&uart, delay, CMD_UART1_BAUD, config.transport_baud);
        self.exit_config(&uart, delay);

        let uart = uart.disable();
        let uart = uart
            .enable(
                UartConfig::new(
                    config.transport_baud.Hz(),
                    DataBits::Eight,
                    None,
                    StopBits::One,
                ),
                peripheral_clock,
            )
            .expect("transport baud should be supported");

        Self::drain_rx(&uart);
        uart
    }
    fn enter_config(&mut self, delay: &mut Delay) {
        let _ = self.reset_pin.set_high();
        let _ = self.cfg_pin.set_low();
        delay.delay_ms(500);
    }

    fn exit_config<P>(&mut self, uart: &UartPeripheral<Enabled, UART1, P>, delay: &mut Delay)
    where
        P: ValidUartPinout<UART1>,
    {
        self.send_command_only(uart, delay, CMD_SAVE);
        delay.delay_ms(200);
        self.send_command_only(uart, delay, CMD_APPLY);
        delay.delay_ms(200);
        self.send_command_only(uart, delay, CMD_RESTART);
        delay.delay_ms(500);
        let _ = self.cfg_pin.set_high();
    }

    fn write_u8<P>(
        &self,
        uart: &UartPeripheral<Enabled, UART1, P>,
        delay: &mut Delay,
        command: u8,
        value: u8,
    ) where
        P: ValidUartPinout<UART1>,
    {
        self.send_frame(uart, delay, command, &[value]);
    }

    fn write_u16<P>(
        &self,
        uart: &UartPeripheral<Enabled, UART1, P>,
        delay: &mut Delay,
        command: u8,
        value: u16,
    ) where
        P: ValidUartPinout<UART1>,
    {
        self.send_frame(uart, delay, command, &value.to_le_bytes());
    }

    fn write_u32<P>(
        &self,
        uart: &UartPeripheral<Enabled, UART1, P>,
        delay: &mut Delay,
        command: u8,
        value: u32,
    ) where
        P: ValidUartPinout<UART1>,
    {
        self.send_frame(uart, delay, command, &value.to_le_bytes());
    }

    fn write_bytes<P>(
        &self,
        uart: &UartPeripheral<Enabled, UART1, P>,
        delay: &mut Delay,
        command: u8,
        payload: &[u8; 4],
    ) where
        P: ValidUartPinout<UART1>,
    {
        self.send_frame(uart, delay, command, payload);
    }

    fn send_command_only<P>(
        &self,
        uart: &UartPeripheral<Enabled, UART1, P>,
        delay: &mut Delay,
        command: u8,
    ) where
        P: ValidUartPinout<UART1>,
    {
        self.send_frame(uart, delay, command, &[]);
    }

    fn send_frame<P>(
        &self,
        uart: &UartPeripheral<Enabled, UART1, P>,
        delay: &mut Delay,
        command: u8,
        payload: &[u8],
    ) where
        P: ValidUartPinout<UART1>,
    {
        let mut frame = [0u8; 7];
        frame[..2].copy_from_slice(&FRAME_PREFIX);
        frame[2] = command;

        for (slot, value) in frame[3..].iter_mut().zip(payload.iter().copied()) {
            *slot = value;
        }

        let frame_len = 3 + payload.len();
        delay.delay_ms(10);
        uart.write_full_blocking(&frame[..frame_len]);
        delay.delay_ms(10);
    }

    fn drain_rx<P>(uart: &UartPeripheral<Enabled, UART1, P>)
    where
        P: ValidUartPinout<UART1>,
    {
        let mut scratch = [0u8; 16];
        while uart.uart_is_readable() {
            if uart.read_raw(&mut scratch).is_err() {
                break;
            }
        }
    }
}
