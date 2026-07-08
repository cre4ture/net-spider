use crate::{DriveState, EXPANDER_SLOT_COUNT};
use embedded_hal::i2c::{Error as _, ErrorKind, I2c, SevenBitAddress};

const REG_IODIRA: u8 = 0x00;
const REG_GPPUA: u8 = 0x0C;
const REG_GPIOA: u8 = 0x12;

#[derive(Clone, Copy)]
pub struct Mcp23017 {
    address: u8,
    states: [DriveState; EXPANDER_SLOT_COUNT],
}

impl Mcp23017 {
    pub const fn placeholder() -> Self {
        Self {
            address: 0,
            states: [DriveState::HiZ; EXPANDER_SLOT_COUNT],
        }
    }

    pub const fn address(&self) -> u8 {
        self.address
    }

    pub const fn states(&self) -> &[DriveState; EXPANDER_SLOT_COUNT] {
        &self.states
    }

    pub fn probe<I2C>(i2c: &mut I2C, address: u8) -> Result<bool, I2C::Error>
    where
        I2C: I2c<SevenBitAddress>,
    {
        let mut registers = [0u8; 2];

        match i2c.write_read(address, &[REG_IODIRA], &mut registers) {
            Ok(()) => Ok(true),
            Err(error) => match error.kind() {
                ErrorKind::NoAcknowledge(_) => Ok(false),
                _ => Err(error),
            },
        }
    }

    pub fn new<I2C>(i2c: &mut I2C, address: u8) -> Result<Self, I2C::Error>
    where
        I2C: I2c<SevenBitAddress>,
    {
        let controller = Self {
            address,
            states: [DriveState::HiZ; EXPANDER_SLOT_COUNT],
        };

        controller.write_register16(i2c, REG_GPPUA, 0)?;
        controller.sync(i2c)?;
        Ok(controller)
    }

    pub fn set_pin_state<I2C>(
        &mut self,
        i2c: &mut I2C,
        pin: usize,
        state: DriveState,
    ) -> Result<(), I2C::Error>
    where
        I2C: I2c<SevenBitAddress>,
    {
        self.states[pin] = state;
        self.sync(i2c)
    }

    pub fn set_all_state<I2C>(&mut self, i2c: &mut I2C, state: DriveState) -> Result<(), I2C::Error>
    where
        I2C: I2c<SevenBitAddress>,
    {
        self.states = [state; EXPANDER_SLOT_COUNT];
        self.sync(i2c)
    }

    fn sync<I2C>(&self, i2c: &mut I2C) -> Result<(), I2C::Error>
    where
        I2C: I2c<SevenBitAddress>,
    {
        let (direction, output) = self.encode();
        self.write_register16(i2c, REG_GPIOA, output)?;
        self.write_register16(i2c, REG_IODIRA, direction)
    }

    fn encode(&self) -> (u16, u16) {
        let mut direction = 0u16;
        let mut output = 0u16;

        for (pin, state) in self.states.iter().copied().enumerate() {
            let mask = 1u16 << pin;

            match state {
                DriveState::High => output |= mask,
                DriveState::Low => {}
                DriveState::HiZ => direction |= mask,
            }
        }

        (direction, output)
    }

    fn write_register16<I2C>(
        &self,
        i2c: &mut I2C,
        register: u8,
        value: u16,
    ) -> Result<(), I2C::Error>
    where
        I2C: I2c<SevenBitAddress>,
    {
        let bytes = value.to_le_bytes();
        i2c.write(self.address, &[register, bytes[0], bytes[1]])
    }
}
