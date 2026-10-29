use crate::Error;

/// Bluetooth LE channel index, 0 through 39.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Channel(u8);

impl Channel {
    pub const ADV_37: Self = Self(37);
    pub const ADV_38: Self = Self(38);
    pub const ADV_39: Self = Self(39);

    pub const fn new(index: u8) -> Result<Self, Error> {
        if index <= 39 {
            Ok(Self(index))
        } else {
            Err(Error::InvalidChannel)
        }
    }

    pub const fn index(self) -> u8 {
        self.0
    }

    /// RF center frequency in MHz, per Bluetooth Core channel mapping.
    pub const fn frequency_mhz(self) -> u16 {
        match self.0 {
            37 => 2402,
            38 => 2426,
            39 => 2480,
            0..=10 => 2404 + self.0 as u16 * 2,
            11..=36 => 2428 + (self.0 as u16 - 11) * 2,
            _ => unreachable!(),
        }
    }
}
