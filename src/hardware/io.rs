
use ad57xx::{Ad57xx_exclusive, Ad57xx_shared};
use embedded_hal::{digital::OutputPin, spi::{SpiBus, SpiDevice}};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use stm32f4xx_hal::{gpio::{Output, Pin}, spi::Spi3};

pub struct IO<DEV, RCLK>{
    pub dac: Ad57xx_shared<DEV>,
    sr: [u8; 2],
    rclk: RCLK,
}

impl<DEV, RCLK> IO<DEV, RCLK> where
    DEV: SpiDevice,
    RCLK: OutputPin,
{
        pub fn new(spi: DEV, rclk: RCLK) -> Self {
            Self{
                dac: Ad57xx_shared::new(spi),
                sr: [0x00; 2],
                rclk,
            }

        }

}
