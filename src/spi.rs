use embedded_hal::spi::*;

use heapless::box_pool;
box_pool!(
    P: [u8;2]
);


use stm32f4xx_hal::spi::{Rx, Tx};
use stm32f4xx_hal::dma::{Transfer, Stream2, Stream3, PeripheralToMemory, MemoryToPeripheral};
use stm32f4xx_hal::gpio::*;
use stm32f4xx_hal::pac;
pub type Spi1DmaRx = Transfer<
    Stream2<pac::DMA2>,
    3,
    Rx<pac::SPI1>,
    PeripheralToMemory,
    P,
>;
pub type Spi1DmaTx = Transfer<
    Stream3<pac::DMA2>,
    3,
    Tx<pac::SPI1>,
    MemoryToPeripheral,
    P,
>;

struct Spi1Dma {
    rx: Spi1DmaRx,
    tx: Spi1DmaTx,
    rclk_ld: Pin<'B', 6, Output<OpenDrain>>,
//    rx_buf: Option<heapless::pool::singleton::Box<P>>,
//    tx_buf: Option<heapless::pool::singleton::Box<P>>,


}


