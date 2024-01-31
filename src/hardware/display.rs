use display_interface_spi::SPIInterface;
use embedded_hal::spi::MODE_3;
use st7565::{displays::DOGL128_6_EXT12V, GraphicsMode, GraphicsPageBuffer, ST7565};
use super::hal::spi::Spi2;
use super::hal::gpio::{Pin, Output};
pub type PageBufferType = GraphicsPageBuffer<132,8>;
pub type DisplayType = ST7565<
        SPIInterface<Spi2, Pin<'B', 11, Output>, Pin<'B', 12, Output>>,
        DOGL128_6_EXT12V,
        GraphicsMode<'static, 132, 8>,
        132,
        64,
        8,
    >;


