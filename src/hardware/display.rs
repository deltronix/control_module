use super::hal::gpio::{Output, Pin};
use super::hal::spi::Spi2;
use display_interface_spi::SPIInterface;
use st7565::{
    displays::DOGL128_6_EXT12V, modes::GraphicsMode, GraphicsPageBuffer, ST7565,
};
pub type PageBufferType = GraphicsPageBuffer<132, 8>;
pub type DisplayType = ST7565<
    SPIInterface<Spi2, Pin<'B', 11, Output>, Pin<'B', 12, Output>>,
    DOGL128_6_EXT12V,
    GraphicsMode<'static, 132, 8>,
    132,
    64,
    8,
>;
