use core::ptr::addr_of_mut;
use cortex_m::peripheral::nvic;
use display_interface_spi::SPIInterface;
use embedded_hal::spi::{MODE_0, MODE_3};
use hal::gpio::PinState;
use hal::prelude::*;
use hal::spi::Spi;
use hal::timer::Counter;
use st7565::displays::DOGL128_6_EXT12V;
use st7565::{GraphicsPageBuffer, ST7565};
use embedded_graphics::prelude::*;
use embedded_graphics::pixelcolor::BinaryColor;
use stm32f4xx_hal as hal;

pub(crate) mod debounce;
pub mod display;
pub mod project;
pub mod switches;
pub mod ui;
pub mod timer;
use switches::Switches;

use self::display::PageBufferType;
use self::ui::UI;

pub struct Hardware {
    pub ui: UI,
}

pub struct Peripherals {
    pub gpiob: hal::pac::GPIOB,
    pub spi1: hal::pac::SPI1,
    pub spi2: hal::pac::SPI2,
    pub tim2: hal::pac::TIM2,
    pub tim14: hal::pac::TIM14
}

static mut DISPLAY_BUFFER: PageBufferType = GraphicsPageBuffer::new();

pub fn setup(mut peripherals: Peripherals, clock: hal::pac::RCC) -> Hardware {
    clock.apb1enr.write(|w|w.tim2en().enabled());
    let rcc = clock.constrain();
    let ccdr = rcc.cfgr.sysclk(168.MHz()).freeze();
    ccdr.timclk1();

    let gpiob = peripherals.gpiob.split();
    let (spi1_sclk, spi1_miso, spi1_mosi, spi1_rclk_ld) = {
        (
            gpiob.pb3.into_alternate(),
            gpiob.pb4.into_alternate(),
            gpiob.pb5.into_alternate(),
            gpiob.pb6.into_open_drain_output_in_state(PinState::High),
        )
    };
    let spi1 = Spi::new(
        peripherals.spi1,
        (spi1_sclk, spi1_miso, spi1_mosi),
        MODE_0,
        1.MHz(),
        &ccdr,
    );
    let (
        mut spi2_disp_rst,
        mut spi2_disp_a0,
        mut spi2_disp_cs,
        spi2_sck,
        spi2_miso,
        spi2_mosi,
        mut disp_backlight,
    ) = (
        gpiob.pb10.into_push_pull_output_in_state(PinState::High),
        gpiob.pb11.into_push_pull_output_in_state(PinState::High),
        gpiob.pb12.into_push_pull_output_in_state(PinState::High),
        gpiob.pb13.into_alternate(),
        gpiob.pb14.into_alternate(),
        gpiob.pb15.into_alternate(),
        gpiob.pb1.into_push_pull_output_in_state(PinState::High),
    );
    let disp_spi = SPIInterface::new(
        Spi::new(
            peripherals.spi2,
            (spi2_sck, spi2_miso, spi2_mosi),
            MODE_3,
            1.MHz(),
            &ccdr,
        ),
        spi2_disp_a0,
        spi2_disp_cs,
    );

    let mut delay = peripherals.tim14.delay_us(&ccdr);

    let mut display =
        unsafe { ST7565::new(disp_spi, DOGL128_6_EXT12V).into_graphics_mode(&mut DISPLAY_BUFFER) };
    display.reset(&mut spi2_disp_rst, &mut delay).unwrap();
    display.flush().unwrap();
    display.set_display_on(true).unwrap();
    display.clear(BinaryColor::Off).unwrap();
    Hardware {
        ui: UI {
            switches: Switches::new(spi1, spi1_rclk_ld),
            display,
        },
    }
}
