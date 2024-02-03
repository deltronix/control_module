use core::cell::RefCell;

use display_interface_spi::SPIInterface;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_hal::spi::{SpiBus, SpiDevice, MODE_0, MODE_2, MODE_3};
use embedded_hal_bus::spi::{RefCellDevice, NoDelay};
use embedded_hal_bus::spi::DeviceError;



use hal::gpio::{Edge, ExtiPin, Input, Output, Pin, PinState};
use hal::pac::SPI3;
use hal::prelude::*;
use hal::spi::{Spi, Spi3};
use st7565::displays::DOGL128_6_EXT12V;
use st7565::{GraphicsPageBuffer, ST7565};
use stm32f4xx_hal as hal;

pub(crate) mod debounce;
pub mod display;
pub mod project;
pub mod switches;
pub mod timer;
pub mod ui;
use switches::Switches;

use self::display::PageBufferType;
use self::ui::UI;

pub struct Hardware {
    pub ui: UI,
    pub clk_in: Pin<'A', 8, Input>,
    pub clk_out: Pin<'C', 15, Output>,
    pub spi3: Spi3,
    //pub io: IO<RefCellDevice<'static, Spi3, Pin<'A', 15, Output>, NoDelay>, 
    //            Pin<'D', 2, Output>>,
    pub io_sync: Pin<'A', 15, Output>,
    pub io_rclk: Pin<'D', 2, Output>
}

static mut DISPLAY_BUFFER: PageBufferType = GraphicsPageBuffer::new();

pub fn setup(peripherals: hal::pac::Peripherals) -> Hardware {
    peripherals.RCC.apb1enr.write(|w| w.tim2en().enabled());
    let rcc = peripherals.RCC.constrain();
    let ccdr = rcc.cfgr.sysclk(168.MHz()).freeze();
    let mut syscfg = peripherals.SYSCFG.constrain();
    let mut exti = peripherals.EXTI;

    let gpioa = peripherals.GPIOA.split();
    let mut clk_in = gpioa.pa8.into_input();
    clk_in.make_interrupt_source(&mut syscfg);
    clk_in.enable_interrupt(&mut exti);
    clk_in.trigger_on_edge(&mut exti, Edge::Rising);

    let gpioc = peripherals.GPIOC.split();
    let clk_out = gpioc.pc15.into_push_pull_output_in_state(PinState::Low);
    let spi3_sclk = gpioc.pc10.into_alternate();
    let spi3_miso = gpioc.pc11.into_alternate();
    let spi3_mosi = gpioc.pc12.into_alternate();

    let gpiod = peripherals.GPIOD.split();
    let spi3_rclk = gpiod.pd2.into_push_pull_output_in_state(PinState::Low);
    let spi3_sync = gpioa.pa15.into_push_pull_output_in_state(PinState::High);

    let gpiob = peripherals.GPIOB.split();
    let spi1_sclk = gpiob.pb3.into_alternate();
    let spi1_miso = gpiob.pb4.into_alternate();
    let spi1_mosi = gpiob.pb5.into_alternate();
    let spi1_rclk_ld = gpiob.pb6.into_open_drain_output_in_state(PinState::High);

    let mut spi2_disp_rst = gpiob.pb10.into_push_pull_output_in_state(PinState::High);
    let mut spi2_disp_a0 = gpiob.pb11.into_push_pull_output_in_state(PinState::High);
    let mut spi2_disp_cs = gpiob.pb12.into_push_pull_output_in_state(PinState::High);
    let spi2_sck = gpiob.pb13.into_alternate();
    let spi2_miso = gpiob.pb14.into_alternate();
    let spi2_mosi = gpiob.pb15.into_alternate();
    let mut disp_backlight = gpiob.pb1.into_push_pull_output_in_state(PinState::High);

    let spi1 = Spi::new(
        peripherals.SPI1,
        (spi1_sclk, spi1_miso, spi1_mosi),
        MODE_0,
        1.MHz(),
        &ccdr,
    );
    let spi2 = SPIInterface::new(
        Spi::new(
            peripherals.SPI2,
            (spi2_sck, spi2_miso, spi2_mosi),
            MODE_3,
            1.MHz(),
            &ccdr,
        ),
        spi2_disp_a0,
        spi2_disp_cs,
    );

    let spi3 = Spi::new(
        peripherals.SPI3,
        (spi3_sclk, spi3_miso, spi3_mosi),
        MODE_2,
        1.MHz(),
        &ccdr,
    );

    //let dev1 = RefCellDevice::new(&spi_bus, spi3_sync, NoDelay);
    /*
    let mut io = IO::new(dev1, spi3_rclk);

    io.dac
        .set_output_range(ad57xx::Channel::AllDacs, ad57xx::OutputRange::Bipolar5V)
        .unwrap();
    io.dac.set_power(ad57xx::Channel::AllDacs, true).unwrap();
    io.dac
        .set_dac_output(ad57xx::Channel::DacA, 0x8000)
        .unwrap();
    io.dac
        .set_dac_output(ad57xx::Channel::DacB, 0x0000)
        .unwrap();
    io.dac
        .set_dac_output(ad57xx::Channel::DacC, 0xFFFF)
        .unwrap();
    */
    let mut delay = peripherals.TIM14.delay_us(&ccdr);

    let mut display =
        unsafe { ST7565::new(spi2, DOGL128_6_EXT12V).into_graphics_mode(&mut DISPLAY_BUFFER) };
    display.reset(&mut spi2_disp_rst, &mut delay).unwrap();
    display.flush().unwrap();
    display.set_display_on(true).unwrap();
    display.clear(BinaryColor::Off).unwrap();
    Hardware {
        ui: UI {
            switches: Switches::new(spi1, spi1_rclk_ld),
            display,
        },
        clk_in,
        clk_out,
        spi3,
        io_sync: spi3_sync,
        io_rclk: spi3_rclk,
    }
}
