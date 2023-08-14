#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use control_module as _; // global logger + panicking-behavior + memory layout
use stm32f4xx_hal::prelude::*;
use rtic::app;
use embedded_hal::spi::{MODE_3};
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    geometry::Size,
    mono_font::{ascii::FONT_8X13, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle},
    text::Text,
};
use st7565::{displays::DOGL128_6_EXT12V, GraphicsPageBuffer, ST7565, GraphicsMode};
use stm32f4xx_hal::{
    spi::Spi,
    spi::Spi2,
    gpio::{PB11, PB12},
};
use rtic_monotonics::systick::*;
use heapless::pool::Box;

#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {

    use stm32f4xx_hal::gpio::{Pin, Output};

    use super::*;
    //use stm32f4xx_hal::gpio::gpiob::{PB10, PB11, PB12, PB13, PB14, PB15};
    // Shared resources go here
    //
    //
    type DisplayType = ST7565<
        SPIInterface<Spi2,Pin<'B',11, Output>,Pin<'B',12, Output>>,
        DOGL128_6_EXT12V,
        GraphicsMode<'static, 128, 8>,
        128,64,8>;
    #[shared]
    struct Shared {
    }

    // Local resources go here
    #[local]
    struct Local {
        disp: &'static DisplayType,
        // TODO: Add resources
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        defmt::info!("RCC constrain");

        let mut rcc = cx.device.RCC.constrain();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token); 

        let ccdr = rcc.cfgr.sysclk(48.MHz()).freeze();
        // defmt::info!("RCC constrain")
        //let syscfg = cx.device.SYSCFG.constrain();

        defmt::info!("gpio setup");
        let (mut spi2_disp_rst, mut spi2_disp_a0, mut spi2_disp_cs, spi2_sck, spi2_miso, spi2_mosi, mut disp_backlight) = {
            let gpiob = cx.device.GPIOB.split();
            (gpiob.pb10.into_push_pull_output(), 
             gpiob.pb11.into_push_pull_output(), 
             gpiob.pb12.into_push_pull_output(), 
             gpiob.pb13.into_alternate(), 
             gpiob.pb14.into_alternate(), 
             gpiob.pb15.into_alternate(),
             gpiob.pb1.into_push_pull_output(),
             )
        };
        disp_backlight.set_high();
        let mut spi2_flash_cs = {
            let gpioc = cx.device.GPIOC.split();
            gpioc.pc6.into_push_pull_output()
        };

        spi2_flash_cs.set_high();
        spi2_disp_rst.set_high();
        spi2_disp_a0.set_high();
        spi2_disp_cs.set_high();


        defmt::info!("display init");
        let disp_spi = 
            SPIInterface::new(
            Spi::new(
                cx.device.SPI2,
                (spi2_sck, spi2_miso, spi2_mosi),
                MODE_3,
                1.MHz(),
                &ccdr
            ),
            spi2_disp_a0,
            spi2_disp_cs,
            );

        let mut timer = cx.device.TIM2.delay_us(&ccdr);

        let mut page_buffer = GraphicsPageBuffer::new();
        let mut disp = ST7565::new(disp_spi, DOGL128_6_EXT12V).into_graphics_mode(&mut page_buffer);
        disp.reset(&mut spi2_disp_rst, &mut timer).unwrap();
        disp.flush().unwrap();
        disp.set_display_on(true).unwrap();

        Circle::new(Point::new(6, 6), 20)
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
            .draw(&mut disp)
            .unwrap();
        Rectangle::new(Point::new(106, 6), Size::new(20, 20))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
            .draw(&mut disp)
            .unwrap();
        let font = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);
        Text::new("Hello,\nRust!", Point::new(43, 13), font)
            .draw(&mut disp)
            .unwrap();

        // Send content to display
        task1::spawn().ok();

        (
            Shared {
            },
            Local {
                // Initialization of local resources go here
                disp: &disp,
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    // TODO: Add tasks
    #[task(priority = 1)]
    async fn task1(_cx: task1::Context) {
        defmt::info!("Hello from task1!");
    }
}

