#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use control_module as _; // global logger + panicking-behavior + memory layout
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    geometry::Size,
    mono_font::{ascii::FONT_8X13, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::String;
use embedded_hal::spi::MODE_3;
use rtic::app;
use rtic_monotonics::systick::*;
use rtic_monotonics::systick::fugit::TimerInstantU32;
use rtic_monotonics::Monotonic;
use st7565::{displays::DOGL128_6_EXT12V, GraphicsMode, GraphicsPageBuffer, ST7565};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::{spi::Spi, spi::Spi2};
use stm32f4xx_hal::gpio::{Output, Pin};
use embedded_graphics::mono_font::iso_8859_1::FONT_9X15;
use embedded_graphics::mono_font::iso_8859_10::FONT_5X7;
use core::fmt::Write;
#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {
    use super::*;
    type DisplayType = ST7565<
        SPIInterface<Spi2, Pin<'B', 11, Output>, Pin<'B', 12, Output>>,
        DOGL128_6_EXT12V,
        GraphicsMode<'static, 132, 8>,
        132,
        64,
        8,
    >;
    #[shared]
    struct Shared {
        disp: &'static mut DisplayType,
    }

    // Local resources go here
    #[local]
    struct Local {
        t_init: TimerInstantU32<1_000>,
    }


    #[init(local = [
        disp: Option<DisplayType> = None,
        page_buffer: Option<GraphicsPageBuffer<132,8>> = None
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let mut rcc = cx.device.RCC.constrain();
        let ccdr = rcc.cfgr.sysclk(48.MHz()).freeze();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token);


        let (
            mut spi2_disp_rst,
            mut spi2_disp_a0,
            mut spi2_disp_cs,
            spi2_sck,
            spi2_miso,
            spi2_mosi,
            mut disp_backlight,
        ) = {
            let gpiob = cx.device.GPIOB.split();
            (
                gpiob.pb10.into_push_pull_output(),
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
        let disp_spi = SPIInterface::new(
            Spi::new(
                cx.device.SPI2,
                (spi2_sck, spi2_miso, spi2_mosi),
                MODE_3,
                1.MHz(),
                &ccdr,
            ),
            spi2_disp_a0,
            spi2_disp_cs,
        );

        let mut timer = cx.device.TIM2.delay_us(&ccdr);

        let page_buffer = cx.local.page_buffer.insert(GraphicsPageBuffer::new());
        let mut display = ST7565::new(disp_spi, DOGL128_6_EXT12V).into_graphics_mode(page_buffer);
        display.reset(&mut spi2_disp_rst, &mut timer).unwrap();
        display.flush().unwrap();
        display.set_display_on(true).unwrap();
        let display = cx.local.disp.insert(display);

        display.clear(BinaryColor::Off).unwrap();
        // Send content to display

        render::spawn().ok();
        update::spawn().ok();
        
        (
            Shared { disp: display }, 
            Local { t_init: Systick::now()})
    }

    // Optional idle, can be removed if not needed.
    //
    #[task(priority = 1)]
    async fn update(_: update::Context){
        loop{
            task1::spawn().ok();
            Systick::delay(1000.millis()).await;
            task2::spawn().ok();
            Systick::delay(1000.millis()).await;
        }
    }
    #[task(priority = 0, shared=[disp])]
    async fn render(mut cx: render::Context) {
        loop {
            cx.shared.disp.lock(|disp|{
                disp.flush().unwrap();
            });
            Systick::delay(20.millis()).await;
        } 
    }
    #[task(priority = 1, shared=[disp])]
    async fn task1(mut cx: task1::Context) {
        defmt::info!("task 1");
        let font = MonoTextStyle::new(&FONT_8X13, BinaryColor::On);
        cx.shared.disp.lock(|disp| {
            disp.clear(BinaryColor::Off).unwrap();
            Text::new("Hello, \nfrom task1!", Point::new(4, 13), font)
                .draw(*disp)
                .unwrap();
        });
        uptime_counter::spawn().ok();
    }

    #[task(priority = 1, shared = [disp])]
    async fn task2(mut cx: task2::Context){
        defmt::info!("task 2");
        let font = MonoTextStyle::new(&FONT_9X15, BinaryColor::On);
        cx.shared.disp.lock(|disp| {
            disp.clear(BinaryColor::Off).unwrap();
            Text::new("Hello, \nfrom task2!", Point::new(4, 13), font)
                .draw(*disp)
                .unwrap();
        });
        uptime_counter::spawn().ok();
    }
    #[task(priority = 1, shared = [disp], local = [t_init])]
    async fn uptime_counter(mut cx: uptime_counter::Context){
        let t = Systick::now();
        let uptime = t.checked_duration_since(*cx.local.t_init).unwrap();
        let fill = PrimitiveStyle::with_fill(BinaryColor::On);
        let font = MonoTextStyle::new(&FONT_5X7, BinaryColor::Off);
        let mut text: String<16> = String::new();
        write!(text, "UPTIME: {:02}:{:02}:{:02}", uptime.to_hours(), uptime.to_minutes() % 60, uptime.to_secs() % 60).unwrap();
        cx.shared.disp.lock(|disp|{
            Rectangle::new(Point::new(0, 56),Size::new(132,8)).into_styled(fill).draw(*disp).unwrap();
            Text::new(text.as_str(), Point::new(16, 62), font)
                .draw(*disp)
                .unwrap();
        });
    }
}


