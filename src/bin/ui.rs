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
use heapless::{String, pool, pool::singleton::*};
use embedded_hal::spi::*;
use rtic::app;
use rtic_monotonics::systick::*;
use rtic_monotonics::systick::fugit::{Instant};
use rtic_monotonics::Monotonic;
use st7565::{displays::DOGL128_6_EXT12V, GraphicsMode, GraphicsPageBuffer, ST7565};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::{
    spi::{Spi, Spi1, Spi2, Rx, Tx},
    gpio::*,
    dma::*,
    pac,
    interrupt::*,
};
use stm32f4xx_hal::dma::config::DmaConfig;
use embedded_graphics::mono_font::iso_8859_1::FONT_9X15;
use embedded_graphics::mono_font::iso_8859_10::FONT_5X7;
use core::fmt::Write;
pool!(
    #[link_section = ".ccmram.A"]
    P: [u8;2]
);
type DisplayType = ST7565<
    SPIInterface<Spi2, Pin<'B', 11, Output>, Pin<'B', 12, Output>>,
    DOGL128_6_EXT12V,
    GraphicsMode<'static, 132, 8>,
    132,
    64,
    8,
>;
const SPI1_BUF_SIZE: usize = 2;
type Spi1DmaRx = Transfer<
    Stream2<pac::DMA2>,
    3,
    Rx<pac::SPI1>,
    PeripheralToMemory,
    heapless::pool::singleton::Box<P>,
>;
type Spi1DmaTx = Transfer<
    Stream3<pac::DMA2>,
    3,
    Tx<pac::SPI1>,
    MemoryToPeripheral,
    heapless::pool::singleton::Box<P>,
>;
#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {

    use heapless::pool::Init;

    use super::*;
    #[shared]
    struct Shared {
        disp: &'static mut DisplayType,
        fps: f32,
        spi1_rx_xfer: Spi1DmaRx,
        spi1_tx_xfer: Spi1DmaTx,
        spi1_rx_buf: Option<heapless::pool::singleton::Box<P>>,
        spi1_tx_buf: Option<heapless::pool::singleton::Box<P>>,
        spi1_rclk_ld: Pin<'B', 6, Output<OpenDrain>>,
    }
    #[local]
    struct Local {
        pos: i32,
        last_disp_update: Instant<u32, 1, 1000>,
        // TODO: Add resources
    }
    #[init(local = [
        disp: Option<DisplayType> = None,
        page_buffer: Option<GraphicsPageBuffer<132,8>> = None,
        //spi1_rx_xfer: Option<Spi1DmaRx> = None,
        //spi1_tx_xfer: Option<Spi1DmaTx> = None,
        spi1_rx_buf: Option<heapless::pool::singleton::Box<P, Init>> = None,
        spi1_tx_buf: Option<heapless::pool::singleton::Box<P, Init>> = None,
        mem: [u8; 32] = [0; 32],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let mut rcc = cx.device.RCC.constrain();
        let ccdr = rcc.cfgr.sysclk(48.MHz()).freeze();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 48_000_000, systick_mono_token);

        let gpiob = cx.device.GPIOB.split();

        // Initialize SPI1
        let ( ui_sclk, ui_miso, ui_mosi, mut ui_rclk_ld) = {(
                gpiob.pb3.into_alternate(),
                gpiob.pb4.into_alternate(),
                gpiob.pb5.into_alternate(),
                gpiob.pb6.into_open_drain_output(),
        )};
        ui_rclk_ld.set_high();
        ui_rclk_ld.set_speed(Speed::High);
        let mut spi1 = Spi::new(cx.device.SPI1, (ui_sclk, ui_miso, ui_mosi), MODE_3, 1.MHz(), &ccdr);
        
        let (spi1_tx, spi1_rx) = spi1.use_dma().txrx();
        let streams = StreamsTuple::new(cx.device.DMA2);
        let spi1_rx_stream = streams.2;
        let spi1_tx_stream = streams.3;

        P::grow(cx.local.mem);
        let spi1_rx_buf: Box<P> = P::alloc().unwrap().init([0x0; 2]);
        let spi1_tx_buf: Box<P> = P::alloc().unwrap().init([0x0; 2]);

        let mut spi1_rx_xfer = Transfer::init_peripheral_to_memory(
            spi1_rx_stream,
            spi1_rx,
            spi1_rx_buf,
            None,
            DmaConfig::default()
            .memory_increment(true)
            .fifo_enable(true)
            .fifo_error_interrupt(true)
            .transfer_complete_interrupt(true)
        );
        let mut spi1_tx_xfer = Transfer::init_memory_to_peripheral(
            spi1_tx_stream,
            spi1_tx,
            spi1_tx_buf,
            None,
            DmaConfig::default()
            .memory_increment(true)
            .fifo_enable(true)
            .fifo_error_interrupt(true)
            .transfer_complete_interrupt(true)
        );

        //let spi1_rx_transfer = cx.local.spi1_rx_xfer.insert(spi1_rx_xfer);
        //let spi1_tx_transfer = cx.local.spi1_tx_xfer.insert(spi1_tx_xfer);
       
        // Initialize SPI2
        let mut spi2_disp_rst = gpiob.pb10.into_push_pull_output();
        let mut spi2_disp_a0 = gpiob.pb11.into_push_pull_output();
        let mut spi2_disp_cs = gpiob.pb12.into_push_pull_output();
        let spi2_sck = gpiob.pb13.into_alternate();
        let spi2_miso = gpiob.pb14.into_alternate();
        let spi2_mosi = gpiob.pb15.into_alternate();
        let mut disp_backlight =  gpiob.pb1.into_push_pull_output();

        let mut spi2_flash_cs = {
            let gpioc = cx.device.GPIOC.split();
            gpioc.pc6.into_push_pull_output()
        };
        spi2_flash_cs.set_high();
        spi2_disp_rst.set_high();
        spi2_disp_a0.set_high();
        spi2_disp_cs.set_high();
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
        disp_backlight.set_high();
        // Send content to display

        spi1_rx_xfer.start(|_rx|{});
        spi1_tx_xfer.start(|_tx|{});

        render::spawn().ok();
        update::spawn().ok();
        
        (
            Shared { 
                disp: display, 
                fps: 0.0, 
                spi1_rx_xfer,
                spi1_tx_xfer,
                spi1_rx_buf: None,
                spi1_tx_buf: None,
                spi1_rclk_ld: ui_rclk_ld,
              }, 
            Local { 
                pos: 0,
                last_disp_update: Systick::now()
        })
    }

    // Optional idle, can be removed if not needed.
    //
    #[task(priority = 1)]
    async fn update(_: update::Context){
        loop{
            task1::spawn().ok();
            Systick::delay(200.millis()).await;
            task2::spawn().ok();
            Systick::delay(200.millis()).await;
        }
    }
    #[task(priority = 1, shared=[disp, fps], local=[last_disp_update])]
    async fn render(mut cx: render::Context) {
        
        loop {
            let mut text: String<16> = String::new();
            let t: Instant<u32, 1, 1000> = Systick::now();
            let t_dif = t.checked_duration_since(*cx.local.last_disp_update);
            match t_dif {
                Some(t_dif) => {
                    cx.shared.fps.lock(|fps|{

                        *fps = 1000.0 / t_dif.to_millis() as f32;
                    }); 

                    *cx.local.last_disp_update = t;
                },
                None => {},
            }
            cx.shared.disp.lock(|disp|{
                disp.flush().unwrap();
            });
            Systick::delay(1.millis()).await;
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
        fps_counter::spawn().ok();
    }

    #[task(priority = 1, shared = [disp], local = [pos])]
    async fn task2(mut cx: task2::Context){
        defmt::info!("task 2");
        let font = MonoTextStyle::new(&FONT_9X15, BinaryColor::On);
        cx.shared.disp.lock(|disp| {
            disp.clear(BinaryColor::Off).unwrap();
            Text::new("Hello, \nfrom task2!", Point::new(4, 13), font)
                .draw(*disp)
                .unwrap();
        });
        fps_counter::spawn().ok();
    }
    #[task(priority = 1, shared = [disp, fps])]
    async fn fps_counter(mut cx: fps_counter::Context){
        let fill = PrimitiveStyle::with_fill(BinaryColor::On);
        let font = MonoTextStyle::new(&FONT_5X7, BinaryColor::Off);
        let mut text: String<16> = String::new();
        cx.shared.fps.lock(|fps|{
            write!(text, "FPS: {:.2}", fps).unwrap();
        });
        cx.shared.disp.lock(|disp|{
            Rectangle::new(Point::new(0, 56),Size::new(132,8)).into_styled(fill).draw(*disp).unwrap();
            Text::new(text.as_str(), Point::new(16, 62), font)
                .draw(*disp)
                .unwrap();
        });
    }

    #[task(binds = DMA2_STREAM2, shared = [spi1_rx_xfer, spi1_rclk_ld])]
    fn dma2_stream2_irq(mut cx: dma2_stream2_irq::Context){
        cx.shared.spi1_rclk_ld.lock(|rclk_ld|{
            rclk_ld.set_high();
            rclk_ld.set_low();
            rclk_ld.set_high();
        });
        let buf: Option<heapless::pool::singleton::Box<P>> = cx.shared.spi1_rx_xfer.lock(|xfer|{
            match xfer.next_transfer(P::alloc().unwrap().init([0x0;2])){
                Ok((buf, current_buf)) => {
                    defmt::info!("{}", buf.as_slice());
                    Some(buf)
                },
                Err(buf) => {defmt::debug!("{}", buf); None},
            }
        });
        buf.unwrap().fill(0);
        cx.shared.spi1_rclk_ld.lock(|rclk_ld|{
            rclk_ld.set_high();
            rclk_ld.set_low();
            rclk_ld.set_high();
        });
    }
    #[task(binds = DMA2_STREAM3, shared = [spi1_tx_xfer, spi1_tx_buf])]
    fn dma2_stream3_irq(mut cx: dma2_stream3_irq::Context){
        let buf: Option<heapless::pool::singleton::Box<P>> = cx.shared.spi1_tx_xfer.lock(|xfer|{
            match xfer.next_transfer(P::alloc().unwrap().init([0x0;2])){
                Ok((buf, current_buf)) => {
                    defmt::info!("{}", buf.as_slice());
                    Some(buf)},
                Err(buf) => {defmt::debug!("{}", buf); None},
            }
        });
        buf.unwrap().fill(0);
    }
}


