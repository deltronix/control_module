#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use control_module as _; // global logger + panicking-behavior + memory layout
use embedded_hal::spi::*;
use rtic::app;
use rtic_monotonics::systick::*;
use rtic_monotonics::systick::fugit::Instant;
use rtic_monotonics::Monotonic;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::{
    spi::{Spi, Rx, Tx},
    gpio::*,
    dma::*,
    pac,
};
use stm32f4xx_hal::dma::config::DmaConfig;
pool!(
    #[link_section = ".ccmram.A"]
    P: [u8;2]
);
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
        let rcc = cx.device.RCC.constrain();
        let ccdr = rcc.cfgr.sysclk(168.MHz()).freeze();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 168_000_000, systick_mono_token);

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
        let spi1 = Spi::new(cx.device.SPI1, (ui_sclk, ui_miso, ui_mosi), MODE_3, 1.MHz(), &ccdr);
        
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

        spi1_rx_xfer.start(|_rx|{});
        spi1_tx_xfer.start(|_tx|{});
        (
            Shared { 
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
    #[idle]
    fn idle(_: idle::Context) -> !{
        loop{
            cortex_m::asm::nop();
        }
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
                Ok((buf, _current_buf)) => {
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
                Ok((buf, _current_buf)) => {
                    defmt::info!("{}", buf.as_slice());
                    Some(buf)},
                Err(buf) => {defmt::debug!("{}", buf); None},
            }
        });
        buf.unwrap().fill(0);
    }
}


