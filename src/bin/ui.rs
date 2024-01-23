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
use heapless::Vec;
box_pool!(
    P: [u8; 4]
);
use stm32f4xx_hal::dma::config::DmaConfig;
use heapless::{box_pool, pool::boxed::Box};
use control_module::spi::{Spi1DmaRx, Spi1DmaTx};
#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {

    use cortex_m::asm::delay;
    use heapless::pool::boxed::{BoxBlock, Box, BoxPool};
    use stm32f4xx_hal::timer::DelayMs;

    use super::*;
    #[shared]
    struct Shared {
        spi1_rx_xfer: Spi1DmaRx,
        spi1_tx_xfer: Spi1DmaTx,
        spi1_rx_buf: Option<Box<P>>,
        spi1_tx_buf: Option<Box<P>>,
        spi1_rclk_ld: Pin<'B', 6, Output<OpenDrain>>,
    }
    #[local]
    struct Local {
        // TODO: Add resources
    }
    #[init(local = [
        //spi1_rx_xfer: Option<Spi1DmaRx> = None,
        //spi1_tx_xfer: Option<Spi1DmaTx> = None,
        spi1_rx_buf: Option<Box<P>> = None,
        spi1_tx_buf: Option<Box<P>> = None,
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

        let blocks: &'static mut [BoxBlock<[u8; 4]>] = {
            const BLOCK: BoxBlock<[u8; 4]> = BoxBlock::new();
            static mut BLOCKS: [BoxBlock<[u8; 4]>; 8] = [BLOCK; 8];
            unsafe{ &mut BLOCKS }
        };
        for block in blocks {
            P.manage(block);
        }
        let mut spi1_rx_xfer = Transfer::init_peripheral_to_memory(
            spi1_rx_stream,
            spi1_rx,
            P.alloc([0;4]).unwrap(),
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
            P.alloc([0;4]).unwrap(),
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
                spi1_rx_xfer,
                spi1_tx_xfer,
                spi1_rx_buf: None,
                spi1_tx_buf: None,
                spi1_rclk_ld: ui_rclk_ld,
              }, 
            Local { 
        })
    }

    // Optional idle, can be removed if not needed.
    //
    #[idle(shared = [spi1_rx_xfer])]
    fn idle(mut cx: idle::Context) -> !{
        defmt::info!("idle start");
        loop{
            delay(168000000);
            cortex_m::asm::nop();
            defmt::info!("idle tick");
        }
    }
    #[task(binds = DMA2_STREAM2, shared = [spi1_rx_xfer, spi1_rclk_ld, spi1_rx_buf])]
    fn dma2_stream2_irq(mut cx: dma2_stream2_irq::Context){
        cx.shared.spi1_rclk_ld.lock(|rclk_ld|{
            rclk_ld.set_high();
            rclk_ld.set_low();
            rclk_ld.set_high();
        });
        cx.shared.spi1_rx_buf.lock(|rx_buf| {
            let (buf, _ ) = cx.shared.spi1_rx_xfer.lock(|xfer|{
                 xfer.next_transfer(P.alloc([0;4]).unwrap())
            }).unwrap();
            *rx_buf = Some(buf);
        });
    

        cx.shared.spi1_rclk_ld.lock(|rclk_ld|{
            rclk_ld.set_high();
            rclk_ld.set_low();
            rclk_ld.set_high();
        });
    }
    #[task(binds = DMA2_STREAM3, shared = [spi1_tx_xfer, spi1_rx_buf])]
    fn dma2_stream3_irq(mut cx: dma2_stream3_irq::Context){
        cx.shared.spi1_tx_xfer.lock(|xfer|{
            cx.shared.spi1_rx_buf.lock(|tx|{
                match tx {
                    Some(tx) => {xfer.next_transfer(tx.clone()).unwrap();},
                    None => {},
                }
            })
        });
    }
}


