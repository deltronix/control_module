#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use control_module as _; /* global logger + panicking-behavior + memory
                           * layout */
use embedded_hal::spi::*;
use rtic::app;
use rtic_monotonics::systick::*;
use stm32f4xx_hal::dma::{
    config::DmaConfig, DmaFlag, MemoryToPeripheral, PeripheralToMemory,
    Stream2, Stream3, Transfer,
};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::{
    dma::*,
    gpio::*,
    pac,
    spi::{Rx, Spi, Tx},
};
const SPI1_XFER_SIZE: usize = 6;

type Spi1DmaRx = Transfer<
    Stream2<pac::DMA2>,
    3,
    Rx<pac::SPI1>,
    PeripheralToMemory,
    &'static mut [u8; SPI1_XFER_SIZE],
>;
type Spi1DmaTx = Transfer<
    Stream3<pac::DMA2>,
    3,
    Tx<pac::SPI1>,
    MemoryToPeripheral,
    &'static mut [u8; SPI1_XFER_SIZE],
>;
#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {

    use cortex_m::asm::delay;

    use super::*;
    #[shared]
    struct Shared {
        spi1_rx_xfer: Spi1DmaRx,
        spi1_tx_xfer: Spi1DmaTx,
        spi1_rclk_ld: Pin<'B', 6, Output<OpenDrain>>,
    }
    #[local]
    struct Local {
        spi1_rx_buffer: Option<&'static mut [u8; SPI1_XFER_SIZE]>,
        spi1_tx_buffer: Option<&'static mut [u8; SPI1_XFER_SIZE]>,
        // TODO: Add resources
    }
    #[init(local = [
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
        let (ui_sclk, ui_miso, ui_mosi, mut ui_rclk_ld) = {
            (
                gpiob.pb3.into_alternate(),
                gpiob.pb4.into_alternate(),
                gpiob.pb5.into_alternate(),
                gpiob.pb6.into_open_drain_output(),
            )
        };
        ui_rclk_ld.set_high();
        ui_rclk_ld.set_speed(Speed::High);
        let spi1 = Spi::new(
            cx.device.SPI1,
            (ui_sclk, ui_miso, ui_mosi),
            MODE_3,
            1.MHz(),
            &ccdr,
        );

        let (spi1_tx, spi1_rx) = spi1.use_dma().txrx();
        let streams = StreamsTuple::new(cx.device.DMA2);
        let spi1_rx_stream = streams.2;
        let spi1_tx_stream = streams.3;

        let spi1_rx_buf =
            cortex_m::singleton!(: [u8; SPI1_XFER_SIZE] = [0; SPI1_XFER_SIZE])
                .unwrap();
        let spi1_tx_buf =
            cortex_m::singleton!(: [u8; SPI1_XFER_SIZE] = [0b10101010; SPI1_XFER_SIZE]).unwrap();

        let mut spi1_rx_xfer = Transfer::init_peripheral_to_memory(
            spi1_rx_stream,
            spi1_rx,
            spi1_rx_buf,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
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
                .transfer_complete_interrupt(true),
        );

        spi1_rx_xfer.start(|_rx| {});
        spi1_tx_xfer.start(|_tx| {});

        let spi1_rx_buffer2 =
            cortex_m::singleton!(: [u8; SPI1_XFER_SIZE] = [0; SPI1_XFER_SIZE])
                .unwrap();
        let spi1_tx_buffer2 =
            cortex_m::singleton!(: [u8; SPI1_XFER_SIZE] = [0b10101010; SPI1_XFER_SIZE]).unwrap();
        (
            Shared {
                spi1_rx_xfer,
                spi1_tx_xfer,
                spi1_rclk_ld: ui_rclk_ld,
            },
            Local {
                spi1_rx_buffer: Some(spi1_rx_buffer2),
                spi1_tx_buffer: Some(spi1_tx_buffer2),
            },
        )
    }

    // Optional idle, can be removed if not needed.
    //
    #[idle(shared = [spi1_rx_xfer])]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle start");
        loop {
            delay(168000000);
            cortex_m::asm::nop();
            defmt::info!("idle tick");
        }
    }
    #[task(binds = DMA2_STREAM2, shared = [spi1_rx_xfer, spi1_rclk_ld], local = [spi1_rx_buffer])]
    fn dma2_stream2_irq(cx: dma2_stream2_irq::Context) {
        let mut rx_transfer = cx.shared.spi1_rx_xfer;
        let mut latchdown = cx.shared.spi1_rclk_ld;
        let rx_buffer = cx.local.spi1_rx_buffer;
        latchdown.lock(|ld| {
            ld.set_low();
            ld.set_high();
        });
        rx_transfer.lock(|xfer| {
            let flags = xfer.flags();
            xfer.clear_flags(DmaFlag::FifoError | DmaFlag::TransferComplete);
            if flags.is_transfer_complete() {
                let (filled_buffer, _) =
                    xfer.next_transfer(rx_buffer.take().unwrap()).unwrap();
                *rx_buffer = Some(filled_buffer);
            }
        });
        latchdown.lock(|ld| {
            ld.set_low();
            ld.set_high();
        });
    }
    #[task(binds = DMA2_STREAM3, shared = [spi1_tx_xfer], local = [spi1_tx_buffer])]
    fn dma2_stream3_irq(cx: dma2_stream3_irq::Context) {
        let mut tx_transfer = cx.shared.spi1_tx_xfer;
        let tx_buffer = cx.local.spi1_tx_buffer;

        tx_transfer.lock(|xfer| {
            let flags = xfer.flags();
            xfer.clear_flags(DmaFlag::FifoError | DmaFlag::TransferComplete);
            if flags.is_transfer_complete() {
                let (filled_buffer, _) =
                    xfer.next_transfer(tx_buffer.take().unwrap()).unwrap();
                *tx_buffer = Some(filled_buffer);
            }
        });
    }
}
