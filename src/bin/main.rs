#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
use control_module as _; // global logger + panicking-behavior + memory layout
use rtic::app;
use heapless::pool::boxed::Box;

#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {
    use super::*;
    use cortex_m::asm::delay;
    use control_module::spi::{UI, UiEvent};
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use stm32f4xx_hal::{prelude::*, spi::{Spi1, Spi, Mode}, pac};
    use embedded_hal::{spi::{MODE_1, SpiDevice, SpiBus, MODE_2, MODE_0}, digital::OutputPin};


    const CAPACITY: usize = 16;
    #[shared]
    struct Shared {
        ui: UI<6>,
    }
    #[local]
    struct Local {
        counter: u8,
    }
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let rcc = cx.device.RCC.constrain();
        let ccdr = rcc.cfgr.sysclk(168.MHz()).freeze();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 168_000_000, systick_mono_token);

        let gpiob = cx.device.GPIOB.split();
        let (spi1_sclk, spi1_miso, spi1_mosi) = {(
                gpiob.pb3.into_alternate(),
                gpiob.pb4.into_alternate(),
                gpiob.pb5.into_alternate(),
        )};
        let mut spi1 = Spi::new(cx.device.SPI1, (spi1_sclk, spi1_miso, spi1_mosi), MODE_0, 1.MHz(), &ccdr);


        let mut ui = UI::new(spi1,gpiob.pb6.into_open_drain_output_in_state(stm32f4xx_hal::gpio::PinState::High));

        let tx: [u8; 6] = [0b10101010; 6];
        ui.transfer(&tx);


        let (s, r) = make_channel!(UiEvent, CAPACITY);


        receiver::spawn(r).unwrap();
        read_ui::spawn(s.clone()).unwrap();
        sender1::spawn(s.clone()).unwrap();
        (
            Shared{
                ui
            },
            Local{
                counter:0,
            }
        )
    }
    #[task(shared = [ui], local = [counter])]
    async fn read_ui(mut cx: read_ui::Context, mut sender: Sender<'static, UiEvent, CAPACITY>) {
        defmt::info!("idle start");
        loop {
            Systick::delay(100.millis()).await;
            cx.shared.ui.lock(|ui|{
                let tx: [u8; 6] = [*cx.local.counter; 6];
                if let Some(rx) = ui.transfer(&tx){
                    defmt::info!("rx: {:?}", rx);
                    ui.update(|ev| sender.send(ev).await.unwrap() );
                }
                else {
                }
            });
            *cx.local.counter = cx.local.counter.wrapping_add(1);
        }
    }


    #[task]
    async fn receiver(_c: receiver::Context, mut receiver: Receiver<'static, UiEvent, CAPACITY>){

    }
    #[task]
    async fn sender1(_c: sender1::Context, mut sender: Sender<'static, UiEvent, CAPACITY>){

    }

}
