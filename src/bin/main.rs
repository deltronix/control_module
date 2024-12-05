#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
use control_module::{self as _};
use rtic::app;

#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {
    use ad57xx::{Ad57xx, Ad57xxShared};
    use control_module::hardware::switches::UiEvent;
    use control_module::hardware::timer::TempoTimer;
    use control_module::hardware::ui::{UiStateMachine, UI};
    use core::{cell::RefCell, mem::MaybeUninit};
    use embedded_hal::spi::SpiDevice;
    use embedded_hal_bus::spi::NoDelay;
    use embedded_hal_bus::spi::RefCellDevice;
    use hal::gpio::{Output, Pin};
    use hal::spi::Spi3;
    use rtic_monotonics::systick::*;
    use rtic_sync::{channel::*, make_channel};
    use statig::prelude::*;
    use stm32f4xx_hal as hal;

    const CAPACITY: usize = 16;
    #[shared]
    struct Shared {
        ui: UI,
    }
    #[local]
    struct Local<'a> {
        ui_fsm: StateMachine<UiStateMachine>,
        spi_dev2: RefCellDevice<'static, Spi3, Pin<'D', 2, Output>, NoDelay>,
        dac: Ad57xxShared<RefCellDevice<'static, Spi3, Pin<'A', 15, Output>, NoDelay>, ad57xx::marker::Ad57x4>,
    }
    #[init(local = [spi_bus: MaybeUninit<RefCell<Spi3>> = MaybeUninit::uninit()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // Initialize hardware
        let hardware = control_module::hardware::setup(cx.device);

        // Setup shared bus on SPI3
        let spi_bus = cx.local.spi_bus.write(RefCell::new(hardware.spi3));
        let spi_dev1 = RefCellDevice::new(spi_bus, hardware.io_sync, NoDelay);
        let spi_dev2 = RefCellDevice::new(spi_bus, hardware.io_rclk, NoDelay);
        let dac = Ad57xxShared::new_ad57x4(spi_dev1);

        // Setup TempoTimer
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 168_000_000, systick_mono_token);
        TempoTimer::start();
        let clk_in = hardware.clk_in;

        // Setup ui state machine
        let ui_fsm = UiStateMachine::new().state_machine();
        let (event_channel_sender, event_channel_receiver) = make_channel!(UiEvent, CAPACITY);

        ev_handler::spawn(event_channel_receiver).unwrap();
        read_ui::spawn(event_channel_sender.clone()).unwrap();
        io::spawn().unwrap();
        dio::spawn([0x00; 2]).unwrap();

        (Shared { ui: hardware.ui }, Local { ui_fsm, dac, spi_dev2 })
    }
    #[task(local = [dac])]
    async fn io(cx: io::Context) {
        let dac = cx.local.dac;
        dac.set_power(ad57xx::ad57x4::ChannelQuad::AllDacs, true).unwrap();
        dac.set_output_range(ad57xx::ad57x4::ChannelQuad::AllDacs, ad57xx::OutputRange::Bipolar5V)
            .unwrap();
        dac.set_dac_output(ad57xx::ad57x4::ChannelQuad::DacA, 0x8000).unwrap();
        dac.set_dac_output(ad57xx::ad57x4::ChannelQuad::DacB, 0x0000).unwrap();
        dac.set_dac_output(ad57xx::ad57x4::ChannelQuad::DacC, 0xFFFF).unwrap();
    }
    #[task(local = [spi_dev2])]
    async fn dio(cx: dio::Context, tx: [u8; 2]) {
        cx.local.spi_dev2.write(&tx).unwrap();
    }
    #[task(shared = [ui])]
    async fn read_ui(mut cx: read_ui::Context, mut sender: Sender<'static, UiEvent, CAPACITY>) {
        loop {
            Systick::delay(1.millis()).await;
            cx.shared.ui.lock(|ui| {
                if ui.switches.transfer().is_some() {
                    ui.switches.update(|ev| {
                        sender.try_send(ev).unwrap();
                    });
                } else {
                }
            });
        }
    }
    #[task(local = [ui_fsm], shared = [ui])]
    async fn ev_handler(mut cx: ev_handler::Context, mut receiver: Receiver<'static, UiEvent, CAPACITY>) {
        loop {
            let ev = receiver.recv().await.unwrap();
            cx.shared.ui.lock(|ui| {
                cx.local.ui_fsm.handle_with_context(&ev, ui);
            })
        }
    }
}
