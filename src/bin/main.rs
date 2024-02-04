#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
use control_module::{self as _}; // global logger + panicking-behavior + memory layout
use rtic::app;


#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {
    use core::cell::RefCell;
    use core::mem::MaybeUninit;

    use ad57xx::Ad57xxShared;
    use control_module::hardware::switches::{LedId, LedState, UiEvent};
    use control_module::hardware::timer::{TempoTimer, TIMER_FREQ};
    use control_module::hardware::ui::{UiStateMachine, UI};
    use embedded_hal::spi::SpiDevice;
    use embedded_hal_bus::spi::{NoDelay, RefCellDevice};
    use fugit::Instant;
    use hal::gpio::{ExtiPin, Output, Pin};
    use hal::spi::Spi3;
    use rtic_monotonics::systick::*;
    use rtic_monotonics::Monotonic;
    use rtic_sync::{channel::*, make_channel};
    use statig::prelude::*;
    use stm32f4xx_hal as hal;
    use tempo_clock::sync::{ExtClock, SyncMessage, TapTempo};

    type SyncMsg = SyncMessage<84_000_000>;

    const CAPACITY: usize = 16;
    #[shared]
    struct Shared {
        ui: UI,
    }
    #[local]
    struct Local<'a> {
        ui_fsm: StateMachine<UiStateMachine>,
        clk_in: Pin<'A', 8>,
        clk_out: Pin<'C', 15, Output>,
        tap_tempo: TapTempo<84_000_000, 8>,
        clock_channel_sender: Sender<'static, SyncMsg, CAPACITY>,
        spi_dev2: RefCellDevice<'static, Spi3, Pin<'D', 2, Output>, NoDelay>,
        dac: Ad57xxShared<RefCellDevice<'static, Spi3, Pin<'A', 15, Output>, NoDelay>>
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
        let dac = Ad57xxShared::new(spi_dev1);

        // Setup TempoTimer
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 168_000_000, systick_mono_token);
        TempoTimer::start();
        let clk_in = hardware.clk_in;
        let tap_tempo = TapTempo::<84_000_000, 8>::new(4, 0.1);
        let (clock_channel_sender, clock_channel_receiver) = make_channel!(SyncMsg, CAPACITY);

        // Setup ui state machine
        let ui_fsm = UiStateMachine::new().state_machine();
        let (event_channel_sender, event_channel_receiver) = make_channel!(UiEvent, CAPACITY);


        ev_handler::spawn(event_channel_receiver).unwrap();
        read_ui::spawn(event_channel_sender.clone()).unwrap();
        test::spawn(clock_channel_receiver).unwrap();
        io::spawn().unwrap();
        io2::spawn().unwrap();
        (
            Shared { ui: hardware.ui },
            Local {
                ui_fsm,
                clk_in,
                tap_tempo,
                clock_channel_sender,
                clk_out: hardware.clk_out,
                dac,
                spi_dev2,
            },
        )
    }

    #[task(local = [dac])]
    async fn io(cx: io::Context){
        let dac = cx.local.dac;
        dac.set_power(ad57xx::Channel::AllDacs, true).unwrap();
        dac
            .set_output_range(ad57xx::Channel::AllDacs, ad57xx::OutputRange::Bipolar5V)
            .unwrap();
        dac
            .set_dac_output(ad57xx::Channel::DacA, 0x8000)
            .unwrap();
        dac
           .set_dac_output(ad57xx::Channel::DacB, 0x0000)
            .unwrap();
        dac
            .set_dac_output(ad57xx::Channel::DacC, 0xFFFF)
            .unwrap();
    }
    #[task(local = [spi_dev2])]
    async fn io2(cx: io2::Context){
        let tx = [0b11001100; 2];
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
    async fn ev_handler(
        mut cx: ev_handler::Context,
        mut receiver: Receiver<'static, UiEvent, CAPACITY>,
    ) {
        loop {
            let ev = receiver.recv().await.unwrap();
            cx.shared.ui.lock(|ui| {
                cx.local.ui_fsm.handle_with_context(&ev, ui);
            })
        }
    }

    #[task(shared = [ui], local = [clk_out])]
    async fn test(mut cx: test::Context, mut receiver: Receiver<'static, SyncMsg, CAPACITY>) {
        let mut on: bool = false;
        loop {
            if let Some(msg) = receiver.recv().await.ok() {
                defmt::info!("receiver: {}", msg);
                cx.shared.ui.lock(|ui| {
                    if on {
                        cx.local.clk_out.set_low();
                        ui.switches.set_led(&LedId::Reset, LedState::Off);
                        on = false;
                    } else {
                        cx.local.clk_out.set_high();
                        ui.switches.set_led(&LedId::Reset, LedState::On);
                        on = true;
                    }
                });
            }
        }
    }

    #[task(binds = TIM2)]
    fn tim2_irq(_cx: tim2_irq::Context) {
        TempoTimer::on_interrupt();
        unsafe {
            TempoTimer::__tq().on_monotonic_interrupt();
        }
        let i: Instant<u64, 1, 84_000_000> = TempoTimer::now();
    }

    #[task(binds = EXTI9_5, local = [clk_in, tap_tempo, clock_channel_sender])]
    fn clk_in_handler(cx: clk_in_handler::Context) {
        cx.local.clk_in.clear_interrupt_pending_bit();
        let now = TempoTimer::now();
        if let Some(dur) = cx.local.tap_tempo.tick(now) {
            let sm = SyncMessage::<TIMER_FREQ>(now, dur);
            cx.local.clock_channel_sender.try_send(sm).unwrap();
            defmt::info!(
                "CLK BPM: {} now: {} dur: {}",
                cx.local.tap_tempo.get_bpm(),
                now,
                dur.to_micros()
            );
        }
    }
}
