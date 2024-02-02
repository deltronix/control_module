#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
use control_module::{self as _, hardware::io::IO}; // global logger + panicking-behavior + memory layout
use rtic::app;


#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {
    use core::cell::RefCell;

    use control_module::hardware::project::{Generator, Lane, Project};
    use control_module::hardware::switches::{LedId, LedState, UiEvent};
    use control_module::hardware::timer::{TempoTimer, TIMER_FREQ};
    use control_module::hardware::ui::{UiStateMachine, UI};
    use embedded_hal::digital::OutputPin;
    use embedded_hal::spi::SpiDevice;
    use embedded_hal_bus::spi::{NoDelay, RefCellDevice};
    use fugit::{Duration, Instant, Rate};
    use hal::gpio::{ExtiPin, Output, Pin};
    use hal::pac::SPI3;
    use hal::spi::{Spi, Spi3};
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
    }
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let hardware = control_module::hardware::setup(cx.device);
        let clk_in = hardware.clk_in;
        let spi_bus = RefCell::new(hardware.spi3);

        let spi_dev1 = RefCellDevice::new(&spi_bus, hardware.io_sync, NoDelay);

        let mut io = control_module::hardware::io::IO::new(spi_dev1, hardware.io_rclk);
        io.dac
            .set_output_range(ad57xx::Channel::AllDacs, ad57xx::OutputRange::Bipolar5V)
            .unwrap();
        io.dac
            .set_dac_output(ad57xx::Channel::DacA, 0x8000)
            .unwrap();
        io.dac
            .set_dac_output(ad57xx::Channel::DacB, 0x0000)
            .unwrap();
        io.dac
            .set_dac_output(ad57xx::Channel::DacC, 0xFFFF)
            .unwrap();

        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 168_000_000, systick_mono_token);
        TempoTimer::start();

        let tap_tempo = TapTempo::<84_000_000, 8>::new(4, 0.1);

        let ui_fsm = UiStateMachine::new().state_machine();
        let (event_channel_sender, event_channel_receiver) = make_channel!(UiEvent, CAPACITY);
        let (clock_channel_sender, clock_channel_receiver) = make_channel!(SyncMsg, CAPACITY);

        let l = Lane::Generator {
            t: Generator::Lfo { waveform: 0xF },
        };
        defmt::info!("sizeof Lane: {}", core::mem::size_of::<Lane>());
        defmt::info!("sizeof Project: {}", core::mem::size_of::<Project>());

        ev_handler::spawn(event_channel_receiver).unwrap();
        read_ui::spawn(event_channel_sender.clone()).unwrap();
        test::spawn(clock_channel_receiver).unwrap();
        (
            Shared { ui: hardware.ui },
            Local {
                ui_fsm,
                clk_in,
                tap_tempo,
                clock_channel_sender,
                clk_out: hardware.clk_out,
                io,
            },
        )
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
