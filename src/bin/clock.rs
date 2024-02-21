#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
use core::task::Poll;

use control_module::{self as _};
use futures::Future;
use rtic::app;


#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [CAN1_TX, CAN1_RX0, CAN1_RX1]
)]
mod app {
    use core::fmt::Write;
    use core::{cell::RefCell, mem::MaybeUninit};

    use super::*;
    use control_module::hardware::switches::{LedId, LedState};
    use control_module::hardware::timer::{TempoTimer, TIMER_FREQ};
    use control_module::hardware::ui::{UiStateMachine, UI};
    use fugit::TimerDurationU64;
    use hal::gpio::{ExtiPin, Output, Pin};
    use hal::spi::Spi3;
    use rtic_monotonics::systick::*;
    use rtic_monotonics::Monotonic;
    use rtic_sync::{channel::*, make_channel};
    use stm32f4xx_hal as hal;
    use heapless::String;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X9, MonoTextStyle},
    pixelcolor::{BinaryColor, PixelColor},
    prelude::*,
    primitives::{Rectangle, PrimitiveStyle},
    text::Text,
};

    use tempo_clock::clock::Clock;
    use tempo_clock::transport::{
        ClockMessage, ClockSource, SenderId, TransportCommand,
    };

    defmt::timestamp!("tick {}", TempoTimer::now().ticks());
    type Duration = TimerDurationU64<84_000_000>;

    const CAPACITY: usize = 16;
    #[shared]
    struct Shared {
        ui: UI,
        clk: Clock<TIMER_FREQ>,
    }
    #[local]
    struct Local<'a> {
        clk_in: Pin<'A', 8>,
        clk_out: Pin<'C', 15, Output>,
        start_stop_in: Pin<'C', 13>,
        clock_sender_exti9_5:
            Sender<'static, ClockMessage<TIMER_FREQ>, CAPACITY>,
        clock_sender_exti15_10:
            Sender<'static, ClockMessage<TIMER_FREQ>, CAPACITY>,
    }
    #[init(local = [spi_bus: MaybeUninit<RefCell<Spi3>> = MaybeUninit::uninit()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // Initialize hardware
        let hardware = control_module::hardware::setup(cx.device);

        // Setup TempoTimer
        TempoTimer::start();
        let (clock_sender, clock_channel_receiver) =
            make_channel!(ClockMessage<TIMER_FREQ>, CAPACITY);
        transport::spawn(clock_channel_receiver).unwrap();

        (
            Shared {
                ui: hardware.ui,
                clk: Clock::default(),
            },
            Local {
                clk_in: hardware.clk_in,
                clk_out: hardware.clk_out,
                start_stop_in: hardware.start_stop_in,
                clock_sender_exti9_5: clock_sender.clone(),
                clock_sender_exti15_10: clock_sender.clone(),
            },
        )
    }
    #[task(shared = [clk, ui])]
    async fn task(mut cx: task::Context){
        (cx.shared.ui, cx.shared.clk).lock(|ui, clk|{
            let disp = ui.display;
            let pos = clk.playhead.position;
            let mut s = String::<16>::new();
            write!(s, "{}", pos.beat()).unwrap();
            Text::new(s.as_str(), Point::new(16,62), MonoTextStyle::new(&FONT_6X9, BinaryColor::Off)).draw(*disp);

        });
    }

    #[task(local = [clk_out], priority = 1)]
    async fn pulse_clk_out(cx: pulse_clk_out::Context) {
        cx.local.clk_out.set_low();
        TempoTimer::delay(Duration::micros(100)).await;
        cx.local.clk_out.set_high();
    }

    #[task(shared = [clk], priority = 1)]
    async fn clk(mut cx: clk::Context){
        loop {

        }

    }

    #[task(shared = [clk], priority = 2)]
    async fn transport(
        mut cx: transport::Context,
        mut receiver: Receiver<'static, ClockMessage<TIMER_FREQ>, CAPACITY>,
    ) {
        use statig::prelude::*;
        use tempo_clock::transport::Transport;
        let mut transport = Transport::default().state_machine();
        loop {
            let inst = cx.shared.clk.lock(|clk| clk.next_tick());
            let msg = match inst {
                // If the clock is running the next tick will be available,
                // use this as a timeout instant
                Some(next_tick) => {
                    match TempoTimer::__tq()
                        .timeout_at(next_tick, receiver.recv())
                        .await
                    {
                        // If a message is received it takes priority over the
                        // internal tick
                        Ok(msg) => msg.unwrap(),
                        // On timeout an internal tick is registered
                        Err(_) => ClockMessage::Tick(),
                    }
                }
                // Otherwise wait for an external event
                None => receiver.recv().await.unwrap(),
            };
            cx.shared.clk.lock(|mut clk| {
                transport.handle_with_context(&msg, &mut clk);
                if clk.playhead.position.tick() == 0 {
                    let _ = pulse_clk_out::spawn();
                }
            })
        }
    }

    #[task(binds = TIM2)]
    fn tim2_irq(_cx: tim2_irq::Context) {
        unsafe {
            TempoTimer::__tq().on_monotonic_interrupt();
        }
    }

    #[task(binds = EXTI9_5, local = [clk_in, clock_sender_exti9_5])]
    fn clk_in_handler(cx: clk_in_handler::Context) {
        cx.local.clk_in.clear_interrupt_pending_bit();
        let sm = ClockMessage::Sync(
            TempoTimer::now(),
            SenderId::Clock(ClockSource::ClockIn),
        );
        cx.local.clock_sender_exti9_5.try_send(sm).unwrap();
    }

    #[task(binds = EXTI15_10, local = [start_stop_in, clock_sender_exti15_10])]
    fn start_stop_in_handler(cx: start_stop_in_handler::Context) {
        if cx.local.start_stop_in.check_interrupt() {
            cx.local.start_stop_in.clear_interrupt_pending_bit();
            if cx.local.start_stop_in.is_high() {
                cx.local
                    .clock_sender_exti15_10
                    .try_send(ClockMessage::TransportCommand(
                        TransportCommand::Play,
                        SenderId::Clock(ClockSource::ClockIn),
                    ))
                    .unwrap();
            } else {
                cx.local
                    .clock_sender_exti15_10
                    .try_send(ClockMessage::TransportCommand(
                        TransportCommand::Stop,
                        SenderId::Clock(ClockSource::ClockIn),
                    ))
                    .unwrap();
            }
        }
    }
}
