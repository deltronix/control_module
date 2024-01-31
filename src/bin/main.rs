#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
use control_module as _; // global logger + panicking-behavior + memory layout
use rtic::app;

#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [TIM3]
)]
mod app {
    use control_module::hardware::switches::{LedId, LedState, UiEvent};
    use control_module::hardware::ui::{UI, UiStateMachine};
    use control_module::hardware::project::{Generator, Lane, Project};
    use statig::prelude::*;
    use rtic_monotonics::systick::*;
    use rtic_monotonics::Monotonic;
    use control_module::hardware::timer::TempoTimer;
    use rtic_sync::{channel::*, make_channel};
    use fugit::{Instant, Duration};

    const CAPACITY: usize = 16;
    #[shared]
    struct Shared {
        ui: UI,
        
    }
    #[local]
    struct Local {
        ui_fsm: StateMachine<UiStateMachine>,
    }
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let p = control_module::hardware::Peripherals {
            gpiob: cx.device.GPIOB,
            spi1: cx.device.SPI1,
            spi2: cx.device.SPI2,
            tim2: cx.device.TIM2,
            tim14: cx.device.TIM14,
        };
        let hardware = control_module::hardware::setup(p, cx.device.RCC);
        
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 168_000_000, systick_mono_token);
        TempoTimer::start();


        TempoTimer::now();
        let ui_fsm = UiStateMachine::new().state_machine();
        let (s, r) = make_channel!(UiEvent, CAPACITY);

        let l = Lane::Generator { t: Generator::Lfo{ waveform: 0xF} };
        defmt::info!("sizeof Lane: {}", core::mem::size_of::<Lane>());
        defmt::info!("sizeof Project: {}", core::mem::size_of::<Project>());
        
        ev_handler::spawn(r).unwrap();
        read_ui::spawn(s.clone()).unwrap();
        test::spawn().unwrap();
        (Shared { ui: hardware.ui }, Local { ui_fsm })
    }
    #[task(shared = [ui])]
    async fn read_ui(mut cx: read_ui::Context, mut sender: Sender<'static, UiEvent, CAPACITY>) {
        loop {
            Systick::delay(1.millis()).await;
            cx.shared.ui.lock(|ui|{
                if ui.switches.transfer().is_some() {
                    ui.switches.update(|ev|{
                        sender.try_send(ev).unwrap();
                    });
                }
                else {}
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
            cx.shared.ui.lock(|ui|{
                cx.local.ui_fsm.handle_with_context(&ev, ui);
            })
        }
    }

    #[task(shared = [ui])]
    async fn test(mut cx: test::Context){
        loop {
            cx.shared.ui.lock(|ui|{
                ui.switches.set_led(&LedId::Reset, LedState::On);
            });
            TempoTimer::delay(500.millis().into()).await;
            cx.shared.ui.lock(|ui|{
                ui.switches.set_led(&LedId::Reset, LedState::Off);
            });
            TempoTimer::delay(500.millis().into()).await;
        }
    }
    
    #[task(binds = TIM2)]
    fn tim2_irq(_cx: tim2_irq::Context) {
        TempoTimer::on_interrupt();
        unsafe{
            TempoTimer::__tq().on_monotonic_interrupt();
        }
        defmt::info!("TIM2 IRQ @ {}", TempoTimer::now());
    }
}
