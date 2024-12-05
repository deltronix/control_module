#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(maybe_uninit_uninit_array)]
#![feature(maybe_uninit_array_assume_init)]
// #![feature(const_maybe_uninit_uninit_array)]
use control_module::{self as _};
use rtic::app;

#[app(
    device = stm32f4xx_hal::pac,
    peripherals = true,
    dispatchers = [CAN1_TX, CAN1_RX0, CAN1_RX1]
)]
mod app {
    use core::fmt::Write;
    use core::{cell::RefCell, mem::MaybeUninit};

    use ad57xx::{Ad57xx, Ad57xxShared};
    use control_module::hardware::display::DisplayType;
    use control_module::hardware::timer::{TempoTimer, TIMER_FREQ};
    use embedded_hal::spi::SpiDevice;
    use embedded_hal_bus::spi::{NoDelay, RefCellDevice};
    use fugit::TimerDurationU64;
    use hal::prelude::_embedded_hal_serial_nb_Read;
    use hal::{
        serial::Serial,
        gpio::{ExtiPin, Output, Pin},
    };
    use hal::{serial::RxISR, spi::Spi3};
    use heapless::{FnvIndexMap, String};
    use midly::stream::MidiStream;
    use midly::MidiMessage;
    use rtic_monotonics::systick::*;
    use rtic_monotonics::Monotonic;
    use rtic_sync::{channel::*, make_channel};
    use stm32f4xx_hal as hal;
    use stm32f4xx_hal::pac::UART4;
    

    use embedded_graphics::{
        mono_font::{ascii::FONT_5X8, MonoTextStyle},
        pixelcolor::BinaryColor,
        prelude::*,
        text::Text,
    };

    use tempo_clock::clock::Clock;
    use tempo_clock::transport::{ClockMessage, ClockSource, TransportCommand};

    defmt::timestamp!("tick {}", TempoTimer::now().ticks());
    type Duration = TimerDurationU64<84_000_000>;

    const CAPACITY: usize = 16;

    #[shared]
    struct Shared {
        disp: DisplayType,
        clk: Clock<TIMER_FREQ>,
    }
    #[local]
    struct Local {
        clk_in: Pin<'A', 8>,
        clk_out: Pin<'C', 15, Output>,
        start_stop_in: Pin<'C', 13>,
        clock_sender_exti9_5:
            Sender<'static, ClockMessage<TIMER_FREQ>, CAPACITY>,
        clock_sender_exti15_10:
            Sender<'static, ClockMessage<TIMER_FREQ>, CAPACITY>,
        clock_sender_midi: Sender<'static, ClockMessage<TIMER_FREQ>, CAPACITY>,
        midi_sender: Sender<'static, (u8, MidiMessage), CAPACITY>,
        midi_stream: midly::stream::MidiStream,
        midi: Serial<UART4>,
        dio: RefCellDevice<'static, Spi3, Pin<'D', 2, Output>, NoDelay>,
        dac: Ad57xxShared<
            RefCellDevice<'static, Spi3, Pin<'A', 15, Output>, NoDelay>,
            ad57xx::marker::Ad57x4,
        >,
    }
    #[init(local = [spi_bus: MaybeUninit<RefCell<Spi3>> = MaybeUninit::uninit()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Initialize hardware
        defmt::info!("Initializing...");
        let hardware = control_module::hardware::setup(cx.device);

        // Setup TempoTimer
        defmt::info!("Tempo timer setup");
        TempoTimer::start();
        let (clock_sender, clock_channel_receiver) =
            make_channel!(ClockMessage<TIMER_FREQ>, CAPACITY);
        transport::spawn(clock_channel_receiver).unwrap();

        // Setup digital and analog io
        defmt::info!("IO setup");
        let spi_bus = cx.local.spi_bus.write(RefCell::new(hardware.spi3));
        let spi_dev1 = RefCellDevice::new(spi_bus, hardware.io_sync, NoDelay);
        let spi_dev2 = RefCellDevice::new(spi_bus, hardware.io_rclk, NoDelay);
        let mut dac = Ad57xxShared::new_ad57x4(spi_dev1);
        dac.set_power(ad57xx::ad57x4::ChannelQuad::AllDacs, true)
            .unwrap();
        dac.set_output_range(
            ad57xx::ad57x4::ChannelQuad::AllDacs,
            ad57xx::OutputRange::Bipolar5V,
        )
        .unwrap();


        defmt::info!("MIDI setup");
        let (midi_sender, midi_receiver) =
            make_channel!((u8, MidiMessage), CAPACITY);
        midi_msg_handler::spawn(midi_receiver).unwrap();

        (
            Shared {
                disp: hardware.ui.display,
                clk: Clock::default(),
            },
            Local {
                clk_in: hardware.clk_in,
                clk_out: hardware.clk_out,
                start_stop_in: hardware.start_stop_in,
                clock_sender_exti9_5: clock_sender.clone(),
                clock_sender_exti15_10: clock_sender.clone(),
                clock_sender_midi: clock_sender.clone(),
                midi_sender,
                midi_stream: MidiStream::new(),
                dio: spi_dev2,
                dac,
                midi: hardware.midi,
            },
        )
    }
    #[task(shared = [clk, disp])]
    async fn task(cx: task::Context) {
        (cx.shared.disp, cx.shared.clk).lock(|disp, clk| {
            let pos = clk.playhead.position;
            let mut s = String::<16>::new();
            write!(s, "bar: {}", pos.bar()).unwrap();
            disp.clear(BinaryColor::Off).unwrap();
            Text::new(
                s.as_str(),
                Point::new(16, 16),
                MonoTextStyle::new(&FONT_5X8, BinaryColor::On),
            )
            .draw(disp)
            .unwrap();
            disp.flush().unwrap();
        });
    }

    #[task( 
        priority = 1,
        local = [
        dac,
        dio,
        gates: [u8;2] = [0;2], 
        channel_notes: MaybeUninit<[FnvIndexMap<u8,u8,8>;16]> = MaybeUninit::uninit(),
        channel_pitchbend: [f32; 16] = [0.0;16]]
        )]
    async fn midi_msg_handler(
        cx: midi_msg_handler::Context,
        mut receiver: Receiver<'static, (u8, MidiMessage), CAPACITY>,
    ) {
        let midi_msg_handler::LocalResources {
            dac,
            dio,
            gates,
            channel_notes,
            channel_pitchbend,
            ..
        } = cx.local;
        
        // Assume init is unsafe, we have to initialize the MaybeUninit before using it. This
        // IndexMap stores velocities with the midi note as key.
        let ch_notes = unsafe{ channel_notes.assume_init_mut().fill(FnvIndexMap::<u8,u8,8>::new()); channel_notes.assume_init_mut()};

        loop {
            while !receiver.is_empty() {
                let (ch, msg) = receiver.recv().await.unwrap();
                let ch = usize::from(ch);
                        match msg {
                            MidiMessage::NoteOff{key,vel}=>{
                                let(key,vel)=(u8::from(key),u8::from(vel));
                                defmt::info!("CH: {}, NOTE OFF: {}, {}",ch,key,vel);
                                ch_notes[usize::from(ch)].remove(&key);
                            },
                            MidiMessage::NoteOn{key,vel}=>{
                                let(key,vel)=(u8::from(key),u8::from(vel));
                                if vel==0{
                                    defmt::info!("CH: {}, NOTE OFF: {}, {}",ch,key,vel);
                                    ch_notes[ch].remove(&key);
                                }
                                else{
                                    defmt::info!("CH: {}, NOTE ON: {}, {}",ch,key,vel);
                                    ch_notes[ch].insert(key,vel).unwrap();
                                }
                            },
                            MidiMessage::Controller { controller: _, value: _ } => {
                            }
                            MidiMessage::PitchBend { bend } => {
                                channel_pitchbend[ch] = bend.as_f32();
                            }
                            MidiMessage::Aftertouch { key, vel } => {
                                let(key,vel)=(u8::from(key),u8::from(vel));
                                if let Some(v) = ch_notes[ch].get_mut(&key){
                                    *v = vel;
                                }
                            },
                            MidiMessage::ChannelAftertouch { vel } => {
                                ch_notes[ch].iter_mut().for_each(|(_, v)|{
                                        *v = vel.into();
                                })
                            }
                           _ => {},
                }

            }

            ch_notes.iter().enumerate().for_each(|(i, notes)|{
                if notes.is_empty(){
                    gates[i/8] &= !(0b1 << i%8);
                }
                else {
                    gates[i/8] |= 0b1 << i%8;
                }
            });
            ch_notes
                .iter()
                .take(4)
                .enumerate()
                .for_each(|(i, notes)| {
                    if let Some((note, _vel)) = notes.last() {
                    let mut val: u16 = u16::from(*note)
                        .clamp(0, 120)
                        .checked_mul(546u16)
                        .unwrap();
                    val += (546.0*channel_pitchbend[i]) as u16;
                    match i {
                        0 => {
                            dac.set_dac_output(
                                ad57xx::ad57x4::ChannelQuad::DacA,
                                val,
                            )
                            .unwrap();
                        }
                        1 => {
                            dac.set_dac_output(
                                ad57xx::ad57x4::ChannelQuad::DacB,
                                val,
                            )
                            .unwrap();
                        }
                        2 => {
                            dac.set_dac_output(
                                ad57xx::ad57x4::ChannelQuad::DacC,
                                val,
                            )
                            .unwrap();
                        }
                        3 => {
                            dac.set_dac_output(
                                ad57xx::ad57x4::ChannelQuad::DacD,
                                val,
                            )
                            .unwrap();
                        }
                        _ => {}
                    }
                    }
                });
            dio.write(gates).unwrap();
        }
    }

    #[task(local = [clk_out], priority = 1)]
    async fn pulse_clk_out(cx: pulse_clk_out::Context) {
        cx.local.clk_out.set_low();
        TempoTimer::delay(Duration::micros(100)).await;
        cx.local.clk_out.set_high();



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
        let sm = ClockMessage::Sync(TempoTimer::now(), ClockSource::ClockIn);
        cx.local.clock_sender_exti9_5.try_send(sm).unwrap();
    }

    #[task(binds = UART4, priority = 3, 
           local = [midi, midi_stream, midi_sender, clock_sender_midi, gates: [u8;2] = [0;2], rx_buffer: [u8; 16] = [0; 16]])]
    fn uart4_handler(cx: uart4_handler::Context) {
        let transfer = cx.local.midi;

        while transfer.is_rx_not_empty() {
            let byte = transfer.read().unwrap();
            cx.local.midi_stream.feed(&[byte], |ev| match ev {
                midly::live::LiveEvent::Midi { channel, message } => {
                    cx.local.midi_sender.try_send((channel.into(), message)).unwrap();
                },
                midly::live::LiveEvent::Common(cmn) => { 
                match cmn {
                    midly::live::SystemCommon::SysEx(_) => todo!(),
                    midly::live::SystemCommon::MidiTimeCodeQuarterFrame(_, _) => todo!(),
                    midly::live::SystemCommon::SongPosition(_) => todo!(),
                    midly::live::SystemCommon::SongSelect(_) => todo!(),
                    midly::live::SystemCommon::TuneRequest => todo!(),
                    midly::live::SystemCommon::Undefined(_, _) => todo!(),
                }
                },
                midly::live::LiveEvent::Realtime(rt) => match rt {
                midly::live::SystemRealtime::TimingClock => {
                        /*
                        cx.local
                            .clock_sender_midi
                            .try_send(ClockMessage::Sync(
                                TempoTimer::now(),
                                ClockSource::MIDI,
                            ))
                            .unwrap();
                            */
                    }
                    midly::live::SystemRealtime::Start => {
                        //cx.local.clock_sender_midi.
                        // try_send(ClockMessage::Sync(TempoTimer::now(),
                        // ClockSource::MIDI)).unwrap();
                    }
                    midly::live::SystemRealtime::Continue => todo!(),
                    midly::live::SystemRealtime::Stop => todo!(),
                    midly::live::SystemRealtime::ActiveSensing => todo!(),
                    midly::live::SystemRealtime::Reset => todo!(),
                    midly::live::SystemRealtime::Undefined(_) => todo!(),
                },
            });
        }
    }

    #[task(binds = EXTI15_10, local = [start_stop_in, clock_sender_exti15_10])]
    fn start_stop_in_handler(cx: start_stop_in_handler::Context) {
        if cx.local.start_stop_in.check_interrupt() {
            cx.local.start_stop_in.clear_interrupt_pending_bit();
            if cx.local.start_stop_in.is_high() {
                cx.local
                    .clock_sender_exti15_10
                    .try_send(ClockMessage::TransportCommand(
                        TransportCommand::Start,
                        ClockSource::ClockIn,
                    ))
                    .unwrap();
            } else {
                cx.local
                    .clock_sender_exti15_10
                    .try_send(ClockMessage::TransportCommand(
                        TransportCommand::Stop,
                        ClockSource::ClockIn,
                    ))
                    .unwrap();
            }
        }
    }
}
