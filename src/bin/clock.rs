#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(maybe_uninit_uninit_array)]
#![feature(maybe_uninit_array_assume_init)]
#![feature(const_maybe_uninit_uninit_array)]
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
    use hal::{
        ReadFlags, ClearFlags,
        dma::{
            config::DmaConfig, DmaFlag, PeripheralToMemory, Stream2, traits::DmaFlagExt,
            Transfer,
        },
        serial::Rx,
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
    use stm32f4xx_hal::pac::DMA1;

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
    const MIDI_BUFFER_SIZE: usize = 128;

    type MidiRxTransfer = Transfer<
        Stream2<DMA1>,
        4,
        Rx<UART4>,
        PeripheralToMemory,
        &'static mut [u8; MIDI_BUFFER_SIZE],
    >;
    #[shared]
    struct Shared {
        disp: DisplayType,
        clk: Clock<TIMER_FREQ>,
        #[lock_free]
        midi_rx_transfer: MidiRxTransfer,
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
        midi_rx_buffer: Option<&'static mut [u8; MIDI_BUFFER_SIZE]>,
        midi_stream: midly::stream::MidiStream,
        dio: RefCellDevice<'static, Spi3, Pin<'D', 2, Output>, NoDelay>,
        dac: Ad57xxShared<
            RefCellDevice<'static, Spi3, Pin<'A', 15, Output>, NoDelay>,
            ad57xx::marker::Ad57x4,
        >,
    }
    #[init(local = [spi_bus: MaybeUninit<RefCell<Spi3>> = MaybeUninit::uninit()])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("Initializing...");

        // Initialize hardware
        let hardware = control_module::hardware::setup(cx.device);

        defmt::info!("Tempo timer setup");
        // Setup TempoTimer
        TempoTimer::start();
        let (clock_sender, clock_channel_receiver) =
            make_channel!(ClockMessage<TIMER_FREQ>, CAPACITY);
        transport::spawn(clock_channel_receiver).unwrap();

        let (midi_sender, midi_receiver) =
            make_channel!((u8, MidiMessage), CAPACITY);
        midi_msg_handler::spawn(midi_receiver).unwrap();

        defmt::info!("IO setup");
        // Setup digital and analog io
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
        // Note! It is better to use memory pools, such as heapless::pool::Pool.
        // But it not work with embedded_dma yet. See CHANGELOG of unreleased main branch and issue https://github.com/japaric/heapless/pull/362 for details.
        let midi_rx_buffer1 =
            cortex_m::singleton!(: [u8; MIDI_BUFFER_SIZE] = [0; MIDI_BUFFER_SIZE])
                .unwrap();
        let midi_rx_buffer2 =
            cortex_m::singleton!(: [u8; MIDI_BUFFER_SIZE] = [0; MIDI_BUFFER_SIZE])
                .unwrap();

        let mut midi_rx_transfer = Transfer::init_peripheral_to_memory(
            hardware.dma1.2,
            hardware.midi,
            midi_rx_buffer1,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );

        midi_rx_transfer.start(|_rx|{});
        (
            Shared {
                disp: hardware.ui.display,
                clk: Clock::default(),
                midi_rx_transfer,
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
                midi_rx_buffer: Some(midi_rx_buffer2),
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
        gates: [u8;2] = [0;2], 
        notes: MaybeUninit<[FnvIndexMap<u8,u8,8>;16]> = MaybeUninit::uninit()]
        )]
    async fn midi_msg_handler(
        cx: midi_msg_handler::Context,
        mut receiver: Receiver<'static, (u8, MidiMessage), CAPACITY>,
    ) {
        let midi_msg_handler::LocalResources {
            dac,
            gates,
            notes: channel_notes,
            ..
        } = cx.local;
        
        // Assume init is unsafe, we have to initialize the MaybeUnint before using it.
        let ch_notes = unsafe{ channel_notes.assume_init_mut().fill(FnvIndexMap::<u8,u8,8>::new()); channel_notes.assume_init_mut()}; 

        loop {
            while !receiver.is_empty() {
                let (ch, msg) = receiver.recv().await.unwrap();
                        match msg {
                            MidiMessage::NoteOff{key,vel}=>{
                                let(ch, key,vel)=(usize::from(ch), u8::from(key),u8::from(vel));
                                defmt::info!("CH: {}, NOTE OFF: {}, {}",ch,key,vel);
                                //ch_notes[usize::from(ch)].remove(&key);
                                gates[ch/8] &= !(1 << ch%8);
                            },
                            MidiMessage::NoteOn{key,vel}=>{
                                let(ch, key,vel)=(usize::from(ch), u8::from(key),u8::from(vel));
                                if vel==0{
                                    defmt::info!("CH: {}, NOTE OFF: {}, {}",ch,key,vel);
                                    //ch_notes[usize::from(ch)].remove(&key);
                                    gates[ch/8] &= !(1 << ch%8);
                                }
                                else{
                                    defmt::info!("CH: {}, NOTE ON: {}, {}",ch,key,vel);
                                    //ch_notes[usize::from(ch)].insert(key,vel).unwrap();
                                    gates[ch/8] |= 1 << ch%8;
                                }
                            },
                            _ => {},

                }

            }
            //dio.write(gates).unwrap();

            /*
            ch_notes.iter().enumerate().for_each(|(i, notes)|{
                if notes.is_empty(){
                    gates[i/8] &= !(0b1 << i%8);
                }
                else {
                    gates[i/8] |= 0b1 << i%8;
                }
            });
            */
            /*
            ch_notes
                .iter()
                .take(4)
                .enumerate()
                .for_each(|(i, notes)| {
                    if let Some((note, _vel)) = notes.last() {
                    let val: u16 = u16::from(*note)
                        .clamp(0, 120)
                        .checked_mul(546u16)
                        .unwrap();
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
                */
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

    #[task(binds = UART4, priority = 3, local = [dio, midi_stream, midi_sender, clock_sender_midi, midi_rx_buffer, gates: [u8;2] = [0;2]], shared = [midi_rx_transfer])]
    fn uart4_handler(mut cx: uart4_handler::Context) {
        let transfer = &mut cx.shared.midi_rx_transfer;
        let gates = cx.local.gates;

        if transfer.is_idle() {
            let bytes_count =
                MIDI_BUFFER_SIZE - transfer.number_of_transfers() as usize;
            let new_buffer = cx.local.midi_rx_buffer.take().unwrap();

            let (buffer, _) = transfer.next_transfer(new_buffer).unwrap();

            let bytes = &buffer[..bytes_count];

            cx.local.midi_stream.feed(bytes, |ev| match ev {
                midly::live::LiveEvent::Midi { channel, message } => {

                    match message {
                            MidiMessage::NoteOff{key,vel}=>{
                                let(ch, key,vel)=(u8::from(channel), u8::from(key),u8::from(vel));
                                defmt::info!("CH: {}, NOTE OFF: {}, {}",ch,key,vel);
                                //ch_notes[usize::from(ch)].remove(&key);
                                gates[usize::from(ch)/8] &= !(1 << ch%8);
                            },
                            MidiMessage::NoteOn{key,vel}=>{
                                let(ch, key,vel)=(u8::from(channel), u8::from(key),u8::from(vel));
                                if vel==0{
                                    defmt::info!("CH: {}, NOTE OFF: {}, {}",ch,key,vel);
                                    //ch_notes[usize::from(ch)].remove(&key);
                                    gates[usize::from(ch)/8] &= !(1 << ch%8);
                                }
                                    else{
                                    defmt::info!("CH: {}, NOTE ON: {}, {}",ch,key,vel);
                                    //ch_notes[usize::from(ch)].insert(key,vel).unwrap();
                                    gates[usize::from(ch)/8] |= 1 << ch%8;
                                }
                            },
                            _ => {},

                     }
                    cx.local.dio.write(gates).unwrap();
                },
                midly::live::LiveEvent::Common(_) => {}
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

            // Free local buffer
            *cx.local.midi_rx_buffer = Some(buffer);
        }

        /*
        while cx.local.midi.is_rx_not_empty() {
            let c = cx.local.midi.read().unwrap();
            cx.local.midi_stream.feed(&[c], |ev| match ev {
                midly::live::LiveEvent::Midi { channel, message } => {
                    cx.local
                        .midi_sender
                        .try_send((u8::from(channel), message))
                        .unwrap();
                }
                midly::live::LiveEvent::Common(_) => {}
                midly::live::LiveEvent::Realtime(rt) => match rt {
                    midly::live::SystemRealtime::TimingClock => {
                        cx.local.clock_sender_midi.try_send(ClockMessage::Sync(TempoTimer::now(), ClockSource::MIDI)).unwrap();
                    },
                    midly::live::SystemRealtime::Start => {
                        //cx.local.clock_sender_midi.try_send(ClockMessage::Sync(TempoTimer::now(), ClockSource::MIDI)).unwrap();
                    },
                    midly::live::SystemRealtime::Continue => todo!(),
                    midly::live::SystemRealtime::Stop => todo!(),
                    midly::live::SystemRealtime::ActiveSensing => todo!(),
                    midly::live::SystemRealtime::Reset => todo!(),
                    midly::live::SystemRealtime::Undefined(_) => todo!(),
                },
            });
        }
        if cx.local.midi.is_idle() {
            cx.local.midi.clear_idle_interrupt();
            cx.local.midi.unlisten(Event::Idle);
            defmt::info!("uart idle")
        }
        */
    }

    #[task(binds = DMA1_STREAM2, priority = 3, shared = [midi_rx_transfer])]
    fn dma1_stream2(mut cx: dma1_stream2::Context) {
        let transfer = cx.shared.midi_rx_transfer;

        let flags = transfer.flags();
        transfer.clear_flags(DmaFlag::FifoError | DmaFlag::TransferComplete);
            if flags.is_transfer_complete(){
                defmt::info!("NO IDLE")
     // Buffer is full, but no IDLE received!
            // You can process this data or discard data (ignore transfer complete interrupt and wait IDLE).

            // Note! If you want process this data, it is recommended to use double buffering.
            // See Transfer::init_peripheral_to_memory for details.
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
