
use core::future::{self, Future};

use hal::gpio::OpenDrain;
use defmt::Format;
use stm32f4xx_hal as hal;
use heapless::FnvIndexMap;
use statig::prelude::*;


/// Debouncer taking N number of bytes as input and debounces over a sample size
/// of S. Meaning S consecutive samples have to be the same for the output to
/// change state.
pub struct Debouncer<const N: usize, const S: usize> {
    index: usize,
    debounce_buffer: [[u8; N]; S],
    current_state: [u8; N],
    previous_state: [u8; N],
    changed: [u8; N],
    idle_state: bool,
}
impl<const N: usize, const S: usize> Debouncer<N, S> {
    pub fn new() -> Self {
        Self {
            index: 0,
            debounce_buffer: [[0; N]; S],
            current_state: [0xFF; N],
            previous_state: [0xFF; N],
            changed: [0x00; N],
            idle_state: true,
        }
    }
    /// Takes N bytes and returns N bytes if a state change was detected
    pub fn debounce(&mut self, buf: &[u8; N]) -> Option<[u8; N]> {
        let mut acc: [u8; N] = [0xFF; N];
        self.debounce_buffer[self.index] = *buf;

        self.index += 1;
        if self.index >= S {
            self.index = 0;
        }

        self.debounce_buffer.iter().for_each(|buf| {
            buf.iter().enumerate().for_each(|(i, b)| {
                acc[i] = acc[i] & b;
            })
        });
        if acc == self.current_state {
            None
        }
        else {
            self.previous_state = self.current_state;
            self.current_state = acc;
            self.current_state.iter().enumerate().for_each(|(i, s)| {self.changed[i] = s ^ self.previous_state[i]});
            Some(self.current_state)
        }
    }
    pub fn changed(&self, index: usize) -> bool{
        (self.changed[index >> 3] & (1 << index % 8)) != 0
    }
    pub fn changed_byte(&self, index: usize) -> u8 {
        self.changed[index]
    }
    pub fn pressed(&self, index: usize) -> bool {
        if self.changed(index) {
            return !self.current_state[index >> 3] & (1 << (index % 8)) != 0
        }
        else {
            false
        }
    }
    pub fn released(&self, index: usize) -> bool{
        if self.changed(index) {
            return self.current_state[index >> 3] & (1 << (index % 8)) == 0
        }
        else {
            false
        }
    }

}


trait UiElement<INPUT, OUTPUT> {
    fn update(&mut self, input: INPUT) -> OUTPUT;
}

#[derive(Debug, Copy, PartialEq, Eq, Clone, Format)]
pub enum SwitchId {
    Scene = 0,
    Part = 1,
    Lane = 2,
    Step = 3,
    CopyLoad = 4,
    PasteSave = 5,
    Select = 6,
    Clear = 7,
    StartStop = 8,
    HoldReset = 9,
    Shift = 10,
    Mode = 11,
    A = 12,
    B = 13,
    C = 14,
    D = 15,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Format)]
pub enum SwitchState{
    Pressed,
    Released,
    Idle,
}

struct MomentarySwitch{
    state: SwitchState,
    id: SwitchId,
}
impl MomentarySwitch{
    pub fn new(id: SwitchId) -> Self {
        Self{ state: SwitchState::Idle, id }
    }
    pub fn id(&self) -> usize {
        return self.id as usize
    }
}

pub enum UiEvent{
    SwitchEvent(SwitchId, SwitchState),
}


/// Abstracts a single momentary switch
impl UiElement<SwitchState, UiEvent> for MomentarySwitch {
    fn update(&mut self, s: SwitchState) -> UiEvent {
        self.state = s;
        UiEvent::SwitchEvent(self.id, self.state)
    }
}

struct CmSwitches{
    switches: [MomentarySwitch; 16],
}
impl CmSwitches {
    pub fn new() -> Self{
        Self {
            switches: [
                MomentarySwitch::new(SwitchId::Scene),
                MomentarySwitch::new(SwitchId::Part),
                MomentarySwitch::new(SwitchId::Lane),
                MomentarySwitch::new(SwitchId::Step),
                MomentarySwitch::new(SwitchId::CopyLoad),
                MomentarySwitch::new(SwitchId::PasteSave),
                MomentarySwitch::new(SwitchId::Select),
                MomentarySwitch::new(SwitchId::Clear),
                MomentarySwitch::new(SwitchId::StartStop),
                MomentarySwitch::new(SwitchId::HoldReset),
                MomentarySwitch::new(SwitchId::Shift),
                MomentarySwitch::new(SwitchId::Mode),
                MomentarySwitch::new(SwitchId::A),
                MomentarySwitch::new(SwitchId::B),
                MomentarySwitch::new(SwitchId::C),
                MomentarySwitch::new(SwitchId::D),
            ],
        }
    }
}

pub struct UI<const N: usize> {
    spi: hal::spi::Spi<hal::pac::SPI1>,
    rclk_ld: hal::gpio::Pin<'B', 6, hal::gpio::Output<OpenDrain>>,
    debouncer: Debouncer<N, 4>,
    cm_switches: CmSwitches
}

#[state_machine(initial = "State::idle()")]
impl<const N: usize> UI<N> {
    pub fn new(
        spi: hal::spi::Spi<hal::pac::SPI1>,
        rclk_ld: hal::gpio::Pin<'B', 6, hal::gpio::Output<OpenDrain>>,
    ) -> Self {
        Self {
            spi,
            rclk_ld,
            debouncer: Debouncer::new(),
            cm_switches: CmSwitches::new(),
        }
    }
    #[state]
    fn idle(event: &UiEvent) -> Response<State> {
        match event {
            UiEvent::SwitchEvent(id, s) => {defmt::info!("{}: {}", id, s); Handled},
        }
    }
    #[state]
    fn context_switch(event: &UiEvent) -> Response<State> {
        match event {
            _ => Handled,
        }

    }

    pub fn transfer(&mut self, tx: &[u8; N]) -> Option<[u8; N]> {
        let mut rx_buffer: [u8; N] = [0b0; N];
        self.rclk_ld.set_high();
        self.rclk_ld.set_low();
        self.rclk_ld.set_high();

        self.spi.transfer(&mut rx_buffer, tx).unwrap();
        self.rclk_ld.set_high();
        self.rclk_ld.set_low();
        self.rclk_ld.set_high();
        if let Some(switch_state) = self.debouncer.debounce(&rx_buffer){
            Some(switch_state)
        }
        else {
            None
        }
    }

    pub async fn update<F>(&mut self, f: F)
    where
        F: Fn(UiEvent)
    {
        self.cm_switches.switches.iter_mut().for_each(|sw|{
            if self.debouncer.changed(sw.id()){
               if self.debouncer.pressed(sw.id()){
                    f(sw.update(SwitchState::Pressed));

               }
               else {
                    f(sw.update(SwitchState::Released));
               };
            }
        });
    }
}


#[derive(Copy, Clone, Debug)]
pub enum MyError {
    Spi(),
}
#[derive(Copy, Clone, Debug)]
pub enum MyError2 {
    Spi(),
}
