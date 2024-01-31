use core::hash::Hash;
use core::marker::PhantomData;

use crate::hardware::hal as hal;
use crate::hardware::debounce::Debouncer;
use defmt::Format;
use heapless::FnvIndexMap;

#[derive(Debug, Copy, PartialEq, Eq, Clone, Format, Hash)]
pub enum SwitchId {
    Project = 0,
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

#[derive(Debug, Copy, PartialEq, Eq, Clone, Format, Hash)]
pub enum LedId {
    Project = 0,
    Part = 1,
    Lane = 2,
    Step = 3,
    P1 = 4,
    P2 = 5,
    P3 = 6,
    P4 = 7,
    Play = 8,
    Stop = 9,
    Reset = 10,
    Small = 11,
    A = 12,
    B = 13,
    C = 14,
    D = 15,
}
#[derive(Debug, Copy, Clone, PartialEq, Eq, Format)]
pub enum LedState {
    Off,
    On,
    Dim,
    Strobe,
}
#[derive(Debug, Copy, Clone, PartialEq, Eq, Format)]
pub enum SwitchState {
    Pressed,
    Released,
    Idle,
}
#[derive(Debug, Copy, Clone, PartialEq, Eq, Format)]
pub enum UiEvent {
    SwitchEvent(SwitchId, SwitchState),
}

trait UiElement<ID, S> {
    fn new(id: ID) -> Self;
    fn update(&mut self, input: S) -> S;
    fn index(&self) -> usize;
}
struct MomentarySwitch {
    state: SwitchState,
    id: SwitchId,
}
impl MomentarySwitch {
    pub fn id(&self) -> SwitchId {
        return self.id;
    }
}
#[derive(Copy, Clone, Debug)]
struct Led {
    state: LedState,
    id: LedId,
}
impl UiElement<LedId, LedState> for Led {
    fn new(id: LedId) -> Self {
        Self {
            state: LedState::Off,
            id
        }
    }
    fn update(&mut self, s: LedState) -> LedState {
        self.state = s;
        self.state
    }
    fn index(&self) -> usize {
        self.id as usize
    }
}
/// Abstracts a single momentary switch
impl UiElement<SwitchId, SwitchState> for MomentarySwitch {
    fn new(id: SwitchId) -> Self {
        Self {
            state: SwitchState::Idle,
            id,
        }
    }
    fn update(&mut self, s: SwitchState) -> SwitchState {
        self.state = s;
        self.state
    }
    fn index(&self) -> usize {
        return self.id as usize;
    }
}

struct UiElementArray<const N: usize, ID, EL, S>
{
    elements: FnvIndexMap<ID, EL, N> ,
    s: PhantomData<S>
}

impl<const N: usize, ID, EL, S> UiElementArray<N, ID, EL, S> where
    EL: UiElement<ID, S> + Copy + core::fmt::Debug,
    ID: Copy + PartialEq + Eq + Hash + core::fmt::Debug
{
    fn new(ids: [ID; N], el: EL) -> Self {
        let mut e = FnvIndexMap::<ID, EL, N>::new();

        ids.iter().for_each(|id|{
            e.insert(*id, EL::new(*id)).unwrap();
        });
        
        Self {
           elements: e,
           s: PhantomData,
        }
        
    }
}

pub struct Switches<const N: usize> {
    spi: hal::spi::Spi<hal::pac::SPI1>,
    rclk_ld: hal::gpio::Pin<'B', 6, hal::gpio::Output<hal::gpio::OpenDrain>>,
    debouncer: Debouncer<N, 4>,
    cm_switches: [MomentarySwitch; 16],
    leds: UiElementArray<16, LedId, Led, LedState>,
    tx_buffer: [u8; N],
}
impl<const N: usize> Switches<N> {
    pub fn new(
        spi: hal::spi::Spi<hal::pac::SPI1>,
        rclk_ld: hal::gpio::Pin<'B', 6, hal::gpio::Output<hal::gpio::OpenDrain>>,
    ) -> Self {
        Self {
            spi,
            rclk_ld,
            debouncer: Debouncer::new(),
            cm_switches: [
                MomentarySwitch::new(SwitchId::Project),
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
            leds: UiElementArray::new(
                [LedId::Project, LedId::Part, LedId::Lane, LedId::Step, 
                LedId::P1, LedId::P2, LedId::P3, LedId::P4,
                LedId::Play, LedId::Stop, LedId::Reset, LedId::Small,
                LedId::A, LedId::B, LedId::C, LedId::D],
                Led::new(LedId::Project)),
            tx_buffer: [0x00; N],
        }
    }
    pub fn transfer(&mut self) -> Option<[u8; N]> {
        self.leds.elements.iter().for_each(|led|{
            let i = led.1.index();
            if led.1.state == LedState::On {
                self.tx_buffer[i >> 3] |= 1 << (i % 8);
            }
            else {
                self.tx_buffer[i >> 3] &= !(1 << (i % 8));
            }
        });

        let mut rx_buffer: [u8; N] = [0b0; N];
        let mut tx_buffer = self.tx_buffer;
        tx_buffer.reverse();
        self.rclk_ld.set_high();
        self.rclk_ld.set_low();
        self.rclk_ld.set_high();



        self.spi.transfer(&mut rx_buffer, &tx_buffer).unwrap();
        self.rclk_ld.set_high();
        self.rclk_ld.set_low();
        self.rclk_ld.set_high();
        if let Some(switch_state) = self.debouncer.debounce(&rx_buffer) {
            Some(switch_state)
        } else {
            None
        }
    }

    pub fn update<F>(&mut self, mut f: F)
    where
        F: FnMut(UiEvent),
    {
        self.cm_switches.iter_mut().for_each(|sw| {
            if self.debouncer.changed(sw.index()) {
                if self.debouncer.pressed(sw.index()) {
                    f(UiEvent::SwitchEvent(sw.id(), sw.update(SwitchState::Pressed)));
                } else {
                    f(UiEvent::SwitchEvent(sw.id(), sw.update(SwitchState::Released)));
                };
            } else {
            }
        });
    }

    pub fn set_led(&mut self, id: &LedId, s: LedState){
        if let Some(led) = self.leds.elements.get_mut(id){
            led.update(s);
        }
    }
}
