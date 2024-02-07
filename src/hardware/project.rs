use fugit::{TimerDurationU64, TimerInstantU64};
use rtic_time::TimerQueue;
//use core::marker::PhantomData;
//use core::str::FromStr;
//use heapless::FnvIndexMap;
//use heapless::Entry;
//use bitfield_struct::bitfield;
//use heapless::{String, IndexMap};

use super::timer::TempoTimer;
use super::timer::TIMER_FREQ;
use defmt::*;

pub struct Note {
}


/// A generator generates either an analog (CV) or digital (TRIG/GATE) signal
///
/// Digital generator types:
///  - Sequencer
///  - Clock/TimeBase
///  - Euclidian
///  - Random
///
///  Analog generator types:
///   - Sequencer
///   - LFO
///   - Harmony
///   - ...

trait Generator {
    async fn next(){}
}


type Duration = TimerDurationU64<TIMER_FREQ>;
type Instant = TimerInstantU64<TIMER_FREQ>;

pub struct Clock{ 
    pub ticks: u32,
    last_instant: Option<Instant>,
    duration: Option<Duration>,
    timer: &'static TimerQueue<TempoTimer>,
}
impl Clock {
    pub fn new(timer: &'static TimerQueue<TempoTimer>) -> Self {
        Self {
            ticks: 0,
            last_instant: None,
            duration: None,
            timer,
        }
        
    }
    pub fn sync(&mut self, inst: Instant, dur: Duration, div: u32){
        self.ticks = 0;
        self.last_instant = Some(inst);
        self.duration = Some(dur / div);
    }
    pub async fn divide(&mut self){
        if let (Some(last), Some(dur)) = (self.last_instant, self.duration) {
            if let Some(next_instant) = last.checked_add_duration(dur){
                self.ticks += 1;
                self.last_instant = Some(next_instant);
                self.timer.delay_until(next_instant).await
            } else {
                error!("Error adding duration")
            }
        }
    }

}


/*
/// A "Project" holds all transient data
pub struct Project<'a> {
    name: String<16>,
    parts: FnvIndexMap<String<8>,Part,16>,
    lanes: FnvIndexMap<String<8>,Lane,128>,
    phantom: PhantomData<&'a bool>
}
#[derive(Debug)]
pub struct Part{
    linked_lane: String<8>,
}

impl<'a> Project<'a> {
    pub fn new() -> Self {
        let mut lanes = FnvIndexMap::<String<8>, Lane, 128>::new();
        let mut parts = FnvIndexMap::<String<8>, Part, 16>::new();

        let s = String::from_str("S1").unwrap();
        lanes.insert(s, Lane::TrigSequence{steps: [0xFFFF; 64]}).unwrap();
        lanes.insert(String::from_str("G1").unwrap(), Lane::Generator{t: Generator::Lfo{waveform: 0}}).unwrap();

        if let Some((k, v)) = lanes.first(){
            let key = String::from_str(k).unwrap();
            parts.insert(String::from_str("P1").unwrap(), Part{linked_lane: key}).unwrap();
        }

        Self {
            name: String::from_str("Unnamed Project").unwrap(),
            lanes,
            parts,
            phantom: PhantomData,
        }
    }
}
#[derive(Debug)]
pub enum Generator{
    Clock,
    Pattern,
    Lfo{waveform:u8},
}


#[derive(Debug)]
pub enum Lane{
    TrigSequence{
        steps: [u16; 64],
    },
    Generator{
        t: Generator,
    },
}
*/


