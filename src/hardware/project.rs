use core::marker::PhantomData;
use core::str::FromStr;
use heapless::FnvIndexMap;
use heapless::Entry;
use bitfield_struct::bitfield;
use heapless::{String, IndexMap};


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

