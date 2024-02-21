/// Debouncer taking N number of bytes as input and debounces over a sample size
/// of S. Meaning S consecutive samples have to be the same for the output to
/// change state. A high idle state is assumed.

pub struct Debouncer<const N: usize, const S: usize> {
    index: usize,
    debounce_buffer: [[u8; N]; S],
    current_state: [u8; N],
    previous_state: [u8; N],
    changed: [u8; N],
}
impl<const N: usize, const S: usize> Debouncer<N, S> {
    pub fn new() -> Self {
        Self {
            index: 0,
            debounce_buffer: [[0xFF; N]; S],
            current_state: [0xFF; N],
            previous_state: [0xFF; N],
            changed: [0x00; N],
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
        } else {
            self.previous_state = self.current_state;
            self.current_state = acc;
            self.current_state.iter().enumerate().for_each(|(i, s)| {
                self.changed[i] = s ^ self.previous_state[i]
            });
            Some(self.current_state)
        }
    }
    pub fn changed(&self, index: usize) -> bool {
        (self.changed[index >> 3] & (1 << index % 8)) != 0
    }
    pub fn pressed(&self, index: usize) -> bool {
        if self.changed(index) {
            (!self.current_state[index >> 3] & (1 << (index % 8))) != 0
        } else {
            false
        }
    }
    pub fn released(&self, index: usize) -> bool {
        if self.changed(index) {
            self.current_state[index >> 3] & (1 << (index % 8)) == 0
        } else {
            false
        }
    }
}
