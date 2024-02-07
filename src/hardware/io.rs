/// Analog and Digital IO abstractions


trait GateOut<const N_BYTES: usize>{

}

/// A CvOut has a number of outputs that can be set to a voltage (f32)
trait CvOut<const N_CHANNELS: usize>{
    fn set_voltage(&mut self, out: usize, v: f32);
    fn v_max() -> f32;
    fn v_min() -> f32;
}
