use atomic_polyfill::{AtomicU64, Ordering};
use fugit::{TimerDurationU64, TimerInstantU64};
use rtic_time::half_period_counter::calculate_now;
use rtic_time::Monotonic;
use rtic_time::TimeoutError;
use rtic_time::TimerQueue;
use stm32f4xx_hal::pac::Interrupt;

use super::hal;

pub const TIMER_FREQ: u32 = 84_000_000;

static OVERFLOW_COUNTER: AtomicU64 = AtomicU64::new(0);
static TIMER_QUEUE: TimerQueue<TempoTimer> = TimerQueue::<TempoTimer>::new();

pub struct TempoTimer;
impl TempoTimer {
    pub fn start() {
        Self::timer().arr.write(|w| w.arr().bits(u32::MAX));

        Self::timer()
            .ccr2()
            .modify(|_, w| w.ccr().bits(u32::MAX - (u32::MAX >> 1)));

        Self::timer().dier.write(|w| {
            // Enable update (overflow) and half-period interrupts
            w.uie().enabled().cc2ie().enabled()
        });

        Self::timer().egr.write(|r| r.ug().set_bit());
        Self::timer().sr.modify(|_, w| w.uif().clear_bit());

        TIMER_QUEUE.initialize(Self {});
        OVERFLOW_COUNTER.store(0, Ordering::SeqCst);

        Self::timer().cr1.write(|w| w.ckd().div1().cen().enabled());
    }

    fn timer() -> &'static hal::pac::tim2::RegisterBlock {
        unsafe { &*hal::pac::TIM2::ptr() }
    }
    fn read_counter() -> u32 {
        Self::timer().cnt.read().cnt().bits()
    }

    pub fn __tq() -> &'static TimerQueue<TempoTimer> {
        &TIMER_QUEUE
    }
    #[inline]
    pub async fn delay(duration: <Self as Monotonic>::Duration) {
        TIMER_QUEUE.delay(duration).await;
    }
    /// Timeout at a specific time
    pub async fn timeout_at<F: core::future::Future>(
        instant: <Self as rtic_time::Monotonic>::Instant,
        future: F,
    ) -> Result<F::Output, TimeoutError> {
        TIMER_QUEUE.timeout_at(instant, future).await
    }
    pub async fn timeout_after<F: core::future::Future>(
        duration: <Self as rtic_time::Monotonic>::Duration,
        future: F,
    ) -> Result<F::Output, TimeoutError> {
        TIMER_QUEUE.timeout_after(duration, future).await
    }
    #[inline]
    pub async fn delay_until(instant: <Self as rtic_time::Monotonic>::Instant) {
        TIMER_QUEUE.delay_until(instant).await
    }
}

impl Monotonic for TempoTimer {
    const ZERO: Self::Instant = TimerInstantU64::from_ticks(0);

    const TICK_PERIOD: Self::Duration =
        TimerDurationU64::MHz(TIMER_FREQ as u64);

    type Instant = TimerInstantU64<TIMER_FREQ>;
    type Duration = TimerDurationU64<TIMER_FREQ>;

    #[inline(always)]
    fn now() -> Self::Instant {
        Self::Instant::from_ticks(calculate_now(
            || OVERFLOW_COUNTER.load(Ordering::Relaxed),
            || Self::read_counter(),
        ))
    }

    fn set_compare(instant: Self::Instant) {
        let now = Self::now();

        let val = match instant.checked_duration_since(now) {
            None => 0,
            Some(x) if x.ticks() <= (u32::MAX as u64) => {
                instant.duration_since_epoch().ticks() as u32
            }
            Some(_x) => 0,
        };

        Self::timer().ccr1().write(|r| r.ccr().bits(val));
    }

    fn clear_compare_flag() {
        Self::timer().sr.modify(|_, w| w.cc1if().clear_bit());
    }
    fn pend_interrupt() {
        cortex_m::peripheral::NVIC::pend(Interrupt::TIM2);
    }
    fn enable_timer() {
        Self::timer().dier.modify(|_, w| w.cc1ie().set_bit());
    }
    fn disable_timer() {
        Self::timer().dier.modify(|_, w| w.cc1ie().clear_bit());
    }
    fn on_interrupt() {
        if Self::timer().sr.read().uif().bit() {
            Self::timer().sr.modify(|_, w| w.uif().clear_bit());
            let prev = OVERFLOW_COUNTER.fetch_add(1, Ordering::Relaxed);
            assert!(prev % 2 == 1, "Tempo Monotonic missed interrupt");
        }
        if Self::timer().sr.read().cc2if().bit() {
            Self::timer().sr.modify(|_, w| w.cc2if().clear_bit());
            let prev = OVERFLOW_COUNTER.fetch_add(1, Ordering::Relaxed);
            assert!(prev % 2 == 0, "Tempo Monotonic missed interrupt");
        }
    }
}
