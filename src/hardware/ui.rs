use super::display::DisplayType;
use super::switches::{LedId, LedState};
use super::switches::{SwitchId, SwitchState, Switches, UiEvent};
use defmt::Format;
use statig::prelude::*;

#[derive(Clone, Copy, Eq, PartialEq, Format, Debug)]
enum View {
    Switch(&'static View),
    Project,
    Part,
    Lane,
    Step,
    Overview,
}

pub struct UI {
    pub switches: Switches<6>,
    pub display: DisplayType,
}

pub struct UiStateMachine {
    view: View,
}

#[state_machine(initial = "State::idle()")]
impl UiStateMachine {
    pub fn new() -> Self {
        Self {
            view: View::Overview,
        }
    }
    #[state]
    fn idle(event: &UiEvent) -> Response<State> {
        match event {
            UiEvent::SwitchEvent(id, s) => {
                defmt::info!("{}: {}", id, s);
                if *s == SwitchState::Pressed {
                    match *id {
                        SwitchId::Project
                        | SwitchId::Part
                        | SwitchId::Lane
                        | SwitchId::Step => Transition(State::view_switch()),
                        SwitchId::CopyLoad => todo!(),
                        SwitchId::PasteSave => todo!(),
                        SwitchId::Select => todo!(),
                        SwitchId::Clear => todo!(),
                        SwitchId::StartStop => todo!(),
                        SwitchId::HoldReset => todo!(),
                        SwitchId::Shift => todo!(),
                        SwitchId::Mode => todo!(),
                        SwitchId::A => todo!(),
                        SwitchId::B => todo!(),
                        SwitchId::C => todo!(),
                        SwitchId::D => todo!(),
                    }
                } else {
                    Handled
                }
            }
        }
    }
    #[state]
    fn view_switch(
        &mut self,
        event: &UiEvent,
        context: &mut UI,
    ) -> Response<State> {
        let view = match event {
            UiEvent::SwitchEvent(id, _) => match id {
                SwitchId::Project => {
                    context.switches.set_led(&LedId::Project, LedState::On);
                    View::Project
                }
                SwitchId::Part => {
                    context.switches.set_led(&LedId::Part, LedState::On);
                    View::Part
                }
                SwitchId::Lane => {
                    context.switches.set_led(&LedId::Lane, LedState::On);
                    View::Lane
                }
                SwitchId::Step => {
                    context.switches.set_led(&LedId::Step, LedState::On);
                    View::Step
                }
                _ => self.view,
            },
        };
        Handled
    }
}
