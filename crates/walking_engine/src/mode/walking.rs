use super::{
    catching::{self, Catching},
    kicking::Kicking,
    stopping::Stopping,
    Mode, WalkTransition,
};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};
use types::{
    joints::body::BodyJoints, motion_command::KickVariant, motor_commands::MotorCommands,
    step::Step, support_foot::Side,
};

use crate::{
    kick_state::KickState, step_plan::StepPlan, step_state::StepState, stiffness::Stiffness as _,
    Context,
};

#[derive(
    Clone, Copy, Debug, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct Walking {
    pub step: StepState,
    pub requested_step: Step,
}

impl Walking {
    pub fn new(
        context: &Context,
        requested_step: Step,
        support_side: Side,
        last_requested_step: Step,
    ) -> Self {
        let (backward_acceleration, forward_acceleration) = if last_requested_step.forward > 0.0 {
            (
                -last_requested_step.forward,
                context.parameters.max_forward_acceleration,
            )
        } else if last_requested_step.forward == 0.0 {
            (
                -context.parameters.max_forward_acceleration,
                context.parameters.max_forward_acceleration,
            )
        } else {
            (
                -context.parameters.max_forward_acceleration,
                -last_requested_step.forward,
            )
        };

        let turn_acceleration =
            if last_requested_step.forward.abs() > context.parameters.forward_turn_threshold {
                context.parameters.max_turn_acceleration * context.parameters.forward_turn_reduction
            } else {
                context.parameters.max_turn_acceleration
            };

        let requested_step = Step {
            forward: last_requested_step.forward
                + (requested_step.forward - last_requested_step.forward)
                    .clamp(backward_acceleration, forward_acceleration),
            left: requested_step.left,
            turn: last_requested_step.turn
                + (requested_step.turn - last_requested_step.turn)
                    .clamp(-turn_acceleration, turn_acceleration),
        };
        let plan = StepPlan::new_from_request(context, requested_step, support_side);
        let step = StepState::new(plan);
        Self {
            step,
            requested_step,
        }
    }

    pub fn new_with_step(step: StepState, requested_step: Step) -> Self {
        Self {
            step,
            requested_step,
        }
    }
}

impl WalkTransition for Walking {
    fn stand(self, context: &Context) -> Mode {
        let current_step = self.step;
        if current_step.is_support_switched(context)
            || current_step.is_timeouted(context.parameters)
        {
            return Mode::Stopping(Stopping::new(
                context,
                current_step.plan.support_side.opposite(),
            ));
        }

        if catching::should_catch(
            context,
            self.step.plan.end_feet,
            self.step.plan.support_side,
        ) {
            return Mode::Catching(Catching::new(
                context,
                self.step,
                self.step.plan.support_side,
            ));
        }

        Mode::Walking(self)
    }

    fn walk(self, context: &Context, requested_step: Step) -> Mode {
        let current_step = self.step;

        if current_step.is_support_switched(context) {
            return Mode::Walking(Walking::new(
                context,
                requested_step,
                current_step.plan.support_side.opposite(),
                self.requested_step,
            ));
        }

        if catching::should_catch(
            context,
            self.step.plan.end_feet,
            self.step.plan.support_side,
        ) {
            return Mode::Catching(Catching::new(
                context,
                self.step,
                self.step.plan.support_side,
            ));
        }

        if current_step.is_timeouted(context.parameters) {
            return Mode::Walking(Walking::new(
                context,
                Step::ZERO,
                current_step.plan.support_side.opposite(),
                self.requested_step,
            ));
        }

        Mode::Walking(self)
    }

    fn kick(
        self,
        context: &Context,
        variant: KickVariant,
        kicking_side: Side,
        strength: f32,
    ) -> Mode {
        let current_step = self.step;

        if current_step.is_support_switched(context) {
            let next_support_side = current_step.plan.support_side.opposite();
            // TODO: all kicks require a pre-step
            if next_support_side != kicking_side {
                return Mode::Walking(Walking::new(
                    context,
                    Step::ZERO,
                    next_support_side,
                    self.requested_step,
                ));
            }

            return Mode::Kicking(Kicking::new(
                context,
                KickState::new(variant, kicking_side, strength),
                next_support_side,
            ));
        }

        if catching::should_catch(
            context,
            self.step.plan.end_feet,
            self.step.plan.support_side,
        ) {
            return Mode::Catching(Catching::new(
                context,
                self.step,
                self.step.plan.support_side,
            ));
        }

        if current_step.is_timeouted(context.parameters) {
            return Mode::Walking(Walking::new(
                context,
                Step::ZERO,
                current_step.plan.support_side.opposite(),
                self.requested_step,
            ));
        }

        Mode::Walking(self)
    }
}

impl Walking {
    pub fn compute_commands(&mut self, context: &Context) -> MotorCommands<BodyJoints> {
        let feet = self.step.compute_feet(context);
        self.step.compute_joints(context, feet).apply_stiffness(
            context.parameters.stiffnesses.leg_stiffness_walk,
            context.parameters.stiffnesses.arm_stiffness,
        )
    }

    pub fn tick(&mut self, context: &Context) {
        self.step.tick(context);
    }
}
