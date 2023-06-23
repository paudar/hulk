use std::sync::Arc;

use crate::{
    interfake::Interfake,
    structs::{
        control::{AdditionalOutputs, MainOutputs},
        Configuration,
    },
};
use color_eyre::{eyre::WrapErr, Result};
use control::{
    active_vision::{self, ActiveVision},
    ball_state_composer::{self, BallStateComposer},
    behavior::node::{self, Behavior},
    kick_selector::{self, KickSelector},
    motion::look_around::LookAround,
    role_assignment::{self, RoleAssignment},
    rule_obstacle_composer::RuleObstacleComposer,
    world_state_composer::{self, WorldStateComposer},
};
use framework::AdditionalOutput;
use serde::{Deserialize, Serialize};
use serialize_hierarchy::SerializeHierarchy;
use tokio::sync::Notify;

#[derive(Clone, Default, Serialize, Deserialize, SerializeHierarchy)]
pub struct Database {
    pub main_outputs: MainOutputs,
    pub additional_outputs: AdditionalOutputs,
}

pub struct BehaviorCycler {
    hardware_interface: Arc<Interfake>,
    own_changed: Arc<Notify>,
    active_vision: ActiveVision,
    ball_state_composer: BallStateComposer,
    behavior: Behavior,
    kick_selector: KickSelector,
    look_around: LookAround,
    role_assignment: RoleAssignment,
    rule_obstacle_composer: RuleObstacleComposer,
    world_state_composer: WorldStateComposer,
}

impl BehaviorCycler {
    pub fn new(
        hardware_interface: Arc<Interfake>,
        own_changed: Arc<Notify>,
        configuration: &Configuration,
    ) -> Result<Self> {
        let active_vision = ActiveVision::new(active_vision::CreationContext {
            field_dimensions: &configuration.field_dimensions,
        })
        .wrap_err("failed to create node `ActiveVision`")?;
        let ball_state_composer = BallStateComposer::new(ball_state_composer::CreationContext {})
            .wrap_err("failed to create node `BallStateComposer`")?;
        let behavior = Behavior::new(node::CreationContext {
            behavior: &configuration.behavior,
            field_dimensions: &configuration.field_dimensions,
            lost_ball_parameters: &configuration.behavior.lost_ball,
        })
        .wrap_err("failed to create node `Behavior`")?;
        let kick_selector = KickSelector::new(kick_selector::CreationContext {})
            .wrap_err("failed to create node `KickSelector`")?;
        let look_around = control::motion::look_around::LookAround::new(
            control::motion::look_around::CreationContext {
                config: &configuration.look_around,
            },
        )
        .wrap_err("failed to create node `LookAround`")?;
        let role_assignment = RoleAssignment::new(role_assignment::CreationContext {
            forced_role: configuration.role_assignment.forced_role.as_ref(),
            player_number: &configuration.player_number,
            spl_network: &configuration.spl_network,
        })
        .wrap_err("failed to create node `RoleAssignment`")?;
        let rule_obstacle_composer = control::rule_obstacle_composer::RuleObstacleComposer::new(
            control::rule_obstacle_composer::CreationContext {},
        )
        .wrap_err("failed to create node `RuleObstacleComposer`")?;
        let world_state_composer = WorldStateComposer::new(world_state_composer::CreationContext {
            player_number: &configuration.player_number,
        })
        .wrap_err("failed to create node `WorldStateComposer`")?;

        Ok(Self {
            hardware_interface,
            own_changed,

            active_vision,
            ball_state_composer,
            behavior,
            kick_selector,
            look_around,
            role_assignment,
            rule_obstacle_composer,
            world_state_composer,
        })
    }

    pub fn cycle(
        &mut self,
        own_database: &mut Database,
        configuration: &Configuration,
    ) -> Result<()> {
        if own_database
            .main_outputs
            .game_controller_state
            .as_ref()
            .is_some()
        {
            let main_outputs = {
                self.rule_obstacle_composer
                    .cycle(control::rule_obstacle_composer::CycleContext {
                        game_controller_state: own_database
                            .main_outputs
                            .game_controller_state
                            .as_ref()
                            .unwrap(),
                        ball_state: own_database.main_outputs.ball_state.as_ref(),
                        filtered_game_state: own_database
                            .main_outputs
                            .filtered_game_state
                            .as_ref()
                            .unwrap(),
                        field_dimensions: &configuration.field_dimensions,
                    })
                    .wrap_err("failed to execute cycle of node `RuleObstacleComposer`")?
            };
            own_database.main_outputs.rule_obstacles = main_outputs.rule_obstacles.value;
        } else {
            own_database.main_outputs.rule_obstacles = Default::default();
        }
        {
            let main_outputs = self
                .role_assignment
                .cycle(role_assignment::CycleContext {
                    ball_position: own_database.main_outputs.ball_position.as_ref(),
                    fall_state: &own_database.main_outputs.fall_state,
                    game_controller_state: own_database.main_outputs.game_controller_state.as_ref(),
                    primary_state: &own_database.main_outputs.primary_state,
                    robot_to_field: own_database.main_outputs.robot_to_field.as_ref(),
                    cycle_time: &own_database.main_outputs.cycle_time,
                    field_dimensions: &configuration.field_dimensions,
                    forced_role: configuration.role_assignment.forced_role.as_ref(),
                    initial_poses: &configuration.localization.initial_poses,
                    optional_roles: &configuration.behavior.optional_roles,
                    player_number: &configuration.player_number,
                    spl_network: &configuration.spl_network,
                    hulk_messages: &own_database.main_outputs.hulk_messages,
                    hardware: &self.hardware_interface,
                })
                .wrap_err("failed to execute cycle of node `RoleAssignment`")?;
            own_database.main_outputs.team_ball = main_outputs.team_ball.value;
            own_database.main_outputs.network_robot_obstacles =
                main_outputs.network_robot_obstacles.value;
            own_database.main_outputs.role = main_outputs.role.value;
        }
        {
            let main_outputs = self
                .ball_state_composer
                .cycle(ball_state_composer::CycleContext {
                    ball_position: own_database.main_outputs.ball_position.as_ref(),
                    penalty_shot_direction: own_database
                        .main_outputs
                        .penalty_shot_direction
                        .as_ref(),
                    robot_to_field: own_database.main_outputs.robot_to_field.as_ref(),
                    team_ball: own_database.main_outputs.team_ball.as_ref(),
                    primary_state: &own_database.main_outputs.primary_state,
                    field_dimensions: &configuration.field_dimensions,
                    game_controller_state: own_database.main_outputs.game_controller_state.as_ref(),
                })
                .wrap_err("failed to execute cycle of node `BallStateComposer`")?;
            own_database.main_outputs.ball_state = main_outputs.ball_state.value;
            own_database.main_outputs.rule_ball_state = main_outputs.rule_ball_state.value;
        }

        {
            let main_outputs = self
                .active_vision
                .cycle(active_vision::CycleContext {
                    ball: own_database.main_outputs.ball_state.as_ref(),
                    rule_ball: own_database.main_outputs.ball_state.as_ref(),
                    cycle_time: &own_database.main_outputs.cycle_time,
                    obstacles: &own_database.main_outputs.obstacles,
                    parameters: &configuration.behavior.look_action,
                    robot_to_field: own_database.main_outputs.robot_to_field.as_ref(),
                })
                .wrap_err("failed to execute cycle of node `ActiveVision`")?;
            own_database.main_outputs.position_of_interest =
                main_outputs.position_of_interest.value;
        }
        {
            if own_database.main_outputs.robot_to_field.as_ref().is_some()
                && own_database.main_outputs.ball_state.as_ref().is_some()
            {
                let main_outputs = {
                    self.kick_selector
                        .cycle(control::kick_selector::CycleContext {
                            robot_to_field: own_database
                                .main_outputs
                                .robot_to_field
                                .as_ref()
                                .unwrap(),
                            ball_state: own_database.main_outputs.ball_state.as_ref().unwrap(),
                            obstacles: &own_database.main_outputs.obstacles,
                            field_dimensions: &configuration.field_dimensions,
                            in_walk_kicks: &configuration.in_walk_kicks,
                            angle_distance_weight: &configuration
                                .kick_selector
                                .angle_distance_weight,
                            max_kick_around_obstacle_angle: &configuration
                                .kick_selector
                                .max_kick_around_obstacle_angle,
                            kick_pose_obstacle_radius: &configuration
                                .kick_selector
                                .kick_pose_obstacle_radius,
                            ball_radius_for_kick_target_selection: &configuration
                                .kick_selector
                                .ball_radius_for_kick_target_selection,
                            closer_threshold: &configuration.kick_selector.closer_threshold,
                            find_kick_targets: &configuration.kick_selector.find_kick_targets,
                            kick_targets: framework::AdditionalOutput::new(
                                true,
                                &mut own_database.additional_outputs.kick_targets,
                            ),
                            instant_kick_targets: framework::AdditionalOutput::new(
                                true,
                                &mut own_database.additional_outputs.instant_kick_targets,
                            ),
                            default_kick_strength: &configuration
                                .kick_selector
                                .default_kick_strength,
                            corner_kick_strength: &configuration.kick_selector.corner_kick_strength,
                        })
                        .wrap_err("failed to execute cycle of node `KickSelector`")?
                };
                own_database.main_outputs.kick_decisions = main_outputs.kick_decisions.value;
                own_database.main_outputs.instant_kick_decisions =
                    main_outputs.instant_kick_decisions.value;
            } else {
                own_database.main_outputs.kick_decisions = Default::default();
                own_database.main_outputs.instant_kick_decisions = Default::default();
            }
        }
        {
            let main_outputs = self
                .world_state_composer
                .cycle(world_state_composer::CycleContext {
                    ball: own_database.main_outputs.ball_state.as_ref(),
                    filtered_game_state: own_database.main_outputs.filtered_game_state.as_ref(),
                    game_controller_state: own_database.main_outputs.game_controller_state.as_ref(),
                    penalty_shot_direction: own_database
                        .main_outputs
                        .penalty_shot_direction
                        .as_ref(),
                    robot_to_field: own_database.main_outputs.robot_to_field.as_ref(),
                    kick_decisions: own_database.main_outputs.kick_decisions.as_ref(),
                    instant_kick_decisions: own_database
                        .main_outputs
                        .instant_kick_decisions
                        .as_ref(),
                    player_number: &configuration.player_number,
                    fall_state: &own_database.main_outputs.fall_state,
                    has_ground_contact: &own_database.main_outputs.has_ground_contact,
                    obstacles: &own_database.main_outputs.obstacles,
                    primary_state: &own_database.main_outputs.primary_state,
                    role: &own_database.main_outputs.role,
                    position_of_interest: &own_database.main_outputs.position_of_interest,
                    rule_ball: own_database.main_outputs.rule_ball_state.as_ref(),
                    rule_obstacles: &own_database.main_outputs.rule_obstacles,
                })
                .wrap_err("failed to execute cycle of node `WorldStateComposer`")?;
            own_database.main_outputs.world_state = main_outputs.world_state.value;
        }
        {
            let main_outputs = self
                .behavior
                .cycle(node::CycleContext {
                    path_obstacles: AdditionalOutput::new(
                        true,
                        &mut own_database.additional_outputs.path_obstacles,
                    ),
                    active_action: AdditionalOutput::new(
                        true,
                        &mut own_database.additional_outputs.active_action,
                    ),
                    world_state: &own_database.main_outputs.world_state,
                    cycle_time: &own_database.main_outputs.cycle_time,
                    configuration: &configuration.behavior,
                    in_walk_kicks: &configuration.in_walk_kicks,
                    field_dimensions: &configuration.field_dimensions,
                    lost_ball_parameters: &configuration.behavior.lost_ball,
                    intercept_ball_parameters: &configuration.behavior.intercept_ball,
                    has_ground_contact: &true,
                })
                .wrap_err("failed to execute cycle of node `Behavior`")?;
            own_database.main_outputs.motion_command = main_outputs.motion_command.value;
        }
        {
            let main_outputs = {
                self.look_around
                    .cycle(control::motion::look_around::CycleContext {
                        config: &configuration.look_around,
                        motion_command: &own_database.main_outputs.motion_command,
                        sensor_data: &own_database.main_outputs.sensor_data,
                        cycle_time: &own_database.main_outputs.cycle_time,
                        current_mode: AdditionalOutput::new(
                            true,
                            &mut own_database.additional_outputs.look_around_mode,
                        ),
                    })
                    .wrap_err("failed to execute cycle of node `LookAround`")?
            };
            own_database.main_outputs.look_around = main_outputs.look_around.value;
        }
        self.own_changed.notify_one();
        Ok(())
    }
}
