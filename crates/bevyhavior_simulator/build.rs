use color_eyre::eyre::{Result, WrapErr};

use code_generation::{generate, write_to_file::WriteToFile, ExecutionMode};
use source_analyzer::{
    cyclers::{CyclerKind, Cyclers},
    manifest::{CyclerManifest, FrameworkManifest},
    pretty::to_string_pretty,
    structs::Structs,
};

fn main() -> Result<()> {
    let manifest = FrameworkManifest {
        cyclers: vec![
            CyclerManifest {
                name: "Control",
                kind: CyclerKind::RealTime,
                instances: vec![""],
                setup_nodes: vec!["crate::fake_data"],
                nodes: vec![
                    "control::active_vision",
                    "control::ball_state_composer",
                    "control::behavior::node",
                    "control::center_of_mass_provider",
                    "control::dribble_path_planner",
                    "control::filtered_game_controller_state_timer",
                    "control::free_kick_signal_filter",
                    "control::game_controller_state_filter",
                    "control::ground_provider",
                    "control::kick_selector",
                    "control::kinematics_provider",
                    "control::motion::look_around",
                    "control::motion::motion_selector",
                    "control::motion::step_planner",
                    "control::motion::walking_engine",
                    "control::motion::walk_manager",
                    "control::odometry",
                    "control::penalty_shot_direction_estimation",
                    "control::primary_state_filter",
                    "control::ready_signal_detection_filter",
                    "control::referee_position_provider",
                    "control::role_assignment",
                    "control::rule_obstacle_composer",
                    "control::search_suggestor",
                    "control::support_foot_estimation",
                    "control::team_ball_receiver",
                    "control::time_to_reach_kick_position",
                    "control::world_state_composer",
                ],
                execution_time_warning_threshold: None,
            },
            CyclerManifest {
                name: "SplNetwork",
                kind: CyclerKind::Perception,
                instances: vec![""],
                setup_nodes: vec!["spl_network::message_receiver"],
                nodes: vec!["spl_network::message_filter"],
                execution_time_warning_threshold: None,
            },
            CyclerManifest {
                name: "ObjectDetection",
                kind: CyclerKind::Perception,
                instances: vec!["Top"],
                setup_nodes: vec!["vision::image_receiver"],
                nodes: vec![
                    "object_detection::pose_detection",
                    "object_detection::pose_filter",
                    "object_detection::pose_interpretation",
                ],
                execution_time_warning_threshold: None,
            },
        ],
    };
    let root = "../../crates/";

    let cyclers = Cyclers::try_from_manifest(manifest, root)?;
    for path in cyclers.watch_paths() {
        println!("cargo:rerun-if-changed={}", path.display());
    }

    println!();
    println!("{}", to_string_pretty(&cyclers)?);

    let structs = Structs::try_from_cyclers(&cyclers)?;
    generate(&cyclers, &structs, ExecutionMode::Run)
        .write_to_file("generated_code.rs")
        .wrap_err("failed to write generated code to file")
}
