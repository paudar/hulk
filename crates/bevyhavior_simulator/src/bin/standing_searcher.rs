use bevy::prelude::*;

use linear_algebra::{point, vector};
use scenario::scenario;
use spl_network_messages::{GameState, PlayerNumber};

use bevyhavior_simulator::{
    ball::BallResource,
    game_controller::GameControllerCommand,
    robot::Robot,
    time::{Ticks, TicksTime},
};
use types::ball_position::SimulatorBallState;

#[scenario]
fn standing_searcher(app: &mut App) {
    app.add_systems(Startup, startup);
    app.add_systems(Update, update);
}

fn startup(
    mut commands: Commands,
    mut game_controller_commands: EventWriter<GameControllerCommand>,
) {
    for number in [
        PlayerNumber::One,
        PlayerNumber::Two,
        PlayerNumber::Three,
        PlayerNumber::Seven,
    ] {
        commands.spawn(Robot::new(number));
    }
    game_controller_commands.send(GameControllerCommand::SetGameState(GameState::Ready));
}

fn update(time: Res<Time<Ticks>>, mut ball: ResMut<BallResource>, mut exit: EventWriter<AppExit>) {
    if time.ticks() == 4200 {
        ball.state = None;
    }
    if time.ticks() == 5000 {
        ball.state = Some(SimulatorBallState {
            position: point![-2.7, -0.2],
            velocity: vector![0.0, 0.0],
        });
    }
    if time.ticks() == 5500 {
        ball.state = None;
    }
    if time.ticks() >= 10_000 {
        println!("Done");
        exit.send(AppExit::Success);
    }
}
