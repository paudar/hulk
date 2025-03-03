import time

import click
import numpy as np
from mujoco_interactive_viewer import Viewer
from nao_env import NaoWalking
from stable_baselines3 import PPO


@click.command()
@click.option(
    "--throw-tomatoes", is_flag=True, help="Throw tomatoes at the Nao."
)
@click.option(
    "--load-policy",
    type=click.STRING,
    default=None,
    help="Load a policy from a file.",
)
def main(*, throw_tomatoes: bool, load_policy: str | None) -> None:
    env = NaoWalking(throw_tomatoes=throw_tomatoes)
    action_space_size = env.action_space.shape[0]
    action = np.zeros(action_space_size)
    _, _, _, _, infos = env.step(action)
    env.reset()

    model = None
    if load_policy is not None:
        model = PPO.load(load_policy)

    dt = env.dt

    env.initialize_terrain(max_height=0.1, step_height=0.01)

    viewer = Viewer(env.model, env.data)
    rewards_figure = viewer.figure("rewards")
    rewards_figure.set_title("Rewards")
    rewards_figure.set_x_label("Step")
    for key in infos:
        rewards_figure.add_line(key)

    total_reward_figure = viewer.figure("total_reward")
    total_reward_figure.add_line("Total Reward")
    total_reward_figure.line_color("Total Reward", red=0.0, green=0.0, blue=1.0)
    total_reward_figure.set_x_label("Step")

    total_reward = 0.0

    fsr_figure = viewer.figure("fsr")
    fsr_figure.set_title("FSR")
    fsr_figure.set_x_label("Step")
    fsr_figure.add_line("Left FSR")
    fsr_figure.add_line("Right FSR")

    gyro_figure = viewer.figure("gyro")
    gyro_figure.set_title("Gyroscope")
    gyro_figure.set_x_label("Step")
    gyro_figure.add_line("X Gyro")
    gyro_figure.add_line("Y Gyro")
    gyro_figure.add_line("Z Gyro")

    while viewer.is_alive:
        start_time = time.time()
        # viewer.track_with_camera("Nao")
        observation, reward, _terminated, _truncated, infos = env.step(action)
        if model:
            action, _ = model.predict(observation, deterministic=True)

        fsr_figure.push_data_to_line("Left FSR", env.nao.left_fsr().sum())
        fsr_figure.push_data_to_line("Right FSR", env.nao.right_fsr().sum())

        gyro_figure.push_data_to_line("X Gyro", env.nao.gyroscope()[0])
        gyro_figure.push_data_to_line("Y Gyro", env.nao.gyroscope()[1])
        gyro_figure.push_data_to_line("Z Gyro", env.nao.gyroscope()[2])

        total_reward += reward

        for key, value in infos.items():
            rewards_figure.push_data_to_line(key, value)

        total_reward_figure.push_data_to_line("Total Reward", total_reward)

        viewer.render()
        end_time = time.time()
        wait_time = max(0, dt - (end_time - start_time))
        time.sleep(wait_time)


if __name__ == "__main__":
    main()
