from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

import pybullet as p
import pybullet_data


@dataclass(frozen=True)
class SimulationConfig:
    mode: str
    seconds: float
    time_step: float
    robot: str
    realtime: bool
    inspect_joints: bool
    log_every: float
    pause_at_end: bool


def parse_args() -> SimulationConfig:
    parser = argparse.ArgumentParser(description="Run a small PyBullet simulation.")
    parser.add_argument("--mode", choices=("gui", "direct"), default="gui")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--time-step", type=float, default=1.0 / 240.0)
    parser.add_argument("--robot", choices=("r2d2", "kuka"), default="r2d2")
    parser.add_argument(
        "--inspect-joints",
        action="store_true",
        help="Print the robot joint table before the simulation starts.",
    )
    parser.add_argument(
        "--log-every",
        type=float,
        default=0.0,
        help="Print base position/orientation every N simulated seconds. 0 disables logs.",
    )
    parser.add_argument(
        "--pause-at-end",
        action="store_true",
        help="Keep the GUI window open after the simulation finishes until Enter is pressed.",
    )
    parser.add_argument(
        "--no-realtime",
        action="store_true",
        help="Run as fast as possible instead of pacing steps to wall-clock time.",
    )
    args = parser.parse_args()

    return SimulationConfig(
        mode=args.mode,
        seconds=args.seconds,
        time_step=args.time_step,
        robot=args.robot,
        realtime=not args.no_realtime,
        inspect_joints=args.inspect_joints,
        log_every=args.log_every,
        pause_at_end=args.pause_at_end,
    )


def connect(mode: str) -> int:
    connection_mode = p.GUI if mode == "gui" else p.DIRECT
    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError("Could not connect to PyBullet.")
    return client_id


def setup_world(config: SimulationConfig) -> int:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(config.time_step)

    p.loadURDF("plane.urdf")
    create_colored_blocks()

    if config.robot == "kuka":
        robot_id = p.loadURDF(
            "kuka_iiwa/model.urdf",
            basePosition=(0, 0, 0.02),
            useFixedBase=True,
        )
        p.resetDebugVisualizerCamera(
            cameraDistance=1.8,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=(0.25, 0, 0.35),
        )
    else:
        robot_id = p.loadURDF("r2d2.urdf", basePosition=(0, 0, 0.35))
        p.resetDebugVisualizerCamera(
            cameraDistance=2.4,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=(0, 0, 0.3),
        )

    return robot_id


def create_colored_blocks() -> None:
    block_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.12, 0.12, 0.12))
    colors = (
        (0.9, 0.1, 0.1, 1),
        (0.1, 0.45, 0.9, 1),
        (0.15, 0.65, 0.25, 1),
    )
    positions = ((0.7, -0.35, 0.12), (0.95, 0.0, 0.12), (0.7, 0.35, 0.12))

    for color, position in zip(colors, positions):
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=(0.12, 0.12, 0.12),
            rgbaColor=color,
        )
        p.createMultiBody(
            baseMass=0.4,
            baseCollisionShapeIndex=block_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
        )


def drive_robot(robot_id: int, robot: str, step_index: int, time_step: float) -> None:
    elapsed = step_index * time_step

    if robot == "kuka":
        joint_count = p.getNumJoints(robot_id)
        for joint_index in range(joint_count):
            target = 0.45 * math.sin(elapsed * 1.2 + joint_index * 0.55)
            p.setJointMotorControl2(
                robot_id,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=target,
                force=90,
            )
        return

    forward_speed = 2.5
    turn_speed = 0.6 * math.sin(elapsed)
    left_velocity = forward_speed - turn_speed
    right_velocity = forward_speed + turn_speed

    for joint_index in range(p.getNumJoints(robot_id)):
        joint_name = p.getJointInfo(robot_id, joint_index)[1].decode("utf-8")
        if "left" in joint_name.lower() and "wheel" in joint_name.lower():
            p.setJointMotorControl2(
                robot_id,
                joint_index,
                p.VELOCITY_CONTROL,
                targetVelocity=left_velocity,
                force=20,
            )
        elif "right" in joint_name.lower() and "wheel" in joint_name.lower():
            p.setJointMotorControl2(
                robot_id,
                joint_index,
                p.VELOCITY_CONTROL,
                targetVelocity=right_velocity,
                force=20,
            )


def print_joint_table(robot_id: int) -> None:
    print("Joint table:")
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        name = info[1].decode("utf-8")
        joint_type = info[2]
        lower_limit = info[8]
        upper_limit = info[9]
        print(
            f"  {joint_index:02d} name={name} "
            f"type={joint_type} limit=({lower_limit:.3f}, {upper_limit:.3f})"
        )


def log_robot_state(robot_id: int, step_index: int, time_step: float) -> None:
    position, orientation = p.getBasePositionAndOrientation(robot_id)
    sim_time = step_index * time_step
    print(
        f"t={sim_time:7.3f}s "
        f"pos=({position[0]: .3f}, {position[1]: .3f}, {position[2]: .3f}) "
        f"orn=({orientation[0]: .3f}, {orientation[1]: .3f}, "
        f"{orientation[2]: .3f}, {orientation[3]: .3f})"
    )


def run(config: SimulationConfig) -> None:
    client_id = connect(config.mode)
    try:
        robot_id = setup_world(config)
        total_steps = max(1, int(config.seconds / config.time_step))
        log_interval_steps = (
            max(1, int(config.log_every / config.time_step))
            if config.log_every > 0
            else 0
        )

        if config.inspect_joints:
            print_joint_table(robot_id)

        for step_index in range(total_steps):
            drive_robot(robot_id, config.robot, step_index, config.time_step)
            p.stepSimulation()
            if log_interval_steps and step_index % log_interval_steps == 0:
                log_robot_state(robot_id, step_index, config.time_step)
            if config.realtime:
                time.sleep(config.time_step)

        position, orientation = p.getBasePositionAndOrientation(robot_id)
        print(f"Finished {total_steps} steps")
        print(f"Robot position: {position}")
        print(f"Robot orientation: {orientation}")
        if config.mode == "gui" and config.pause_at_end:
            input("Simulation finished. Press Enter to close PyBullet...")
    finally:
        p.disconnect(client_id)


def main() -> None:
    config = parse_args()
    run(config)


if __name__ == "__main__":
    main()
