import math
import time
from datetime import timedelta

import hydra
from omegaconf import DictConfig
import numpy as np
import matplotlib.pyplot as plt
import os

from env.mjcontrol_interface import MuJoCoInterface


@hydra.main(config_path="env/config", config_name="base_env")
def main(cfg: DictConfig):
    """Run a simple repeating left-right motion and plot action vs. position.

    Parameters (via Hydra overrides or defaults):
      - motion.period: seconds (default 3.0)
      - motion.amplitude: radians for the selected joint (default 0.2)
      - motion.joint_index: which actuator/joint to drive (default 0)
      - motion.duration: total run time in seconds (default 10.0)
    """

    # Resolve motion params with safe defaults if not present in config
    period = float(cfg.motion.period) if "motion" in cfg and "period" in cfg.motion else 3.0
    amplitude = float(cfg.motion.amplitude) if "motion" in cfg and "amplitude" in cfg.motion else 0.2
    joint_index = int(cfg.motion.joint_index) if "motion" in cfg and "joint_index" in cfg.motion else 0
    duration = float(cfg.motion.duration) if "motion" in cfg and "duration" in cfg.motion else 10.0

    # Use a non-interactive backend when running headless to avoid display requirements
    try:
        if cfg.sim.headless:
            plt.switch_backend("Agg")
    except Exception:
        pass

    # Build simulation interface (inherits sim.* from base config)
    model_path = hydra.utils.to_absolute_path(cfg.sim.scene_dir)
    sim = MuJoCoInterface(
        model_path=model_path,
        camera_names=cfg.sim.camera_names,
        headless=cfg.sim.headless,
        control_timestep=cfg.sim.control_timestep,
        render_fps=cfg.sim.render_fps,
    )

    # Determine stepping granularity: how many physics steps per control tick
    mj_dt = sim.model.opt.timestep  # physics timestep (s)
    ctrl_dt = float(cfg.sim.control_timestep)
    steps_per_ctrl = max(1, int(round(ctrl_dt / mj_dt)))

    # Base control vector and safety clip using actuator ctrl ranges if available
    base_ctrl = np.copy(sim.data.ctrl)
    nu = sim.nu
    if joint_index < 0 or joint_index >= nu:
        raise IndexError(f"joint_index {joint_index} out of range [0, {nu-1}]")

    # Read actuator control range for clipping (shape: [nu, 2])
    ctrl_range = None
    if hasattr(sim.model, "actuator_ctrlrange") and sim.model.actuator_ctrlrange is not None:
        ctrl_range = np.array(sim.model.actuator_ctrlrange)


    # Data logs
    t_wall = []  # wall-clock timestamps (s from start)
    u_log = []   # commanded control (selected joint)
    q_log = []   # measured position (selected joint)

    print(
        f"[RUN] Driving joint {joint_index} with sine pattern: amplitude={amplitude} rad, period={period}s, "
        f"duration={duration}s, ctrl_dt={ctrl_dt}s ({steps_per_ctrl} sim steps/tick)."
    )

    t0 = time.perf_counter()
    next_tick = t0
    try:

        while True:
            elapsed = time.perf_counter() - t0
            if elapsed >= duration:
                break

            # Sine command around base control
            u = base_ctrl.copy()
            if elapsed > 3:
                u_val = base_ctrl[joint_index] + amplitude * (math.cos(2.0 * math.pi * (elapsed-3) / period) - 1)
            else:
                u_val = base_ctrl[joint_index]

            # Clip to actuator range if available
            if ctrl_range is not None:
                low, high = ctrl_range[joint_index]
                u_val = float(np.clip(u_val, low, high))

            u[joint_index] = u_val

            # Apply action
            sim.apply_joint_action(u)

            # Advance simulation for one control interval
            for _ in range(steps_per_ctrl):
                sim.step_simulation()

            # Pace wall clock to control period to ensure consistent timing in GUI vs headless
            next_tick += ctrl_dt
            now = time.perf_counter()
            sleep_dt = next_tick - now
            if sleep_dt > 0:
                time.sleep(sleep_dt)

            # Log wall time, input (control) and measured position for selected dof
            prop = sim.get_proprioception()
            t_wall.append(elapsed)
            u_log.append(u[joint_index])
            # Assumption: joint_index maps to same index in qpos for first nu joints
            q_log.append(float(prop["qpos"][joint_index]))


        # Plot action vs. measured position
        fig, ax = plt.subplots(figsize=(9, 4.5))
        ax.plot(t_wall, u_log, label=f"u (ctrl) [joint {joint_index}]", linewidth=2)
        ax.plot(t_wall, q_log, label=f"q (position) [joint {joint_index}]", linewidth=2)
        ax.set_xlabel("Time (s)")
        ax.set_title("Action vs. Position (single joint)")
        ax.grid(True, alpha=0.3)
        ax.legend()

        out_name = f"action_vs_qpos_joint{joint_index}.png"
        fig.tight_layout()
        out_path = os.path.join(os.getcwd(), out_name)
        fig.savefig(out_path, dpi=150)
        print(f"[DONE] Saved plot to: {out_path}")

    except KeyboardInterrupt:
        print("[INFO]: Interrupted by user.")
    finally:
        sim.close()
        run_time = timedelta(seconds=int(time.perf_counter() - t0))
        print(f"[INFO]: Simulation closed. Runtime ~{run_time}.")


if __name__ == "__main__":
    main()
