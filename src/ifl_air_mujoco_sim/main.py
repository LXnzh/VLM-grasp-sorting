import hydra
from omegaconf import DictConfig
import mujoco
import numpy as np
import os

from env.mjcontrol_interface import MuJoCoInterface

@hydra.main(config_path="env/config", config_name="base_env")
def main(cfg: DictConfig):
    # Set up MuJoCo model
    model_path = hydra.utils.to_absolute_path(cfg.sim.scene_dir)
    
    sim = MuJoCoInterface(
        model_path=model_path,
        camera_names=cfg.sim.camera_names,
        headless=cfg.sim.headless,
        control_timestep=cfg.sim.control_timestep,
        objects_config=cfg.get('objects', []),
        random_object_count=cfg.get('random_object_count', 0),
        fixed_object_names=cfg.get('fixed_object_names', []),
        scene_mode=cfg.get('scene_mode', 'mix'),
        scene_object_categories=cfg.get('scene_object_categories', {}),
        assigned_object_names=cfg.get('assigned_object_names', []),
        placement_slots=cfg.get('placement_slots', []),
    )

    try:
        while True:
            sim.step_simulation()
    except KeyboardInterrupt:
        print("[INFO]: Simulation interrupted by user.")
    finally:
        sim.close()
        print("[INFO]: Simulation closed.")
    #while goal_not_reached:
    #    obs = self.get_proprioception()
    #    self.apply_joint_action(action)
    #    self.step_simulation()
    #    elapsed = time.time() - t0

    

if __name__ == "__main__":
    main()
