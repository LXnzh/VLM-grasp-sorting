import hydra
from omegaconf import DictConfig
import time
from env.ros2_interface import UR10eRos2Interface

@hydra.main(config_path="env/config", config_name="base_env", version_base=None)
def main(cfg: DictConfig):
    sim_cfg = {**cfg.sim,
               'model_path': hydra.utils.to_absolute_path(cfg.sim.scene_dir),
               'objects_config': cfg.get('objects', []),
               'random_object_count': cfg.get('random_object_count', 0),
               'fixed_object_names': cfg.get('fixed_object_names', []),
               'scene_mode': cfg.get('scene_mode', 'mix'),
               'scene_object_categories': cfg.get('scene_object_categories', {}),
               'assigned_object_names': cfg.get('assigned_object_names', []),
               'placement_slots': cfg.get('placement_slots', []),
               }
    interface = UR10eRos2Interface(sim_cfg)
    print("[INFO] UR10eRos2Interface started. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[INFO] Shutting down...")
    finally:
        interface.close()

if __name__ == "__main__":
    main()
