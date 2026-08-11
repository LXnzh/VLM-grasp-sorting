import os

import hydra
from omegaconf import DictConfig
import time
from env.ros2_interface import UR10eRos2Interface

@hydra.main(config_path="env/config", config_name="base_env", version_base=None)
def main(cfg: DictConfig):
    sorting_enabled = str(
        os.environ.get("MY_COURSE_SINGLE_BIN_MODE", "")
    ).strip().lower() in {"1", "true", "yes", "on"}
    classification_bins = (
        cfg.get("classification_bins_single_random")
        if sorting_enabled
        else cfg.get("classification_bins")
    )
    placement_slots = (
        cfg.get("sorting_placement_slots", [])
        if sorting_enabled
        else cfg.get("placement_slots", [])
    )
    sim_values = {**cfg.sim}
    if sorting_enabled:
        sim_values["render_fps"] = max(
            float(sim_values.get("render_fps", 0.0)),
            float(cfg.get("sorting_render_fps", 10.0)),
        )
    sim_cfg = {**sim_values,
               'model_path': hydra.utils.to_absolute_path(cfg.sim.scene_dir),
               'objects_config': cfg.get('objects', []),
               'random_object_count': cfg.get('random_object_count', 0),
               'fixed_object_names': cfg.get('fixed_object_names', []),
               'scene_mode': cfg.get('scene_mode', 'mix'),
               'scene_object_categories': cfg.get('scene_object_categories', {}),
               'assigned_object_names': cfg.get('assigned_object_names', []),
               'placement_slots': placement_slots,
               'classification_bins': classification_bins,
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
