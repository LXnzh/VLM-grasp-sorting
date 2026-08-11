import hydra
from omegaconf import DictConfig
import time
import numpy as np
import cv2

from env.mjcontrol_interface import MuJoCoInterface

camera_names = ["camera_orbbec_static", "camera_realsense"]  # Example camera names to render

@hydra.main(config_path="env/config", config_name="base_env")
def main(cfg: DictConfig):
    # Create sim
    sim = MuJoCoInterface(
        model_path=hydra.utils.to_absolute_path(cfg.sim.scene_dir),
        camera_names=camera_names,
        headless=cfg.sim.headless,
        control_timestep=cfg.sim.control_timestep,  # physics dt
    )
    sim.init_renderer(width=640, height=480)

    # --- tunables ---
    physics_dt = float(cfg.sim.control_timestep)   # fixed physics step
    cam_fps     = cfg.sim.render_fps               # cap camera to 30 Hz
    cam_period  = 1.0 / cam_fps
    # -------------    

    acc = 0.0
    t_last = time.perf_counter()
    t_last_cam = t_last

    try:
        while True:
            now = time.perf_counter()
            acc += now - t_last
            t_last = now

            # --- fixed-timestep physics ---
            # Step as many times as needed to catch up, each at physics_dt
            while acc >= physics_dt:
                sim.step_simulation()
                acc -= physics_dt

            # --- camera at capped FPS ---
            if now - t_last_cam >= cam_period:
                t_last_cam = now
                
                for cam_name in camera_names:
                    img = sim.get_camera_image(cam_name)
                    # Display (BGR for OpenCV)
                    if img is not None:
                        if img.dtype != np.uint8:
                            img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
                        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        cv2.imshow(cam_name, img_bgr)
                
                    cv2.waitKey(1)

            # Optional tiny sleep to avoid a hot loop burning a full core
            # (Physics timing stays stable because of the accumulator.)
            time.sleep(0.0005)

    except KeyboardInterrupt:
        print("[INFO]: Simulation interrupted by user.")
    finally:
        sim.close()
        cv2.destroyAllWindows()
        print("[INFO]: Simulation closed.")

if __name__ == "__main__":
    main()
