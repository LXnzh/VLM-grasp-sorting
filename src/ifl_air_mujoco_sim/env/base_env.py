import os
from typing import Optional
import hydra
from omegaconf import DictConfig, OmegaConf
import numpy as np
import mujoco
import mediapy as media

import mujoco.viewer
import enum
import tqdm
import inspect

from dm_control import mjcf
from pathlib import Path
from xml.etree import ElementTree as ET



class MujocoBaseEnv:
    def __init__(self, cfg: DictConfig):
        self.cfg = cfg

        self._setup_scene()


    def _setup_scene(self):
       
        # ensure this method is called only during initialization
        stack = inspect.stack()
        caller_name = inspect.stack()[1].function
        if caller_name != '__init__':
            raise RuntimeError(f"Init only method called from {caller_name}")
        
        # get model for robot arm and gripper
        arm_model = Path(self.cfg.env.arm)
        gripper_model = Path(self.cfg.env.gripper)

        # check if the arm model exists
        if not os.path.exists(arm_model):
            raise FileNotFoundError(f"Arm model {arm_model} does not exist.")
        if not os.path.exists(gripper_model):
            raise FileNotFoundError(f"Gripper model {gripper_model} does not exist.")
        
        arm_assets_dir = Path(self.cfg.env.arm_assets)
        gripper_assets_dir = Path(self.cfg.env.gripper_assets)

        if not os.path.exists(arm_assets_dir):
            raise FileNotFoundError(f"Arm model {arm_model} does not exist.")
        if not os.path.exists(gripper_assets_dir):
            raise FileNotFoundError(f"Gripper model {gripper_model} does not exist.")
    
        arm_mjcf = mjcf.from_path(arm_model)
        gripper_mjcf = mjcf.from_path(gripper_model)

        physics = mjcf.Physics.from_mjcf_model(gripper_mjcf)
        attachment_site = arm_mjcf.find("site", "attachment_site")
        if attachment_site is None:
            raise ValueError("No attachment site found in the arm model.")
        
        # Expand the ctrl and qpos keyframes to account for the new hand DoFs.
        arm_key = arm_mjcf.find("key", "home")
        if arm_key is not None:
            hand_key = gripper_mjcf.find("key", "home")
            if hand_key is None:
                arm_key.ctrl = np.concatenate([arm_key.ctrl, np.zeros(physics.model.nu)])
                arm_key.qpos = np.concatenate([arm_key.qpos, np.zeros(physics.model.nq)])
            else:
                arm_key.ctrl = np.concatenate([arm_key.ctrl, hand_key.ctrl])
                arm_key.qpos = np.concatenate([arm_key.qpos, hand_key.qpos])

        attachment_site.attach(gripper_mjcf)

        # set the meshdir to "assets" because we will copy the assets there
        xml_string = arm_mjcf.to_xml_string()
        root = ET.fromstring(xml_string)
        compiler = root.find("compiler")
        if compiler is not None:
            compiler.set("meshdir", "assets")
        else:
            compiler = ET.Element("compiler", {meshdir: "assets"})
            root.insert(0, compiler)
        xml_string = ET.tostring(root, encoding='unicode')

        output_dir = Path("models/ur10e_2f85")
        output_dir.mkdir(parents=True, exist_ok=True)

        output_path = output_dir / "ur10e_2f85.xml"
        with open(output_path, "w") as f:
            f.write(xml_string)
        print(f"Combined model saved to {output_path}")

        # Copy assets from both models (beware renaming with hashes to ensure unique names)
        target_dir = output_dir / "assets"
        target_dir.mkdir(exist_ok=True)

        for asset in arm_mjcf.asset.all_children():
            if hasattr(asset, 'file'):
                original_name = asset.file.get_vfs_filename(filename_with_hash=False)
                new_name = asset.file.get_vfs_filename(filename_with_hash=True)
                
                # check if asset originally from arm or gripper
                if getattr(asset, 'class') is not None:
                    src_dir = gripper_assets_dir
                else:
                    src_dir = arm_assets_dir
                
                # copy asset from source directory to target directory
                src_file = src_dir / original_name
                
                if src_file.exists():
                    dest_file = target_dir / new_name
                    if not dest_file.exists():
                        print(f"Copying {src_file} to {dest_file}")
                        dest_file.write_bytes(src_file.read_bytes())
                    else:
                        print(f"File {dest_file} already exists, skipping copy.")
                else:
                    print(f"Source file {src_file} does not exist, skipping copy.")




# Example usage:
@hydra.main(version_base=None, config_path="config", config_name="base_env")
def main(cfg: DictConfig):
    env = MujocoBaseEnv(cfg)
    #done = False
    #while not done:
    #    env.render()
    #env.close()

if __name__ == "__main__":
    main()