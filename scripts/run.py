"""Launch Isaac Sim Simulator first."""
import argparse
from isaaclab.app import AppLauncher
# add argparse arguments
parser = argparse.ArgumentParser(description="This script tests YuMi with random jitter in a physics environment.")
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
args_cli.headless = True # Defaulted True for headless development
args_cli.enable_cameras = True # Defaulted True for rendering viewpoints
print(args_cli)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
from xi.isaaclab_viser.configs.scene_configs.yumi_scene_cfg import (
    YumiPickTigerCfg, YumiCoffeeMakerCfg, YumiCoffeeMakerDistractorsCfg
)

from xi.isaaclab_viser.yumi_simulators.yumi_coffee_maker import CoffeeMaker
# from xi.isaaclab_viser.yumi_simulators.old.yumi_pick_tiger import PickTiger
import os
from pathlib import Path


def main():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    data_dir = os.path.join(dir_path, "../data")
    
    scene_config = YumiCoffeeMakerDistractorsCfg(num_envs=30, env_spacing=80.0) #YumiCoffeeMakerCfg(num_envs=45, env_spacing=80.0)
    # scene_config = YumiPickTigerCfg(num_envs=45, env_spacing=80.0)
    output_dir = os.path.join(dir_path, "../output_data/yumi_coffee_maker")
    # output_dir = os.path.join(dir_path, "../output_data/yumi_tiger")
    urdf_path = {
        'robot': Path(f'{data_dir}/yumi_description/urdf/yumi.urdf'), #Path(f'{data_dir}/yumi_description/urdf/yumi.urdf'),
    }
    CoffeeMaker(
                simulation_app,
                scene_config, 
                init_viser = True,
                urdf_path = urdf_path,
                save_data=True,
                output_dir = output_dir)
    
    # Using robot USD file in the scene_config and loading the same URDF separately speeds up IsaacLab init 
    # significantly (Avoids running URDF to USD converter)
    
if __name__ == "__main__":
    main()
