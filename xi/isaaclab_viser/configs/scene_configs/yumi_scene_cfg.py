# import matplotlib.pyplot as plt
import numpy as np
import os
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg, DeformableObjectCfg
from isaaclab.utils import configclass

from pathlib import Path
dir_path = os.path.dirname(os.path.realpath(__file__))
data_dir = os.path.join(dir_path, "../../../../data")

##
# Pre-defined configs
##
from xi.isaaclab_viser.configs.scene_configs.yumi_base_cfg import YumiBaseCfg
# from xi.isaaclab_viser.configs.scene_configs.yumi_base_skybox_cfg import YumiBaseCfg
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg

@configclass
class YumiPickTigerCfg(YumiBaseCfg):
    """Design the scene with sensors on the robot."""
    
    tiger = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Tiger",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/tiger/tiger_new.usd",
            # scale=(0.9, 0.9, 0.9),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=16,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                articulation_enabled = False,
            ),
            ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.45, 0.00, 0.06)),
    )

@configclass
class YumiCoffeeMakerCfg(YumiBaseCfg):
    """Design the scene with sensors on the robot."""
    coffee_maker = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/coffee_maker",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/coffee_maker/coffee_maker.usd",
            # scale=(1.3, 1.3, 1.3),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.4, 0.1, 0.093)),
    )
    
    mug = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/mug",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/mug/mug.usd",
            # scale=(0.9, 0.9, 0.9),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),

        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.4, 0.1, 0.09)),

    )
    
@configclass
class YumiCoffeeMakerDistractorsCfg(YumiBaseCfg):
    """Design the scene with sensors on the robot."""
    coffee_maker = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/coffee_maker",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/coffee_maker/coffee_maker.usd",
            # scale=(1.3, 1.3, 1.3),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.4, 0.1, 0.093)),
    )
    
    mug = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/mug",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/mug/mug.usd",
            # scale=(0.9, 0.9, 0.9),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),

        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.45, 0.1, 0.09)),

    )
    
    distractor_0 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/distractor_0",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/mug/mug.usd",
            # scale=(0.9, 0.9, 0.9),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),

        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.4, 0.15, 2.2692e-02)),

    )
    
    distractor_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/distractor_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/mug/mug.usd",
            # scale=(0.9, 0.9, 0.9),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),

        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.4, 0.1, 2.2692e-02)),

    )

@configclass
class YumiLedLightCfg(YumiBaseCfg):
    """Design the scene with sensors on the robot."""
    led_light2_subpart_0 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/led_light2_subpart_0",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/led_light/led_light2_subpart_0/led_light2_subpart_0.usd",
            # scale=(1.3, 1.3, 1.3),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.145)),
    )
    
    led_light2_subpart_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/led_light2_subpart_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/led_light/led_light2_subpart_1/led_light2_subpart_1.usd",
            # scale=(1.3, 1.3, 1.3),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.145)),
    )

    led_light2_subpart_2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/led_light2_subpart_2",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/led_light/led_light2_subpart_2/led_light2_subpart_2.usd",
            # scale=(1.3, 1.3, 1.3),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.145)),
    )
    
    led_light2_subpart_3 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/led_light2_subpart_3",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{data_dir}/assets/object_scans/led_light/led_light2_subpart_3/led_light2_subpart_3.usd",

            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    # rigid_body_enabled=False,
                    kinematic_enabled=True,
                    disable_gravity=True,
                    # solver_position_iteration_count=16,
                    # solver_velocity_iteration_count=16,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    # disable_gravity=False,
                ),
            # articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            #     articulation_enabled = False,
            #     ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=False,
                ),
            ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.145)),
    )