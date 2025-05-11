"""Robot URDF visualizer

Requires yourdfpy and robot_descriptions. Any URDF supported by yourdfpy should work.
- https://github.com/robot-descriptions/robot_descriptions.py
- https://github.com/clemense/yourdfpy

The :class:`viser.extras.ViserUrdf` is a lightweight interface between yourdfpy
and viser. It can also take a path to a local URDF file as input.
"""

from __future__ import annotations

import time
from typing import Literal

import numpy as np
import tyro
# from robot_descriptions.loaders.yourdfpy import load_robot_description

import viser
from viser.extras import ViserUrdf
from pathlib import Path
from autolab_core import RigidTransform
import trimesh
import os
import cv2

import matplotlib.pyplot as plt
from datetime import datetime

YUMI_REST_POSE = {
    "yumi_joint_1_r": 1.21442839,
    "yumi_joint_2_r": -1.03205606,
    "yumi_joint_7_r": -1.10072738,
    "yumi_joint_3_r": 0.2987352 - 0.2,
    "yumi_joint_4_r": -1.85257716,
    "yumi_joint_5_r": 1.25363652,
    "yumi_joint_6_r": -2.42181893,
    "yumi_joint_1_l": -1.24839656,
    "yumi_joint_2_l": -1.09802876,
    "yumi_joint_7_l": 1.06634394,
    "yumi_joint_3_l": 0.31386161 - 0.2,
    "yumi_joint_4_l": 1.90125141,
    "yumi_joint_5_l": 1.3205139,
    "yumi_joint_6_l": 2.43563939,
    "gripper_r_joint": 0, # 0.025,
    "gripper_l_joint": 0, # 0.025,
}

YUMI_LEFT_JOINTS = [
    "yumi_joint_1_l",
    "yumi_joint_2_l",
    "yumi_joint_7_l",
    "yumi_joint_3_l",
    "yumi_joint_4_l",
    "yumi_joint_5_l",
    "yumi_joint_6_l",
]

def main() -> None:
    # Start viser server.
    server = viser.ViserServer()

    # ViserUrdf expects the path to be a path object or a yourdfpy.URDF object
    urdf_path = Path("/home/xi/xi/data/yumi_description/urdf/yumi.urdf")
    viser_urdf = ViserUrdf(
            server, urdf_path
        )
    # Set initial robot configuration.
    viser_urdf.update_cfg(YUMI_REST_POSE)

    # Visualize the camera.
    camera_tf = RigidTransform.load("/home/xi/xi/data/brio_info/cam2world.tf")
    brio_path = Path("/home/xi/xi/data/brio_info/BRIO.stl")
    brio_mesh = trimesh.load(str(brio_path))
    cam_to_brio = RigidTransform(
            rotation=RigidTransform.quaternion_from_axis_angle(
                np.array([1, 0, 0]) * (np.pi)
            ),
            translation=np.array([0.0, 0.0, 0.0]),
        )
    # camera_frame = server.scene.add_frame(
    #     "camera",
    #     position=camera_tf.translation,  # rough alignment.
    #     wxyz=camera_tf.quaternion,
    #     show_axes=True,
    #     axes_length=0.1,
    #     axes_radius=0.005,
    # )
    # server.scene.add_mesh_trimesh(
    #     "camera/mesh",
    #     mesh=brio_mesh,
    #     scale=0.001,
    #     position=cam_to_brio.translation,
    #     wxyz=cam_to_brio.quaternion,
    # )   

    # adding controls to custom lights in the scene
    server.scene.add_transform_controls("/control0", position=(0.0, 10.0, 5.0))
    server.scene.add_label("/control0/label", "Directional")
    server.scene.add_transform_controls("/control1", position=(0.0, -5.0, 5.0))
    server.scene.add_label("/control1/label", "Point")

    directional_light = server.scene.add_light_directional(
        name="/control0/directional_light",
        color=(186, 219, 173),
    )
    point_light = server.scene.add_light_point(
        name="/control1/point_light",
        color=(192, 255, 238),
        intensity=30.0,
    )

    # Create default light toggle.
    gui_default_lights = server.gui.add_checkbox("Default lights", initial_value=True)
    gui_default_lights.on_update(
        lambda _: server.scene.enable_default_lights(gui_default_lights.value)
    )

    # Create light control inputs.
    with server.gui.add_folder("Directional light"):
        gui_directional_color = server.gui.add_rgb(
            "Color", initial_value=directional_light.color
        )
        gui_directional_intensity = server.gui.add_slider(
            "Intensity",
            min=0.0,
            max=20.0,
            step=0.01,
            initial_value=directional_light.intensity,
        )

        @gui_directional_color.on_update
        def _(_) -> None:
            directional_light.color = gui_directional_color.value

        @gui_directional_intensity.on_update
        def _(_) -> None:
            directional_light.intensity = gui_directional_intensity.value

    with server.gui.add_folder("Point light"):
        gui_point_color = server.gui.add_rgb("Color", initial_value=point_light.color)
        gui_point_intensity = server.gui.add_slider(
            "Intensity",
            min=0.0,
            max=200.0,
            step=0.01,
            initial_value=point_light.intensity,
        )

        @gui_point_color.on_update
        def _(_) -> None:
            point_light.color = gui_point_color.value

        @gui_point_intensity.on_update
        def _(_) -> None:
            point_light.intensity = gui_point_intensity.value
    

    output_path = "/home/xi/xi/xi_output/"
    button = server.gui.add_button("Render All BRIO Views")
    rnd_light = server.gui.add_button("Randomize Lighting")
    
    # Create default light toggle.
    gui_default_lights = server.gui.add_checkbox("Default lights", initial_value=True)
    gui_default_lights.on_update(
        lambda _: server.scene.enable_default_lights(gui_default_lights.value)
    )

    @rnd_light.on_click
    def _(_) -> None:
        directional_light.color = np.random.randint([0, 0, 0], 255)
        directional_light.intensity = np.random.random_sample() * 100
        breakpoint()
        # directional_light.position = 
        # direction_light.

    @button.on_click
    def _(event: viser.GuiEvent) -> None:
        client = event.client
        assert client is not None   

        breakpoint()
        # client.scene.reset()

        data_path = "/home/xi/xi/data/sample_traj/"
        print("Loading joint angles...")
        print(data_path+"joint_angles.npy")
        joint_angles = np.load(data_path+"joint_angles.npy")
        print(joint_angles.shape)
        # Set initial robot configuration.
        curr_cfg = YUMI_REST_POSE
        viser_urdf.update_cfg(curr_cfg)
        
        curr_time = datetime.now().strftime("%Y-%m-%d-%H%M")
        img_output_path = output_path+curr_time+"/rendered_imgs"
        mask_output_path = output_path+curr_time+"/masks"
        os.makedirs(img_output_path, exist_ok=True)
        os.makedirs(mask_output_path, exist_ok=True)

        for i, left_cfg in enumerate(joint_angles):
            if not i % 10 == 0:
                continue
            for key, value in zip(YUMI_LEFT_JOINTS, left_cfg):
                curr_cfg[key] = value
            print(f"Frame {i} joint angles")
            viser_urdf.update_cfg(curr_cfg)

            print("Rendering image...")
            # viser crashes if we try to do it at 4K, fov is the vertical FOV
            breakpoint()
            img = client.get_render(height=2160//2, width=4096//2, wxyz=camera_tf.quaternion, position=camera_tf.translation, fov=0.8274606, transport_format="png")
            img = cv2.resize(img, (4096, 2160), interpolation=cv2.INTER_LINEAR)
            cv2.imwrite(f"{img_output_path}/{i}.png", img)
            cv2.imwrite(f"{mask_output_path}/{i}.png", img[:,:,-1])
            # RGB 
            # plt.imshow(images[-1])
            # plt.show()
            # 
            # View alpha mask
            # plt.imshow(images[-1][:,:,-1])
            # plt.show()
        print("Done!")


    # Sleep forever.
    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    tyro.cli(main)