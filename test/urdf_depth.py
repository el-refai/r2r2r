"""
Script for visualizing a robot from a URDF.
"""

import sys
import time
import logging
import argparse
import numpy as np
from functools import partial

from yourdfpy import __version__
from yourdfpy import URDF
import trimesh
from pyrender import Scene, Node, IntrinsicsCamera, OffscreenRenderer, RenderFlags, Mesh, Viewer, PerspectiveCamera 
from pathlib import Path
from autolab_core import RigidTransform
import datetime
import os
import cv2
import matplotlib.pyplot as plt

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

def as_mesh(scene_or_mesh):
    """
    Convert a possible scene to a mesh.

    If conversion occurs, the returned mesh has only vertex and face data.
    """
    if isinstance(scene_or_mesh, trimesh.Scene):
        if len(scene_or_mesh.geometry) == 0:
            mesh = None  # empty scene
        else:
            # we lose texture information here
            mesh = trimesh.util.concatenate(
                tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                    for g in scene_or_mesh.geometry.values()))
    else:
        assert(isinstance(mesh, trimesh.Trimesh))
        mesh = scene_or_mesh
    return mesh

def pyrender_depth(trimesh_scene, img_width, img_height, cam_matrix, cam_pose, cam_trans, cam_quat):
    # source: https://github.com/mmatl/pyrender/issues/58
    scene = Scene()
    # scene = Scene.from_trimesh_scene(trimesh_scene)

    # urdf_mesh = as_mesh(trimesh_scene)
    # mesh = Mesh.from_trimesh(trimesh_scene, smooth = False)
    robot_mesh = Mesh.from_trimesh(trimesh_scene)
    robot_node = Node(mesh=robot_mesh)
    scene.add_node(robot_node)

    camera = PerspectiveCamera(yfov=0.8274606)
    # camera = IntrinsicsCamera(fx=cam_matrix[0, 0], fy=cam_matrix[1, 1], cx=cam_matrix[0, 2], cy=cam_matrix[1, 2], znear=0.001, zfar=20)

    # camera_trimesh = trimesh.load("/home/xi/xi/data/brio_info/BRIO.stl")
    # cam_mesh = Mesh.from_trimesh(camera_trimesh)
    # camera_node = Node(mesh=cam_mesh, translation = cam_trans, rotation=cam_quat)
    # scene.add(cam_mesh, pose = cam_pose)
    # scene.add_node(camera_node)

    scene.add(camera, pose=cam_pose)
    
    # v = Viewer(scene, shadows=True)
    # while True:
    #     time.sleep(1.0)
    r = OffscreenRenderer(img_width, img_height)

    color, depth = r.render(scene)
    breakpoint()
    plt.imshow(depth)
    plt.show()
    return color, depth

def run_trajectory(urdf_model):
    camera_tf = RigidTransform.load("/home/xi/xi/data/brio_info/cam2world.tf")
    rot_z_180 = RigidTransform(
            rotation=RigidTransform.quaternion_from_axis_angle(
                np.array([0, 0, 1]) * (np.pi)
            ),
            translation=np.array([0.0, 0.0, 0.0]), from_frame="world", to_frame="world"
        )
    rot_y_180 = RigidTransform(
            rotation=RigidTransform.quaternion_from_axis_angle(
                np.array([0, 1, 0]) * (np.pi)
            ),
            translation=np.array([0.0, 0.0, 0.0]), from_frame="world", to_frame="world"
        )
    rot_x_180 = RigidTransform(
            rotation=RigidTransform.quaternion_from_axis_angle(
                np.array([1, 0, 0]) * (np.pi)
            ),
            translation=np.array([0.0, 0.0, 0.0]), from_frame="world", to_frame="world"
        )
    cam_trans = camera_tf.translation
    # outputs wxyz
    cam_quat = camera_tf.quaternion
    # pyrender wants xyzw
    cam_pyrender_quat = np.roll(cam_quat, -1)

    # cam_pose = (rot_z_180*camera_tf).matrix
    # cam_pose = (rot_y_180*camera_tf).matrix
    # cam_pose = (rot_x_180*camera_tf).matrix
    # breakpoint()
    cam_pose = camera_tf.matrix
    cam_intrinsics = np.load("/home/xi/xi/data/brio_info/camera_calibration.npz")
    cam_matrix = cam_intrinsics["camera_matrix"]
    dist_coeffs = cam_intrinsics["dist_coeffs"]

    data_path = "/home/xi/xi/data/sample_traj/"
    print("Loading joint angles...")
    print(data_path+"joint_angles.npy")
    joint_angles = np.load(data_path+"joint_angles.npy")
    print(joint_angles.shape)
    # Set initial robot configuration.
    curr_cfg = YUMI_REST_POSE.copy()
    urdf_model.update_cfg(curr_cfg)
    
    curr_time = datetime.datetime.now().strftime("%Y-%m-%d-%H%M")
    output_path = "/home/xi/xi/xi_output/"
    color_output_path = output_path+curr_time+"/rendered_color"
    depth_output_path = output_path+curr_time+"/rendered_depth"
    os.makedirs(color_output_path, exist_ok=True)
    os.makedirs(depth_output_path, exist_ok=True)

    for i, left_cfg in enumerate(joint_angles):
        if not i % 10 == 0:
            continue
        for key, value in zip(YUMI_LEFT_JOINTS, left_cfg):
            curr_cfg[key] = value
        print(f"Frame {i} joint angles")
        urdf_model.update_cfg(curr_cfg)
        urdf_trimesh_scene = urdf_model.scene
        trimesh_cfg_path = "/home/xi/xi/xi_output/curr_urdf_cfg_trimesh_scene.glb"
        urdf_trimesh_scene.export(file_obj=trimesh_cfg_path)
        urdf_mesh = trimesh.load(trimesh_cfg_path, force='mesh')

        print("Rendering image...")        
        color, depth = pyrender_depth(urdf_mesh, img_width=4096, img_height=2160, cam_matrix=cam_matrix, cam_pose=cam_pose, cam_trans=cam_trans, cam_quat= cam_pyrender_quat)
        cv2.imwrite(f"{color_output_path}/{i}.png", color)
        np.save(f"{depth_output_path}/{i}.png", depth)
        # cv2.imwrite(f"{depth_output_path}/{i}.png", depth)
    print("Done!")

def main(args):
    """Wrapper allowing string arguments in a CLI fashion.

    Args:
      args (List[str]): command line parameters as list of strings
          (for example  ``["--verbose", "42"]``).
    """
    # args = parse_args(args)

    # NOTE: PREVIOUS PYGLET VERSION IS 2.0.18, I AM DOWNGRADING TO 1.5.28 SO I CAN DO trimesh.show() TO DEBUG STUFF!!!
    urdf_model = URDF.load(
            "/home/xi/xi/data/yumi_description/urdf/yumi.urdf", build_collision_scene_graph=True, load_collision_meshes=True
        )
    # urdf_model.show(
    #     # collision_geometry=args.collision,
    #     callback=callback,
    # )

    run_trajectory(urdf_model=urdf_model)


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`.

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    run()