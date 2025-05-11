import numpy as np
import trimesh
import pyrender
import cv2 as opencv

from pyrender.node import Node

if __name__ == '__main__':
    fuzzy_trimesh = trimesh.load('/home/xi/pyrender/examples/models/fuze.obj')
    mesh = pyrender.Mesh.from_trimesh(fuzzy_trimesh)
    fuze_node = Node(mesh=mesh, translation=np.array([0.1, 0.15, -np.min(fuzzy_trimesh.vertices[:,2])]))
    scene = pyrender.Scene()
    fuze_matrix = np.array([
        [1.0, 0.0, 0.0, 0.1],
        [0.0, 1.0, 0.0, 0.15],
        [0.0, 0.0, 1.0, -0.1],
        [0.0, 0.0, 0.0, 1.0]
    ])
    # fuze_node = Node(mesh=mesh, matrix=fuze_matrix)
    scene.add_node(fuze_node)

    cam = pyrender.PerspectiveCamera(yfov=(np.pi / 3.0))
    cam_pose = np.array([
        [0.0,  -np.sqrt(2)/2, np.sqrt(2)/2, 0.5],
        [1.0, 0.0,           0.0,           0.0],
        [0.0,  np.sqrt(2)/2,  np.sqrt(2)/2, 0.5],
        [0.0,  0.0,           0.0,          1.0]
    ])
    cam_node = scene.add(cam, pose=cam_pose)
    r = pyrender.OffscreenRenderer(viewport_width=400, viewport_height=200)
    color, depth = r.render(scene)
    opencv.imshow('Renderer', color)
    opencv.waitKey(500)