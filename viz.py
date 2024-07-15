"""Read depth file coming from the Apple LiDAR and display it with OpenCV"""

import numpy as np
import matplotlib.pyplot as plt
import cv2
from argparse import ArgumentParser
import json
import open3d
from time import time, sleep
import os
import json
import cv2
from utils import project
from line_mesh import LineMesh

MIN_LIDAR_POINTS = 100

def make_bounding_boxes(path):
    with open(path,"r") as file:
        jsonData = json.load(file)

    points = []
    lines = []
    i = 0
    for bbox in jsonData:

        if bbox["class"] != "TYPE_VEHICLE":
            continue

        if bbox["num_lidar_points_in_box"] < MIN_LIDAR_POINTS:
            continue

        x = bbox["center_x"]
        y = bbox["center_y"]
        z = bbox["center_z"]
        height = bbox["height"]

        cos_angle = np.cos(bbox["heading"])
        sin_angle = np.sin(bbox["heading"])
        length = bbox["width"]
        width = bbox["length"]
        rot = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

        corners = (
            np.array(
                [
                    [width, width, -width, -width],
                    [length, -length, -length, length],
                ]
            )
            / 2
        )

        # Apply angle rotation
        corners = np.dot(rot, corners) + np.array([x, y]).reshape(2, 1)

        br = corners[:, 0]
        tr = corners[:, 1]
        tl = corners[:, 2]
        bl = corners[:, 3]
    
        points += [
            [tl[0], tl[1], z - height / 2],
            [tr[0], tr[1], z - height / 2],
            [bl[0], bl[1], z - height / 2],
            [br[0], br[1], z - height / 2],
            [tl[0], tl[1], z + height / 2],
            [tr[0], tr[1], z + height / 2],
            [bl[0], bl[1], z + height / 2],
            [br[0], br[1], z + height / 2],
        ]

        lines += [
            [0 + 8 * i, 1 + 8 * i],
            [0 + 8 * i, 2 + 8 * i],
            [1 + 8 * i, 3 + 8 * i],
            [2 + 8 * i, 3 + 8 * i],
            [4 + 8 * i, 5 + 8 * i],
            [4 + 8 * i, 6 + 8 * i],
            [5 + 8 * i, 7 + 8 * i],
            [6 + 8 * i, 7 + 8 * i],
            [0 + 8 * i, 4 + 8 * i],
            [1 + 8 * i, 5 + 8 * i],
            [2 + 8 * i, 6 + 8 * i],
            [3 + 8 * i, 7 + 8 * i],
        ]

        # if bbox["class"] == "TYPE_VEHICLE":

        i += 1
        # elif bbox["class"] == "TYPE_SIGN":
        #     colors += [[0, 1, 0] for _ in range(12)]
        # elif bbox["class"] == "TYPE_PEDESTRIAN":
        #     colors += [[0, 0, 1] for _ in range(12)]
        # else:
        #     print(bbox["class"]) 
    
    return points, lines

class Model:
    def __init__(self):
        self.__vis = open3d.visualization.VisualizerWithKeyCallback()
        self.__vis.create_window(window_name='Waymo Lidar Detection', width=1920, height=1280)

        self.pcd = None
        self.bboxes = None

        self.__vis.register_key_callback(32, self.pause)
        self.__vis.register_key_callback(78, self.next_frame)
        self.__vis.get_render_option().point_size = 3
        self.__vis.get_render_option().background_color = [0.1, 0.1, 0.1]
        self.is_paused = False
        self.next_frame = False

    def set_view(self):
        ctr = self.__vis.get_view_control()
        ctr.set_front((-1.0, 0.0, 1.0))
        ctr.set_up((0.0, 0.0, 1.0))
        ctr.set_lookat((0.0, 0.0, 0.0))
        ctr.set_zoom(0.2)

    def pause(self, _):
        self.is_paused = not self.is_paused

    def next_frame(self, _):
        self.next_frame = True

    def add_xyz(self, xyz, points, lines):
        self.pcd = open3d.geometry.PointCloud()

        self.pcd.points = open3d.utility.Vector3dVector(xyz)
        self.line_set = open3d.geometry.LineSet(
            points=open3d.utility.Vector3dVector([]),
            lines=open3d.utility.Vector2iVector([]),
        )

        self.bboxes = open3d.geometry.TriangleMesh()
        line_mesh = LineMesh(points, lines)
        self.bboxes.vertices = line_mesh.cylinder_segments[0].vertices
        self.bboxes.triangles = line_mesh.cylinder_segments[0].triangles

        self.__vis.add_geometry(self.pcd)
        self.__vis.add_geometry(self.bboxes)
        self.__vis.poll_events()
        self.__vis.update_renderer()

    def show(self):
        self.bboxes.paint_uniform_color([1, 0, 0])
        self.__vis.update_geometry(self.pcd)
        self.__vis.update_geometry(self.bboxes)
        
        self.__vis.poll_events()
        self.__vis.update_renderer()

    def kill(self):
        return not self.__vis.poll_events()


def display_point_cloud():
    
    model = Model()

    model.set_view()

    start = time()

    nb_frames = len(os.listdir(f"{args.sample}/pointclouds/"))
    i = 1
    while i <= nb_frames and not model.kill():
        # if time() - start < 0.1:
        #     continue

        if model.is_paused:
            if not model.next_frame:
                model.show()
                continue
            else:
                model.next_frame = False
            
        pcd = open3d.io.read_point_cloud(f"{args.sample}/pointclouds/pointcloud_{i:03d}.ply")

        points, lines = make_bounding_boxes(f"{args.sample}/{labels_or_inference}/{labels_or_inference}_{i:03d}.json")

        if model.pcd is None:
            model.add_xyz(pcd.points, points, lines)

            model.set_view()
        
        # print(colors)
        line_mesh = LineMesh(points, lines, radius=0.1)
        line_mesh.cylinder_segments = []
        line_mesh.create_line_mesh()
        model.bboxes.vertices = line_mesh.cylinder_segments[0].vertices
        model.bboxes.triangles = line_mesh.cylinder_segments[0].triangles
        

        model.pcd.points = open3d.utility.Vector3dVector(pcd.points)
        
        model.show()

        if args.front:
            folder = "front"
        elif args.front_left:
            folder = "front_left"
        elif args.front_right:
            folder = "front_right"
        elif args.side_left:
            folder = "side_left"
        elif args.side_right:
            folder = "side_right"
        else:
            i += 1
            start = time()
            continue

        img = cv2.imread(f"{args.sample}/images/{folder}/image_{i:03d}.png")

        with open(f"{args.sample}/{labels_or_inference}/{labels_or_inference}_{i:03d}.json","r") as file:
            jsonData = json.load(file)

        for bbox in jsonData:

            if bbox["class"] != "TYPE_VEHICLE":
                continue

            if bbox["num_lidar_points_in_box"] < MIN_LIDAR_POINTS:
                continue

            height = bbox["height"]

            cos_angle = np.cos(bbox["heading"])
            sin_angle = np.sin(bbox["heading"])
            length = bbox["width"]
            width = bbox["length"]
            rot = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

            corners = (
                np.array(
                    [
                        [width, width, -width, -width],
                        [length, -length, -length, length],
                    ]
                )
                / 2
            )

            # Apply angle rotation
            corners = np.dot(rot, corners) + np.array([bbox["center_x"], bbox["center_y"]]).reshape(2, 1)

            br = corners[:, 0]
            tr = corners[:, 1]
            tl = corners[:, 2]
            bl = corners[:, 3]
        
            z = bbox["center_z"]
            tld = np.array([tl[0], tl[1], z - height / 2])
            trd = np.array([tr[0], tr[1], z - height / 2])
            bld = np.array([bl[0], bl[1], z - height / 2])
            brd = np.array([br[0], br[1], z - height / 2])
            tlu = np.array([tl[0], tl[1], z + height / 2])
            tru = np.array([tr[0], tr[1], z + height / 2])
            blu = np.array([bl[0], bl[1], z + height / 2])
            bru = np.array([br[0], br[1], z + height / 2])

            # u, v = project(tld, to_cam_from_world, K)
            tld_proj = tuple(project(tld, to_cam_from_world, K).astype(int))
            trd_proj = tuple(project(trd, to_cam_from_world, K).astype(int))
            bld_proj = tuple(project(bld, to_cam_from_world, K).astype(int))
            brd_proj = tuple(project(brd, to_cam_from_world, K).astype(int))
            tlu_proj = tuple(project(tlu, to_cam_from_world, K).astype(int))
            tru_proj = tuple(project(tru, to_cam_from_world, K).astype(int))
            blu_proj = tuple(project(blu, to_cam_from_world, K).astype(int))
            bru_proj = tuple(project(bru, to_cam_from_world, K).astype(int))

            if tld_proj == (-1, -1) or trd_proj == (-1, -1) or bld_proj == (-1, -1) or brd_proj == (-1, -1) or tlu_proj == (-1, -1) or tru_proj == (-1, -1) or blu_proj == (-1, -1) or bru_proj == (-1, -1):
                continue

            color=(255, 0, 255)
            thickness = 2
            # bottom
            cv2.line(
                img,
                tld_proj,
                trd_proj,
                color=color,
                thickness=thickness,
            )
            cv2.line(
                img,
                trd_proj,
                brd_proj,
                color=color,
                thickness=thickness,
            )
            cv2.line(
                img,
                brd_proj,
                bld_proj,
                color=color,
                thickness=thickness,
            )
            cv2.line(
                img,
                bld_proj,
                tld_proj,
                color=color,
                thickness=thickness,
            )

            # top
            cv2.line(
                img,
                tlu_proj,
                tru_proj,
                color=color,
                thickness=thickness,
            )
            cv2.line(
                img,
                tru_proj,
                bru_proj,
                color=color,
                thickness=thickness,
            )
            cv2.line(
                img,
                bru_proj,
                blu_proj,
                color=color,
                thickness=thickness,
            )
            cv2.line(
                img,
                blu_proj,
                tlu_proj,
                color=color,
                thickness=thickness,
            )

            # verticals
            cv2.line(
                img,
                tld_proj,
                tlu_proj,
                color=color,
                thickness=thickness,
            )
            cv2.line(
                img,
                bld_proj,
                blu_proj,
                color=color,
                thickness=thickness,
            )
            cv2.line(
                img,
                trd_proj,
                tru_proj,
                color=color,
                thickness=thickness,
            )
            cv2.line(
                img,
                brd_proj,
                bru_proj,
                color=color,
                thickness=thickness,
            )

            image_overlay = img.copy()

            # Overlay red color over the whole bbox
            hull = cv2.convexHull(
                np.array(
                    [
                        tld_proj,
                        trd_proj,
                        bld_proj,
                        brd_proj,
                        tlu_proj,
                        tru_proj,
                        blu_proj,
                        bru_proj,
                    ],
                    dtype=int,
                )
            ).reshape(1, -1, 2)
            cv2.fillPoly(image_overlay, pts=hull, color=(0, 0, 255))

            alpha = 0.3
            img = cv2.addWeighted(img, 1 - alpha, image_overlay, alpha, 0)
            
        cv2.imshow("Image", img)
        k = cv2.waitKey(10)

        if k == 27:
            model.kill()
            break

        i += 1

        start = time()

    while not model.kill():
        model.show()




if __name__ == "__main__":
    parser = ArgumentParser(
        prog="Read and display the content of a sample (pointcloud + labelled bounding box),"
        "and optionnally the images",
    )
    parser.add_argument('sample')
    parser.add_argument('--inference',
                    action='store_true', help="If not specified, we will read GT, otherwise we will read inference")
    parser.add_argument('--front',
                    action='store_true')
    parser.add_argument('--front-left',
                    action='store_true')
    parser.add_argument('--front-right',
                    action='store_true')
    parser.add_argument('--side-left',
                    action='store_true')
    parser.add_argument('--side-right',
                    action='store_true')
    args = parser.parse_args()

    labels_or_inference = "labels" if not args.inference else "inference"

    if args.front or args.front_left or args.front_right or args.side_left or args.side_right:
        with open(f"{args.sample}/cameras.json","r") as file:
            cameraData = json.load(file)

        if args.front:
            cam = cameraData["FRONT"]
        elif args.front_left:
            cam = cameraData["FRONT_LEFT"]
        elif args.front_right:
            cam = cameraData["FRONT_RIGHT"]
        elif args.side_left:
            cam = cameraData["SIDE_LEFT"]
        elif args.side_right:
            cam = cameraData["SIDE_RIGHT"]

        intrinsic = cam["intrinsic"]

        K = np.array([[intrinsic[0], 0, intrinsic[2]],
                [0, intrinsic[1], intrinsic[3]],
                [0, 0, 1]])
        
        extrinsic = cam["extrinsic"]
        
        pose = np.array(extrinsic).reshape(4, 4)

        to_cam_from_world = np.linalg.inv(pose)

    display_point_cloud()
