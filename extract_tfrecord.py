import tensorflow.compat.v1 as tf
import numpy as np
from argparse import ArgumentParser
from pathlib import Path

tf.enable_eager_execution()

from waymo_open_dataset.utils import  frame_utils
from waymo_open_dataset import dataset_pb2 as open_dataset

from plyfile import PlyData, PlyElement

import json
import matplotlib.pyplot as plt
from tqdm import tqdm


classes = ["UNKNOWN", "TYPE_VEHICLE", "TYPE_PEDESTRIAN", "TYPE_SIGN", "TYPE_CYCLIST"]

parser = ArgumentParser()
parser.add_argument('tfrecord')
parser.add_argument('out_dir')
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
out_dir = Path(args.out_dir) / Path(args.tfrecord).stem
print(f"Write data to {out_dir}")
Path(out_dir).mkdir(parents=True, exist_ok=True)
Path(f"{out_dir}/pointclouds").mkdir(parents=True, exist_ok=True)
Path(f"{out_dir}/images").mkdir(parents=True, exist_ok=True)
if args.front:
    Path(f"{out_dir}/images/front").mkdir(parents=True, exist_ok=True)
if args.front_left:
    Path(f"{out_dir}/images/front_left").mkdir(parents=True, exist_ok=True)
if args.front_right:
    Path(f"{out_dir}/images/front_right").mkdir(parents=True, exist_ok=True)
if args.side_left:
    Path(f"{out_dir}/images/side_left").mkdir(parents=True, exist_ok=True)
if args.side_right:
    Path(f"{out_dir}/images/side_right").mkdir(parents=True, exist_ok=True)
Path(f"{out_dir}/labels").mkdir(parents=True, exist_ok=True)

dataset = tf.data.TFRecordDataset(args.tfrecord, compression_type='')

CAMERA_NAMES = ["NONE", "FRONT", "FRONT_LEFT", "FRONT_RIGHT", "SIDE_LEFT", "SIDE_RIGHT"]

idx = 1
for data in tqdm(dataset, total=198):
    frame = open_dataset.Frame()
    frame.ParseFromString(bytearray(data.numpy()))

    # save calibration information (first frame)
    if idx == 1:
        cameras = frame.context.camera_calibrations
        cameras_dict = {}
        for camera in cameras:
            name = CAMERA_NAMES[camera.name]
            cameras_dict[name] = {"intrinsic": list(camera.intrinsic), "extrinsic": list(camera.extrinsic.transform), "width": camera.width, "height": camera.height}

        with open(f'{out_dir}/cameras.json', 'w') as f:
            json.dump(cameras_dict, f) 

    # Save camera images
    if args.front:
        plt.imsave(f"{out_dir}/images/front/image_{idx:03}.png", tf.image.decode_jpeg(frame.images[0].image).numpy())
    if args.front_left:
        plt.imsave(f"{out_dir}/images/front_left/image_{idx:03}.png", tf.image.decode_jpeg(frame.images[1].image).numpy())
    if args.side_left:
        plt.imsave(f"{out_dir}/images/side_left/image_{idx:03}.png", tf.image.decode_jpeg(frame.images[2].image).numpy())
    if args.front_right:
        plt.imsave(f"{out_dir}/images/front_right/image_{idx:03}.png", tf.image.decode_jpeg(frame.images[3].image).numpy())
    if args.side_right:
        plt.imsave(f"{out_dir}/images/side_right/image_{idx:03}.png", tf.image.decode_jpeg(frame.images[4].image).numpy())

    # Save point cloud
    (range_images, camera_projections,
     _, range_image_top_pose) = frame_utils.parse_range_image_and_camera_projection(
        frame)

    points, cp_points = frame_utils.convert_range_image_to_point_cloud(
        frame,
        range_images,
        camera_projections,
        range_image_top_pose
    )

    points_all = np.concatenate(points, axis=0)

    vertex = np.array([tuple(pt) for pt in points_all], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    el = PlyElement.describe(vertex, 'vertex')
    PlyData([el]).write(f"{out_dir}/pointclouds/pointcloud_{idx:03}.ply")

    # Save bbox labels
    res = []
    for label in frame.laser_labels:
        obj = {"class": classes[label.type], "center_x": label.box.center_x, "center_y": label.box.center_y, "center_z": label.box.center_z,
              "width": label.box.width, "length": label.box.length, "height": label.box.height, "heading": label.box.heading,
              "num_lidar_points_in_box": label.num_lidar_points_in_box}
        res.append(obj)

    with open(f"{out_dir}/labels/labels_{idx:03}.json", 'w') as f:
        json.dump(res, f)

    idx += 1

