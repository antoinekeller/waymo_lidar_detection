# waymo_lidar_detection

## Dataset extraction

This network is trained on the [Waymo Open Dataset](https://waymo.com/open/download/).
I used the latest version (March 2024) of the [Perception Dataset](https://console.cloud.google.com/storage/browser/waymo_open_dataset_v_1_4_3)

In the individual files, you will find _.tfrecord_ segments splitted in training/validation/testing folders. I suggest to download them (manually or with _gsutil_) inside `tfrecords/training` and `tfrecords/validation`.

After downloading required python package (like `waymo-open-dataset-tf-2-12-0`), use this command to extract PLY, images, labels, and camera intrinsic/extrinsic poses.

```
python3 extract_tfrecord.py tfrecords/training/individual_files_training_segment_[...]_with_camera_labels.tfrecord dataset/training/
```

This will put your data inside `dataset/training`

Note that this worked (for me) on Ubuntu 22.04, not on MacOS.

This will write to `dataset/training/individual_files_training_segment_[...]_with_camera_labels`:

- `pointclouds`: containing LiDAR scans at each timestamp (PLY)
- `labels`: containing objects detected as oriented bounding boxes with classes
- `cameras.json`: intrinsic/extrinsic of the 5 cameras surrounding the vehicle
- `images`: containing images

To enable image extraction simultaneously, you can add `--front`, `--front-left`, `--front-right`, `--side-left` or `side-right` at the end of previous command.

<p style="text-align:center">
<em>Front camera</em></br>
<img src="res/front_example.png"  width="400" alt>
</p>

|             Front-left             |           Front-right            |
| :--------------------------------: | :------------------------------: |
| ![lol](res/front_left_example.png) | ![](res/front_right_example.png) |

|             Side-left             |           Side-right            |
| :-------------------------------: | :-----------------------------: |
| ![lol](res/side_left_example.png) | ![](res/side_right_example.png) |

If you want to do it for many tfrecords (which is a good idea), you can use this basic script:

```
sh extract_tfrecords.sh
```

## Insights on the dataset

<p style="text-align:center">
<em>By day</em></br>
<img src="res/front_by_day.gif"  width="400">
</p>

<p style="text-align:center">
<em>By night</em></br>
<img src="res/front_by_night.gif"  width="400">
</p>

<p style="text-align:center">
<em>Under the rain</em></br>
<img src="res/front_by_rain.gif"  width="400">
</p>

There are usually ~200 frames per sample. It corresponds to ~30 seconds, meaning that we are running at ~6.7 fps.

The LiDAR scans are fused into one single pointcloud surrounding the vehicle. They contain >100k points and can see up to 70 meters around the car. The ground is at z = 0. The car is located at the origin (0, 0, 0). The forward direction is +X, +Y points to the left of the car, Z points upwards.

<p style="text-align:center">
<em>Fused point cloud</em></br>
<img src="res/pointcloud_example.png"  width="600">
</p>
