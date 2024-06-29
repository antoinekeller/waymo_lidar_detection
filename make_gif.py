"""Use this basic script to generate GIF from images"""

import cv2
import os
import imageio
from tqdm import tqdm

folder = "dataset/training/individual_files_training_segment-10082223140073588526_6140_000_6160_000_with_camera_labels/images/front/"
frames = []

for img_path in tqdm(sorted(os.listdir(folder))):
    frame = cv2.imread(f"{folder}/{img_path}", )
    frame = cv2.resize(frame, (480, 360))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frames.append(frame)

imageio.mimsave("output.gif", frames, fps=15, loop=1000)