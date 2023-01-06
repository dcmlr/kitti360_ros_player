# Copyright 2022 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
# and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or other materials provided
# with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
# endorse or promote products derived from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import sys
from tqdm import tqdm
import copy
from sklearn.neighbors import KDTree
from pyntcloud import PyntCloud
import numpy as np
import pandas as pd
"""
This script generates new folders in the data_3d_raw directory that then
contain the velodyne pointclouds, but with a ring value that represents the
recovered label from the 3d semantics"

For each point in velodyne pointcloud frame we take the closest point in the 3d
semantics dataset within 20cm radius and take its semantic label. If no point
is found the point is marked as unlabeled.
"""

KITTI360_DIR = sys.argv[1]
RAW_3D_DIR = os.path.join(KITTI360_DIR, "data_3d_raw")
SEMANTIC_DIR = os.path.join(KITTI360_DIR, "data_3d_semantics", "train")
POSES_DIR = os.path.join(KITTI360_DIR, "data_poses")
SEARCH_RADIUS = 0.2

SKIP_EXISTING = False


def main():
    for sequence_dir in ["2013_05_28_drive_0000_sync"]:
        # for sequence_dir in os.listdir(RAW_3D_DIR):
        print(f"{sequence_dir=}")
        path_in = os.path.join(RAW_3D_DIR, sequence_dir,
                               "velodyne_points/data")
        path_out = os.path.join(RAW_3D_DIR, sequence_dir,
                                "velodyne_points_labeled")
        semantics_path = os.path.join(SEMANTIC_DIR, sequence_dir, "static")

        # semantic mapping
        df_sem = pd.Series(
            os.listdir(semantics_path)).to_frame(name="filename")
        df_sem["start_frame"], df_sem["end_frame"] = zip(
            *df_sem["filename"].str.split('[_.]').str[:2].apply(
                lambda x: (int(x[0]), int(x[1]))))
        df_sem = df_sem.sort_values(by="start_frame")

        # create outdir if it does not exist
        if not os.path.exists(path_out):
            os.mkdir(path_out)

        current_semantic_filename = None
        current_semantic_kdtree = None
        current_semantic_points = None
        current_semantic_points_IDs = None

        # read poses for current sequence
        poses = pd.read_csv(os.path.join(POSES_DIR, sequence_dir, "poses.txt"),
                            sep=" ",
                            header=None)
        # index = frame id = (was first column)
        poses = poses.set_index(0)
        # pandas Series index is frame_id
        poses = poses.apply(lambda row: np.array(row).reshape(3, 4), axis=1)

        # TODO determine max frame
        max_frame = int(sorted(os.listdir(path_in))[-1].replace(".bin", ""))
        for frame in tqdm(range(0, max_frame), desc="frame"):
            frame_filename = '{:010d}'.format(frame) + ".bin"
            outputfilepath = os.path.join(
                path_out, frame_filename.replace(".bin", ".npy"))

            if SKIP_EXISTING and os.path.exists(outputfilepath):
                print(f"skipping {frame=} in {sequence_dir}. Exists already.")
                continue

            points = np.fromfile(os.path.join(path_in, frame_filename),
                                 dtype=np.float32).reshape(-1, 4)

            # extracted from transform.py
            trans_velo_to_imu = np.array(
                [[0.99992906, 0.0057743, 0.01041756, 0.77104934],
                 [0.00580536, -0.99997879, -0.00295331, 0.29854144],
                 [0.01040029, 0.00301357, -0.99994137, -0.83628022],
                 [0, 0, 0, 1]])

            # get latest possible pose to transform velodyne points
            temp_frame = frame if frame > 0 else 1
            trans_imu_to_world = poses.loc[poses.index[
                poses.index <= temp_frame][-1]]
            trans_imu_to_world = np.vstack(
                [trans_imu_to_world,
                 np.array([0, 0, 0, 1])])

            # this is the transform from velo -> world
            trans_velo_to_world = np.matmul(trans_imu_to_world,
                                            trans_velo_to_imu)

            # apply transformation to each point
            # NOTE possible performance improvement
            points_copy = points.copy()
            points_copy[:, 3] = 1
            points[:, :3] = np.apply_along_axis(
                lambda p: np.matmul(trans_velo_to_world, p),
                axis=1,
                arr=points_copy)[:, :3]

            # load semantics file from drive or cache
            cand = df_sem[(df_sem["start_frame"] <= frame)
                          & (df_sem["end_frame"] > frame)]
            if not cand.empty:
                sem_filename = cand["filename"].iloc[0]
                if current_semantic_filename != sem_filename:
                    current_semantic_filename = sem_filename
                    # load input data as kdtree
                    current_semantic_points = PyntCloud.from_file(
                        os.path.join(semantics_path, sem_filename)).points
                    current_semantic_kdtree = KDTree(
                        current_semantic_points[["x", "y", "z"]])
                    current_semantic_points_IDs = current_semantic_points[
                        "semantic"].copy()
                    # if point is not found we call iloc[-1] and then the last line is chosen
                    current_semantic_points_IDs["default"] = 0
            else:
                print(f"skipping {frame=}, no static semantic 3d data")
                continue

            # determine ring values
            # at index i in indices we get the index of the closest labeled point
            indices, _ = current_semantic_kdtree.query_radius(
                points[:, :3],
                r=SEARCH_RADIUS,
                return_distance=True,
                sort_results=True)

            # get label for closest semantic point if there is one, otherwise label (0 = undefined)
            chosen_semantic_indices = [
                chosen[0] if chosen.size > 0 else -1 for chosen in indices
            ]
            labels = np.array([
                current_semantic_points_IDs.iloc[chosen_semantic_indices].
                to_numpy()
            ]).T

            # add ring values to points
            points = np.append(points, labels, axis=1)
            points = np.core.records.fromarrays(points.T,
                                                dtype=[('x', 'float32'),
                                                       ('y', 'float32'),
                                                       ('z', 'float32'),
                                                       ('remission',
                                                        'float32'),
                                                       ('ring', 'uint16')])
            np.save(outputfilepath, points)


main()
