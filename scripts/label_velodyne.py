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

SKIP_EXISTING = True


def main():
    for sequence_dir in os.listdir(RAW_3D_DIR):
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

            # shift points to gpsimu according to car layout
            points[:, 0] = points[:, 0] + 0.81
            points[:, 1] = points[:, 1] - 0.32
            points[:, 2] = points[:, 2] + 0.83

            # rotate
            # x does not change
            # y is inverted
            # z is inverted
            points[:, 1] = (-1) * points[:, 1]
            points[:, 2] = (-1) * points[:, 2]

            # get latest possible pose to transform velodyne points
            temp_frame = frame if frame > 0 else 1
            tf_matrix = poses.loc[poses.index[poses.index <= temp_frame][-1]]

            # rotate and then translate
            # NOTE possible performance improvement
            points[:, :3] = np.apply_along_axis(
                lambda p: np.matmul(tf_matrix[:, :3], p) + tf_matrix[:, 3],
                axis=1,
                arr=points[:, :3])

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
            else:
                print(f"skipping {frame=}, no static semantic 3d data")
                continue

            # determine ring values
            indices, _ = current_semantic_kdtree.query_radius(
                points[:, :3],
                r=SEARCH_RADIUS,
                return_distance=True,
                sort_results=True)
            # get label for choses point
            labels = []
            for chosen in indices:
                if len(chosen) == 0:
                    # if no points found --> unlabeled
                    labels.append(0)
                else:
                    # otherwise chose closest
                    labels.append(
                        current_semantic_points["semantic"].iloc[chosen[0]])
            # make it a pretty column
            labels = np.array([labels]).T

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
