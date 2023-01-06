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

import numpy as np
from scipy.spatial.transform import *

# transformations copied from calibrations directory
image_00 = np.array([
    0.0371783278, -0.0986182135, 0.9944306009, 1.5752681039, 0.9992675562,
    -0.0053553387, -0.0378902567, 0.0043914093, 0.0090621821, 0.9951109327,
    0.0983468786, -0.6500000000
]).reshape(3, 4)
image_01 = np.array([
    0.0194000864, -0.1051529641, 0.9942668106, 1.5977241400, 0.9997374956,
    -0.0100836652, -0.0205732716, 0.5981494900, 0.0121891942, 0.9944049345,
    0.1049297370, -0.6488433108
]).reshape(3, 4)
image_02 = np.array([
    0.9995185086, 0.0041276589, -0.0307524527, 0.7264036936, -0.0307926666,
    0.0100608424, -0.9994751579, -0.1499658517, -0.0038160970, 0.9999408692,
    0.0101830998, -1.0686400091
]).reshape(3, 4)
image_03 = np.array([
    -0.9996821702, 0.0005703407, -0.0252038325, 0.7016842127, -0.0252033830,
    0.0007820814, 0.9996820384, 0.7463650950, 0.0005898709, 0.9999995315,
    -0.0007674583, -1.0751978255
]).reshape(3, 4)
cam_to_velo = np.array([
    0.04307104361, -0.08829286498, 0.995162929, 0.8043914418, -0.999004371,
    0.007784614041, 0.04392796942, 0.2993489574, -0.01162548558, -0.9960641394,
    -0.08786966659, -0.1770225824
]).reshape(3, 4)
sick_to_velo = np.array([
    0.9998328856, -0.01305514558, -0.01279702916, -0.3971222434, 0.01322436405,
    0.9998250388, 0.01322905751, -0.009085164561, 0.0126220829, -0.01339607931,
    0.9998305997, -0.07072622777
]).reshape(3, 4)

# use transformation between imu and cam0 and velo and cam0
cam_to_velo_44 = np.vstack([cam_to_velo, np.array([0, 0, 0, 1])])
velo_to_cam_44 = np.linalg.inv(cam_to_velo_44)
velo_to_imu = np.matmul(image_00, velo_to_cam_44)

sick_to_imu = np.matmul(np.vstack([velo_to_imu,
                                   np.array([0, 0, 0, 1])]),
                        np.vstack([sick_to_velo,
                                   np.array([0, 0, 0, 1])]))[:3, :]


def transformation_matrix_to_ROS_static_transform(matrix):
    # input: numpy array in shape 3,4
    # X Y Z yaw(z) pitch(y) roll(x)
    print(
        " ".join(map(str, matrix[:, 3:].T[0])),
        " ".join(map(str,
                     Rotation.from_matrix(matrix[:, :3]).as_euler("ZYX"))))


print("image 00 -> imu")
transformation_matrix_to_ROS_static_transform(image_00)
print("image 01 -> imu")
transformation_matrix_to_ROS_static_transform(image_01)
print("image 02 -> imu")
transformation_matrix_to_ROS_static_transform(image_02)
print("image 03 -> imu")
transformation_matrix_to_ROS_static_transform(image_03)
print("velodyne -> imu")
transformation_matrix_to_ROS_static_transform(velo_to_imu)
print("sick -> imu")
transformation_matrix_to_ROS_static_transform(sick_to_imu)
