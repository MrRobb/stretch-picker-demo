#!/usr/bin/env python3

"""
This module implements the detection of the objects on the field of view of the camera. 

INPUT
-----
It takes the ROS /camera/rgb/image_raw topic and uses a YOLOv5s model to detect the objects.

OUTPUT
------
The module is implemented as a ROS node, and it publishes the detected objects on the /detections topic.
"""
from __future__ import print_function
import cv2
import rospy
import torch
import numpy as np
import pandas as pd
from typing import List, Optional
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from actionlib import SimpleActionClient
from control_msgs.msg import PointHeadAction
from image_geometry import PinholeCameraModel
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Pose2D
from scipy.spatial.transform import Rotation
from numba import njit
import numpy as np

# GLOBAL PARAMETERS (can be changed)
CAMERA_TOPIC = "/camera/color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
DEPTH_IMAGE_TOPIC = "/camera/aligned_depth_to_color/image_raw"

POINT_HEAD_ACTION = "/head_controller/point_head_action"
MAX_ITERATIONS = 3  # Iterations for extracting the camera intrinsic parameters

YOLO_MODEL = torch.hub.load("ultralytics/yolov5", "yolov5s", pretrained=True)

OUTPUT_TOPIC = "/objects/detected"

# GLOBAL VARIABLES (should not be changed)
CAMERA_INTRINSICS = PinholeCameraModel()
OUTPUT_PUBLISHER: Optional[rospy.Publisher] = None
DETECTED_OBJECTS: List[Detection2D] = []
DEPTH_IMAGE = None


@njit(fastmath=True)
def numba_image_to_pointcloud(depth_image, bounding_box, camera_matrix):
    x_min, y_min, x_max, y_max = bounding_box
    h, w = depth_image.shape

    # check and correct the bounding box to be within the rgb_image
    x_min = int(round(max(0, x_min)))
    y_min = int(round(max(0, y_min)))
    x_max = int(round(min(w - 1, x_max)))
    y_max = int(round(min(h - 1, y_max)))
    if x_max < x_min:
        x_max = x_min
    if y_max < y_min:
        y_max = y_min

    f_x = camera_matrix[0, 0]
    c_x = camera_matrix[0, 2]
    f_y = camera_matrix[1, 1]
    c_y = camera_matrix[1, 2]

    out_w = (x_max - x_min) + 1
    out_h = (y_max - y_min) + 1
    points = np.empty((out_h * out_w, 3), dtype=np.float32)

    i = 0
    x = x_min
    while x <= x_max:
        y = y_min
        while y <= y_max:
            z_3d = depth_image[y, x] / 1000.0
            x_3d = ((x - c_x) / f_x) * z_3d
            y_3d = ((y - c_y) / f_y) * z_3d
            points[i] = (x_3d, y_3d, z_3d)
            i += 1
            y += 1
        x += 1

    return points


def fit_plane_to_height_image(height_image, mask):
    # Perform a least squares fit of a plane to the masked region of
    # the height_image. Find the 3 element vector a for the equation
    # aX ~= z where X[:,i] = [x_i, y_i, 1]^T, z[i] = z_i and a=[alpha,
    # beta, gamma] such that alpha*x + beta*y + gamma ~= z .
    z = height_image[mask > 0]
    nonzero = cv2.findNonZero(mask)
    perform_test = False
    if perform_test:
        print("z.shape =", z.shape)
        print(z)
        for n in range(10):
            test_x, test_y = nonzero[n][0]
            test_z = height_image[test_y, test_x]

            print("x, y, z, z_test =", test_x, test_y, test_z, z[n])
    num_points, s1, s2 = nonzero.shape
    nonzero = np.reshape(nonzero, (num_points, 2))
    X_T = np.append(nonzero, np.ones((num_points, 1)), axis=1)
    a0 = np.matmul(z, X_T)
    A1 = np.matmul(X_T.transpose(), X_T)
    A1 = np.linalg.inv(A1)
    a = np.matmul(a0, A1)
    X = X_T.transpose()
    # aX ~= z
    return a, X, z


def fit_plane_to_height_image_error(a, X, z):
    # Calculate the fit error for the plane.
    z_fit = np.matmul(a, X)
    fit_error = z - z_fit
    return fit_error, z_fit


def svd_fit(points, verbose=False):
    # calculate and subtract the mean
    center = np.mean(points, axis=0)

    if verbose:
        print("center =", center)

    # make the point distribution have zero mean
    points_zero_mean = points - center

    if verbose:
        print("points_zero_mean[:5] =", points_zero_mean[:5])
        print("points_zero_mean.shape =", points_zero_mean.shape)

    # find the covariance matrix, C, for the data
    C = np.cov(points_zero_mean.transpose())

    # find the SVD of the covariance matrix
    u, s, vh = np.linalg.svd(C)

    e0 = np.reshape(u[:, 0], (3, 1))
    e1 = np.reshape(u[:, 1], (3, 1))
    e2 = np.reshape(u[:, 2], (3, 1))

    center = np.reshape(center, (3, 1))

    return center, e0, e1, e2


class FitPlane:
    def __init__(self):
        self.d = None
        self.n = None
        # defines the direction from points to the camera
        self.towards_camera = np.reshape(np.array([0.0, 0.0, -1.0]), (3, 1))

    def set_plane(self, n, d):
        self.n = n
        self.d = d
        self.update()

    def update(self):
        return

    def get_plane_normal(self):
        return -self.n if self.n is not None else None

    def get_plane_coordinate_system(self):
        z_p = -self.n
        # two options to avoid selecting poor choice that is almost
        # parallel to z_p
        x_approx = np.reshape(np.array([1.0, 0.0, 0.0]), (3, 1))
        x_approx_1 = x_approx - (np.matmul(z_p.transpose(), x_approx) * z_p)
        x_approx = np.reshape(np.array([0.0, 1.0, 0.0]), (3, 1))
        x_approx_2 = x_approx - (np.matmul(z_p.transpose(), x_approx) * z_p)
        x_approx_1_mag = np.linalg.norm(x_approx_1)
        x_approx_2_mag = np.linalg.norm(x_approx_2)
        if x_approx_1_mag > x_approx_2_mag:
            x_p = x_approx_1 / x_approx_1_mag
        else:
            x_p = x_approx_2 / x_approx_2_mag
        y_p = np.reshape(np.cross(z_p.flatten(), x_p.flatten()), (3, 1))

        p_origin = self.d * self.n
        return x_p, y_p, z_p, p_origin

    def get_points_on_plane(
        self, plane_origin=None, side_length=1.0, sample_spacing=0.01
    ):
        x_p, y_p, z_p, p_origin = self.get_plane_coordinate_system()
        h = side_length / 2.0
        if plane_origin is None:
            plane_list = [
                np.reshape((x_p * alpha) + (y_p * beta) + p_origin, (3,))
                for alpha in np.arange(-h, h, sample_spacing)
                for beta in np.arange(-h, h, sample_spacing)
            ]
        else:
            plane_origin = np.reshape(plane_origin, (3, 1))
            plane_list = [
                np.reshape((x_p * alpha) + (y_p * beta) + plane_origin, (3,))
                for alpha in np.arange(-h, h, sample_spacing)
                for beta in np.arange(-h, h, sample_spacing)
            ]

        plane_array = np.array(plane_list)
        return plane_array

    def abs_dist(self, points_array):
        out = np.abs(
            np.matmul(self.n.transpose(), points_array.transpose()) - self.d
        ).flatten()
        return out

    def height(self, points_array):
        # positive is closer to the camera (e.g., above floor)
        # negative is farther from the camera (e.g., below floor)?
        out = -(
            np.matmul(self.n.transpose(), points_array.transpose()) - self.d
        ).flatten()
        return out

    def get_points_nearby(self, points_array, dist_threshold_mm):
        # return points that are within a distance from the current plane
        if (self.n is not None) and (self.d is not None):
            dist = np.abs(
                np.matmul(self.n.transpose(), points_array.transpose()) - self.d
            ).flatten()
            # only points < dist_threshold meters away from the plane are
            # considered in the fit dist_threshold = 0.2 #1.0 #0.5 #0.2

            dist_threshold_m = dist_threshold_mm / 1000.0
            thresh_test = np.abs(dist) < dist_threshold_m
            points = points_array[thresh_test, :]
        else:
            points = points_array
        return points

    def fit_svd(
        self,
        points_array,
        dist_threshold_mm=200.0,
        prefilter_points=False,
        verbose=True,
    ):
        # relevant numpy documentation for SVD:
        #
        # "When a is a 2D array, it is factorized as u @ np.diag(s) @ vh"
        #
        # " The rows of vh are the eigenvectors of A^H A and the
        # columns of u are the eigenvectors of A A^H. In both cases
        # the corresponding (possibly non-zero) eigenvalues are given
        # by s**2. "

        if prefilter_points:
            # only fit to points near the current plane
            points = self.get_points_nearby(points_array, dist_threshold_mm)
        else:
            points = points_array

        center, e0, e1, e2 = svd_fit(points, verbose)

        # find the smallest eigenvector, which corresponds to the
        # normal of the plane
        n = e2

        # ensure that the direction of the normal matches our convention
        approximate_up = self.towards_camera
        if np.matmul(n.transpose(), approximate_up) > 0.0:
            n = -n
        if verbose:
            print("SVD fit")
            print("n =", n)
            print("np.linalg.norm(n) =", np.linalg.norm(n))

        # center = np.reshape(center, (3,1))
        d = np.matmul(n.transpose(), center)
        if verbose:
            print("d =", d)

        self.d = d
        self.n = n
        if verbose:
            print("self.d =", self.d)
            print("self.n =", self.n)
        self.update()

    def fit_ransac(
        self,
        points_array,
        dist_threshold=0.2,
        ransac_inlier_threshold_m=0.04,
        use_density_normalization=False,
        number_of_iterations=100,
        prefilter_points=False,
        verbose=True,
    ):
        # Initial RANSAC algorithm based on pseudocode on Wikipedia
        # https://en.wikipedia.org/wiki/Random_sample_consensus

        if prefilter_points:
            # only fit to points near the current plane
            dist_threshold_mm = dist_threshold * 1000.0
            points = self.get_points_nearby(points_array, dist_threshold_mm)
        else:
            points = points_array

        num_points = points.shape[0]
        indices = np.arange(num_points)

        ransac_threshold_m = ransac_inlier_threshold_m

        min_num_inliers = 100

        approximate_up = self.towards_camera

        # should be well above the maximum achievable error, since
        # error is average distance in meters

        best_model_inlier_selector = None
        best_model_inlier_count = 0

        for i in range(number_of_iterations):
            if verbose:
                print("RANSAC iteration", i)
            candidate_inliers = points[np.random.choice(indices, 3), :]
            c0, c1, c2 = candidate_inliers
            # fit plane to candidate inliers
            n = np.cross(c1 - c0, c2 - c0)
            if np.dot(n, approximate_up) > 0.0:
                n = -n
            n = np.reshape(n / np.linalg.norm(n), (3, 1))
            c0 = np.reshape(c0, (3, 1))
            d = np.matmul(n.transpose(), c0)

            dist = np.abs(np.matmul(n.transpose(), points.transpose()) - d).flatten()
            select_model_inliers = dist < ransac_threshold_m
            if use_density_normalization:
                inliers = points[select_model_inliers]
                # square grid with this many bins to a side, small
                # values (e.g., 10 and 20) can result in the fit being
                # biased towards edges of the planar region
                num_bins = 100  # num_bins x num_bins = total bins
                density_image, mm_per_pix, x_indices, y_indices = create_density_image(
                    inliers,
                    self,
                    image_width_pix=num_bins,
                    view_width_m=5.0,
                    return_indices=True,
                )
                density_image = np.reciprocal(density_image, where=density_image != 0.0)
                number_model_inliers = np.int(
                    np.round(np.sum(density_image[y_indices, x_indices]))
                )
            else:
                number_model_inliers = np.count_nonzero(select_model_inliers)
            if number_model_inliers > min_num_inliers:
                if verbose:
                    print("model found with %d inliers" % number_model_inliers)
                if number_model_inliers > best_model_inlier_count:
                    if verbose:
                        print(
                            "model has more inliers than the previous best model, so updating"
                        )
                    best_model_n = n
                    best_model_d = d
                    best_model_inlier_count = number_model_inliers
                    best_model_inlier_selector = select_model_inliers
                    best_model_inliers = None
                    best_model_error = None
                elif number_model_inliers == best_model_inlier_count:
                    if verbose:
                        print(
                            "model has the same number of inliers as the previous best model, so comparing"
                        )
                    model_inliers = points[select_model_inliers]
                    # error is the average distance of points from the plane
                    # sum_i | n^T p_i - d |
                    # should be able to make this faster by selecting from the already computed distances
                    new_error = np.average(
                        np.abs(np.matmul(n.transpose(), model_inliers.transpose()) - d)
                    )
                    if best_model_inliers is None:
                        best_model_inliers = points[best_model_inlier_selector]
                    if best_model_error is None:
                        # should be able to make this faster by
                        # selecting from the already computed
                        # distances
                        best_model_error = np.average(
                            np.abs(
                                np.matmul(
                                    best_model_n.transpose(),
                                    best_model_inliers.transpose(),
                                )
                                - best_model_d
                            )
                        )
                    if new_error < best_model_error:
                        if verbose:
                            print(
                                "model has a lower error than the previous model, so updating"
                            )
                        best_model_n = n
                        best_model_d = d
                        best_model_inlier_count = number_model_inliers
                        best_model_inlier_selector = select_model_inliers
                        best_model_inliers = model_inliers
                        best_model_error = new_error
        if best_model_inlier_count > 0:
            if verbose:
                print("RANSAC FINISHED")
                print("new model found by RANSAC:")
            self.d = best_model_d
            self.n = best_model_n
            if verbose:
                print("self.d =", self.d)
                print("self.n =", self.n)
            self.update()
        else:
            print("RANSAC FAILED TO FIND A MODEL")


def filter_points(points_array, camera_matrix, box_2d, min_box_side_m, max_box_side_m):
    # Decompose the camera matrix.
    f_x = camera_matrix[0, 0]
    c_x = camera_matrix[0, 2]
    f_y = camera_matrix[1, 1]
    c_y = camera_matrix[1, 2]

    # These need to be flipped with respect to the basic update
    # function to account for the rotation applied as part of the
    # head orientation estimation.
    x0, y0, x1, y1 = box_2d
    detection_box_width_pix = y1 - y0
    detection_box_height_pix = x1 - x0

    z_min = min_box_side_m * min(
        f_x / detection_box_width_pix, f_y / detection_box_height_pix
    )
    z_max = max_box_side_m * max(
        f_x / detection_box_width_pix, f_y / detection_box_height_pix
    )

    z = points_array[:, 2]
    mask_z = (z > z_min) & (z < z_max)

    # TODO: Handle situations when the cropped rectangle contains no
    # reasonable depth values.

    # Second, filter for depths that are within one maximum head
    # length away from the median depth.
    remaining_z = z[mask_z]
    out_points = np.empty((0, 3), dtype=np.float32)
    if len(remaining_z) > 0:
        median_z = np.median(remaining_z)
        min_z = median_z - max_box_side_m
        max_z = median_z + max_box_side_m
        mask_z = (z > min_z) & (z < max_z)
        remaining_z = z[mask_z]
        if len(remaining_z) > 0:
            out_points = points_array[mask_z]

    return out_points


def landmarks_2d_to_3d(landmarks, camera_matrix, depth_image, default_z_3d):

    f_x = camera_matrix[0, 0]
    c_x = camera_matrix[0, 2]
    f_y = camera_matrix[1, 1]
    c_y = camera_matrix[1, 2]

    landmarks_3d = {}
    for name, xy in landmarks.items():
        x, y = xy
        z = depth_image[y, x]
        if z > 0:
            z_3d = z / 1000.0
        else:
            z_3d = default_z_3d
        x_3d = ((x - c_x) / f_x) * z_3d
        y_3d = ((y - c_y) / f_y) * z_3d
        landmarks_3d[name] = (x_3d, y_3d, z_3d)

    return landmarks_3d


def bounding_box_2d_to_3d(
    points_array, box_2d, camera_matrix, head_to_camera_mat=None, fit_plane=False
):

    x0, y0, x1, y1 = box_2d

    f_x = camera_matrix[0, 0]
    c_x = camera_matrix[0, 2]
    f_y = camera_matrix[1, 1]
    c_y = camera_matrix[1, 2]

    center_xy_pix = np.array([0.0, 0.0])
    center_xy_pix[0] = (x0 + x1) / 2.0
    center_xy_pix[1] = (y0 + y1) / 2.0
    # These need to be flipped with respect to the basic update
    # function to account for the rotation applied as part of the
    # head orientation estimation.
    detection_box_width_pix = y1 - y0
    detection_box_height_pix = x1 - x0

    num_points = points_array.shape[0]
    if num_points >= 1:
        box_depth = np.median(points_array, axis=0)[2]
    else:
        print(
            "WARNING: No reasonable depth image points available in the detected rectangle. No work around currently implemented for lack of depth estimate."
        )
        return None

    # Convert to 3D point in meters using the camera matrix.
    center_z = box_depth
    center_x = ((center_xy_pix[0] - c_x) / f_x) * center_z
    center_y = ((center_xy_pix[1] - c_y) / f_y) * center_z

    detection_box_width_m = (detection_box_width_pix / f_x) * box_depth
    detection_box_height_m = (detection_box_height_pix / f_y) * box_depth

    if head_to_camera_mat is None:
        R = np.identity(3)
        quaternion = Rotation.from_dcm(R).as_quat()
        x_axis = R[:3, 0]
        y_axis = R[:3, 1]
        z_axis = R[:3, 2]
    else:
        quaternion = Rotation.from_dcm(head_to_camera_mat).as_quat()
        x_axis = head_to_camera_mat[:3, 0]
        y_axis = head_to_camera_mat[:3, 1]
        z_axis = head_to_camera_mat[:3, 2]

    plane = None

    # Find suitable 3D points within the Face detection box. If there
    # are too few points, do not proceed with fitting a plane.
    num_points = points_array.shape[0]
    min_number_of_points_for_plane_fitting = 16
    enough_points = num_points >= min_number_of_points_for_plane_fitting
    if fit_plane and (not enough_points):
        print(
            "WARNING: There are too few points from the depth image for plane fitting. number of points =",
            num_points,
        )
    elif fit_plane:
        plane = FitPlane()
        plane.fit_svd(points_array, verbose=False)

        #####################################
        # Find the points on the fit plane corresponding with the
        # four Face rectangle corners. Then, use the mean of the 4
        # points as the 3D center for the marker.
        d = plane.d
        n = plane.n

        def pix_to_plane(pix_x, pix_y):
            z = 1.0
            x = ((pix_x - c_x) / f_x) * z
            y = ((pix_y - c_y) / f_y) * z
            point = np.array([x, y, z])
            ray = point / np.linalg.norm(point)
            point = ((d / np.matmul(n.transpose(), ray)) * ray).flatten()
            return point

        corners = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
        corner_points = []
        total_corner = np.array([0.0, 0.0, 0.0])
        for (pix_x, pix_y) in corners:
            corner_point = pix_to_plane(pix_x, pix_y)
            total_corner += corner_point
            corner_points.append(corner_point)
        center_x, center_y, center_z = total_corner / 4.0

        # Use the corners on the fit plane to estimate the x and y
        # axes for the marker.
        top_left, top_right, bottom_right, bottom_left = corner_points

        y_axis = (top_left + top_right) - (bottom_left + bottom_right)
        y_length = np.linalg.norm(y_axis)
        if y_length > 0.0:
            y_axis = y_axis / y_length
        else:
            y_axis = None

        x_axis = (top_right + bottom_right) - (top_left + bottom_left)
        x_length = np.linalg.norm(x_axis)
        if x_length > 0.0:
            x_axis = x_axis / x_length
        else:
            x_axis = None

        #####################################

        plane_normal = plane.get_plane_normal()

        if x_axis is not None:
            old_x_axis = np.reshape(x_axis, (3, 1))
        else:
            old_x_axis = np.array([1.0, 0.0, 0.0])

        if y_axis is not None:
            old_y_axis = np.reshape(y_axis, (3, 1))
        else:
            old_y_axis = np.array([0.0, 1.0, 0.0])

        new_z_axis = plane_normal
        # The following methods directly use the z axis from the
        # plane fit.
        if (x_axis is not None) and (y_axis is None):
            # For tests with the two wrist markers, this method
            # appeared to be the best. It showed the most
            # stability. In particular, it showed the least
            # rotation around the normal to the marker.
            new_x_axis = old_x_axis - (
                np.matmul(new_z_axis.transpose(), old_x_axis) * new_z_axis
            )
            new_x_axis = new_x_axis / np.linalg.norm(new_x_axis)
            new_y_axis = np.reshape(
                np.cross(new_z_axis.flatten(), new_x_axis.flatten()), (3, 1)
            )
        elif (x_axis is None) and (y_axis is not None):
            new_y_axis = old_y_axis - (
                np.matmul(new_z_axis.transpose(), old_y_axis) * new_z_axis
            )
            new_y_axis = new_y_axis / np.linalg.norm(new_y_axis)
            new_x_axis = np.reshape(
                np.cross(new_y_axis.flatten(), new_z_axis.flatten()), (3, 1)
            )
        elif False:
            # Attempt to reduce bias due to selecting one of the
            # old axes by averaging the results from both axes.
            new_x_axis_1 = old_x_axis - (
                np.matmul(new_z_axis.transpose(), old_x_axis) * new_z_axis
            )
            new_x_axis_1 = new_x_axis_1 / np.linalg.norm(new_x_axis_1)

            new_y_axis_1 = np.reshape(
                np.cross(new_z_axis.flatten(), new_x_axis_1.flatten()), (3, 1)
            )

            new_y_axis_2 = old_y_axis - (
                np.matmul(new_z_axis.transpose(), old_y_axis) * new_z_axis
            )
            new_y_axis_2 = new_y_axis_2 / np.linalg.norm(new_y_axis_2)

            new_y_axis = (new_y_axis_1 + new_y_axis_2) / 2.0
            new_y_axis = new_y_axis / np.linalg.norm(new_y_axis)
            new_x_axis = np.reshape(
                np.cross(new_y_axis.flatten(), new_z_axis.flatten()), (3, 1)
            )
        else:
            if (x_axis is None) and (y_axis is None):
                print(
                    "WARNING: The detected corners did not project to reasonable 3D points on the fit plane."
                )
                # print('         corners[0] =', corners[0])
                new_y_axis = old_y_axis
                new_x_axis = old_x_axis
            else:
                # Attempt to reduce bias due to selecting one of the
                # old axes by averaging the results from both axes.
                new_y_axis_1 = old_y_axis - (
                    np.matmul(new_z_axis.transpose(), old_y_axis) * new_z_axis
                )
                new_y_axis_1 = new_y_axis_1 / np.linalg.norm(new_y_axis_1)

                new_x_axis_1 = np.reshape(
                    np.cross(new_y_axis_1.flatten(), new_z_axis.flatten()), (3, 1)
                )

                new_x_axis_2 = old_x_axis - (
                    np.matmul(new_z_axis.transpose(), old_x_axis) * new_z_axis
                )
                new_x_axis_2 = new_x_axis_2 / np.linalg.norm(new_x_axis_2)

                new_x_axis = (new_x_axis_1 + new_x_axis_2) / 2.0
                new_x_axis = new_x_axis / np.linalg.norm(new_x_axis)
                new_y_axis = np.reshape(
                    np.cross(new_z_axis.flatten(), new_x_axis.flatten()), (3, 1)
                )

        x_axis = new_x_axis.flatten()
        y_axis = new_y_axis.flatten()
        z_axis = new_z_axis.flatten()

        R = np.identity(3)
        R[:3, 0] = x_axis
        R[:3, 1] = y_axis
        R[:3, 2] = z_axis

        quaternion = Rotation.from_dcm(R).as_quat()

    if plane is not None:
        simple_plane = {"n": plane.n, "d": plane.d}
    else:
        simple_plane = None

    box_3d = {
        "center_xyz": (center_x, center_y, center_z),
        "quaternion": quaternion,
        "x_axis": x_axis,
        "y_axis": y_axis,
        "z_axis": z_axis,
        "width_m": detection_box_width_m,
        "height_m": detection_box_height_m,
        "width_pix": detection_box_width_pix,
        "height_pix": detection_box_height_pix,
        "plane": simple_plane,
    }

    return box_3d


def detections_2d_to_3d(
    detections_2d,
    rgb_image,
    camera_info,
    depth_image,
    fit_plane: bool = False,
    min_box_side_m=None,
    max_box_side_m=None,
):

    orig_h, orig_w, c = rgb_image.shape

    def clip_xy(x_in, y_in):
        x_out = x_in
        y_out = y_in
        x_out = max(0, x_out)
        x_out = min(orig_w - 1, x_out)
        y_out = max(0, y_out)
        y_out = min(orig_h - 1, y_out)
        return x_out, y_out

    camera_matrix = np.reshape(camera_info.K, (3, 3))
    distortion_coefficients = np.array(camera_info.D)

    def clockwise_rotate_bounding_box(box_2d):
        x0, y0, x1, y1 = box_2d
        orig_x0 = (orig_w - 1) - y1
        orig_y0 = x0
        orig_x1 = (orig_w - 1) - y0
        orig_y1 = x1
        return (orig_x0, orig_y0, orig_x1, orig_y1)

    def counterclockwise_rotate_bounding_box(box_2d):
        x0, y0, x1, y1 = box_2d
        orig_x0 = y0
        orig_y0 = (orig_h - 1) - x1
        orig_x1 = y1
        orig_y1 = (orig_h - 1) - x0
        return (orig_x0, orig_y0, orig_x1, orig_y1)

    def clockwise_rotate_xy(x, y):
        return ((orig_w - 1) - y), x

    def counterclockwise_rotate_xy(x, y):
        return y, (orig_h - 1) - x

    rotvec = np.array([0.0, 0.0, 1.0]) * (-np.pi / 2.0)
    counterclockwise_rotate_mat = Rotation.from_rotvec(rotvec).as_matrix()

    detections_3d = []

    for h in detections_2d:
        box_3d = None
        landmarks_3d = None
        box_2d = h.get("box")
        label = h.get("label")
        ypr = h.get("ypr")
        landmarks_2d = h.get("landmarks")
        points_3d = None
        front = h.get("front")

        if box_2d is not None:
            box_2d = counterclockwise_rotate_bounding_box(box_2d)
            x0, y0, x1, y1 = box_2d
            x0, y0 = clip_xy(x0, y0)
            x1, y1 = clip_xy(x1, y1)

            if (
                (x0 < 0)
                or (y0 < 0)
                or (x1 < 0)
                or (y1 < 0)
                or (x0 >= orig_w)
                or (y0 >= orig_h)
                or (x1 >= orig_w)
                or (y1 >= orig_h)
                or (x0 >= x1)
                or (y0 >= y1)
            ):
                print("---------------")
                print(
                    "WARNING: detection bounding box goes outside of the original image dimensions or has other issues, so ignoring detection."
                )
                print("box_2d =", box_2d)
                print("rgb_image.shape =", rgb_image.shape)
                print("---------------")
                box_2d = None

        if landmarks_2d is not None:
            rotated_landmarks_2d = {}
            for name, xy in landmarks_2d.items():
                rotated_xy = counterclockwise_rotate_xy(xy[0], xy[1])
                x0, y0 = rotated_xy
                x0, y0 = clip_xy(x0, y0)
                rotated_landmarks_2d[name] = (x0, y0)
            landmarks_2d = rotated_landmarks_2d

        if ypr is not None:
            yaw, pitch, roll = ypr
            head_ypr = np.array([-yaw, pitch, roll])
            rotation_mat = Rotation.from_euler("yxz", head_ypr).as_matrix()
            head_to_camera_mat = np.matmul(counterclockwise_rotate_mat, rotation_mat)
        else:
            head_to_camera_mat = counterclockwise_rotate_mat

        if (box_2d is not None) or (landmarks_2d is not None) or (ypr is not None):

            box_depth_m = 0.0
            if box_2d is not None:
                points_3d = numba_image_to_pointcloud(
                    depth_image, box_2d, camera_matrix
                )
                if (min_box_side_m is not None) and (max_box_side_m is not None):
                    points_3d = filter_points(
                        points_3d, camera_matrix, box_2d, min_box_side_m, max_box_side_m
                    )
                box_3d = bounding_box_2d_to_3d(
                    points_3d,
                    box_2d,
                    camera_matrix,
                    head_to_camera_mat=head_to_camera_mat,
                    fit_plane=fit_plane,
                )
                if box_3d is None:
                    box_depth_m = None
                else:
                    box_depth_m = box_3d["center_xyz"][2]

            if landmarks_2d is not None:
                if box_depth_m is None:
                    landmarks_3d = None
                else:
                    landmarks_3d = landmarks_2d_to_3d(
                        landmarks_2d, camera_matrix, depth_image, box_depth_m
                    )

        detections_3d.append(
            {
                "box_3d": box_3d,
                "landmarks_3d": landmarks_3d,
                "box_2d": box_2d,
                "label": label,
                "ypr": ypr,
                "landmarks_2d": landmarks_2d,
                "points_3d": points_3d,
                "front": front,
            }
        )

    return detections_3d


def build_detection(df: pd.Series, image: Image) -> Detection2D:
    """
    Build detection message
    """
    (x_min, y_min, z_min) = CAMERA_INTRINSICS.projectPixelTo3dRay(
        (df["xmin"], df["ymin"])
    )
    (x_max, y_max, z_max) = CAMERA_INTRINSICS.projectPixelTo3dRay(
        (df["xmax"], df["ymax"])
    )
    xcenter = (x_min + x_max) / 2
    ycenter = (y_min + y_max) / 2
    zcenter = (z_min + z_max) / 2
    width = x_max - x_min
    height = y_max - y_min
    msg = Detection2D(
        header=image.header,
        results=[
            ObjectHypothesisWithPose(
                id=df["class"],
                score=df["confidence"],
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=xcenter, y=ycenter, z=zcenter),
                        orientation=None,
                    ),
                    covariance=list(np.zeros((36,), dtype=float)),
                ),
            )
        ],
        bbox=BoundingBox2D(
            center=Pose2D(x=xcenter, y=ycenter, theta=0.0),
            size_x=width,
            size_y=height,
        ),
        source_img=image,
    )
    return msg


def apply_yolo(image: Image):
    """
    Apply YOLOv7 to the image
    """
    if YOLO_MODEL is None:
        rospy.logerr("YOLOv7 model not loaded")
        return

    if image is None:
        rospy.logerr("Image is None")
        return

    # Save in the displayed image
    cv_image = CvBridge().imgmsg_to_cv2(image, "bgr8")
    results = YOLO_MODEL(cv_image)
    results.render()

    # Columns: [xmin', 'ymin', 'xmax', 'ymax', 'confidence', 'class', 'name']
    df_results: pd.DataFrame = results.pandas().xyxy[0]

    # New detection
    detections2d = [
        {
            "class_id": row["class"],
            "label": row["name"],
            "confidence": row["confidence"],
            "box": (row["xmin"], row["ymin"], row["xmax"], row["ymax"]),
        }
        for (_, row) in df_results.iterrows()
    ]
    depth_image = rospy.wait_for_message(
        DEPTH_IMAGE_TOPIC, Image, timeout=rospy.Duration(10)
    )
    cv_depth_image = CvBridge().imgmsg_to_cv2(depth_image, "passthrough")
    detections3d = detections_2d_to_3d(
        detections2d, cv_image, CAMERA_INTRINSICS, cv_depth_image
    )
    print("Success!")

    # Old detection
    global DETECTED_OBJECTS
    DETECTED_OBJECTS = [
        build_detection(row, image) for (_, row) in df_results.iterrows()
    ]

    return cv_image


def process_frame(image: Image):
    # Rotate image 90 degrees
    cv_image = CvBridge().imgmsg_to_cv2(image, "bgr8")
    cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)

    # Construct rospy image
    image = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")

    cv_image = apply_yolo(image=image)

    # Save image as PNG
    # cv2.imwrite("image.png", cv_image)  # type: ignore

    # Send image to output topic
    global OUTPUT_PUBLISHER
    if OUTPUT_PUBLISHER is not None:
        header = image.header
        header.stamp = rospy.Time.now()
        msg = Detection2DArray(header=header, detections=DETECTED_OBJECTS)
        OUTPUT_PUBLISHER.publish(msg)
    else:
        rospy.logerr("Publisher not created")


def create_point_head_client():
    """
    Creates a point head client
    """
    rospy.loginfo("Creating action client to head controller...")
    client = SimpleActionClient(POINT_HEAD_ACTION, PointHeadAction)

    for _ in range(MAX_ITERATIONS):
        if client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Action client created")
            return client
        else:
            rospy.logwarn("Waiting for action server...")

    rospy.logerr("Action client not created")


def main():
    rospy.init_node("detect_objects")

    # Initialize parameters server
    rospy.loginfo("Initializing parameter server...")

    # Get camera intrinsics
    rospy.loginfo("Getting camera intrinsic parameters...")
    msg = rospy.wait_for_message(
        CAMERA_INFO_TOPIC, CameraInfo, timeout=rospy.Duration(10)
    )  # type: CameraInfo | None
    if msg is not None:
        rospy.loginfo("Camera intrinsic parameters received")
        global CAMERA_INTRINSICS
        CAMERA_INTRINSICS.fromCameraInfo(msg)
        print(CAMERA_INTRINSICS)
    else:
        rospy.logerr("Camera intrinsic parameters not received")

    # Publisher
    rospy.loginfo("Creating publisher...")
    global OUTPUT_PUBLISHER
    OUTPUT_PUBLISHER = rospy.Publisher(OUTPUT_TOPIC, Detection2DArray, queue_size=1)
    if OUTPUT_PUBLISHER is not None:
        rospy.loginfo("Publisher created")
    else:
        rospy.logerr("Publisher not created")

    rospy.Subscriber(CAMERA_TOPIC, Image, process_frame, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
