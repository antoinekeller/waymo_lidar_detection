import numpy as np

def project(point_in_world, to_camera_from_world, K):
    """
    Given the camera intrinsic and extrinsic calibration matrices,
    projet a 3D point in world coordinates to the image
    """

    point_in_cam = to_camera_from_world[:3, :3].dot(point_in_world.T) + to_camera_from_world[:3, 3]

    if point_in_cam[0] < 0:
        return np.array([-1, -1])
    
    point_in_cam /= -point_in_cam[0]

    proj = K.dot(np.array([point_in_cam[1], point_in_cam[2], 1]))

    return proj[:2]