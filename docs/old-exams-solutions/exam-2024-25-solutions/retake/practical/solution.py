import cv2
import mujoco
import numpy as np
import os

# In case you want to use the Rotation class from scipy
from scipy.spatial.transform import Rotation as R


def get_global_camera_pose(model, data, camera_name):
    """
    Get the global pose of the camera in the world frame.

    As one can read in MuJoCo documentation,
    https://mujoco.readthedocs.io/en/stable/modeling.html#cameras,
    the cameras in MuJoCo look towards the negative Z axis of the camera frame,
    while positive X and Y correspond to right and up in the image plane, respectively.
    This is different from the standard camera frame in computer vision,
    where the camera looks towards the positive Z axis.
    Hence you might have to add an additional transformation to the camera pose
    before you can use OpenCV to process the images.
    """
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    rotation_matrix = data.cam_xmat[camera_id].reshape(3, 3)
    position = data.cam_xpos[camera_id]
    return position, rotation_matrix


class Detector:
    def __init__(self, model, data) -> None:
        self.im_height = 480  # Image height (pixels)
        self.im_width = 640   # Image width (pixels)

        # TODO: Determine camera parameters and construct camera matrix
        self.fovy = 90
        self.focal_length_y = self.im_height / (2 * np.tan(np.deg2rad(self.fovy) / 2))
        self.focal_length_x = self.focal_length_y  # Assuming square pixels

        self.principal_point_x = self.im_width / 2
        self.principal_point_y = self.im_height / 2

        self.camera_matrix = np.array([
            [self.focal_length_x, 0, self.principal_point_x],
            [0, self.focal_length_y, self.principal_point_y],
            [0, 0, 1]
        ], dtype=float)
        # END TODO

        self.dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
        self.model = model
        self.data = data

    def detect(self, img, save_dir = 'imgs', file_name = None) -> None:
        # Load the dictionary that was used to generate the markers.
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        # TODO: Detect the cube and find its position
        detectorParams = cv2.aruco.DetectorParameters()
        detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        detector = cv2.aruco.ArucoDetector(aruco_dict, detectorParams)

        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect the markers in the image
        corners, ids, _ = detector.detectMarkers(gray)

        detected = ids is not None
        cube_center_world_frame = None

        if ids is not None:
            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.2 - 0.04, self.camera_matrix, self.dist_coeffs)

            # Use the first detected marker to determine the distance
            rvec = rvecs[0]
            tvec = tvecs[0]

            # Convert the rotation vector to a rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Define the point [0, 0, 1] in the marker's coordinate frame
            cube_center_marker_frame = np.array([0, 0, -0.1]).reshape((3, 1))
            cube_center_camera_frame = np.dot(rotation_matrix, cube_center_marker_frame) + tvec.reshape((3, 1))

            # Get the camera rotation matrix
            camera_name = "camera"
            camera_position, camera_rotation_matrix = get_global_camera_pose(self.model, self.data, camera_name)
            flip_view = R.from_euler('xyz', [180, 0, 0], degrees=True).as_matrix()
            combined_rotation_matrix = np.dot(camera_rotation_matrix, flip_view)
            cube_center_world_frame = np.dot(combined_rotation_matrix, cube_center_camera_frame.reshape(3,)) + camera_position
        # END TODO

        self.test_detection(detected, cube_center_world_frame)

        if file_name is not None:
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            cv2.imwrite(f"{save_dir}/{file_name}.png", img)

    def test_detection(self, detected, estimated_pos = None):
        if detected:
            print("Cube detected. Estimated position:", estimated_pos)
            body_id = self.model.body('body').id
            body_position = self.data.xpos[body_id]

            error = np.linalg.norm(np.array(body_position) - np.array(estimated_pos))
            assert error < 0.05
