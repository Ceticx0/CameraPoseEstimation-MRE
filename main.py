import apriltag
import cv2
import numpy as np
from coordinate_systems import openCVPoseToWPILib
from wpimath.geometry import *

cam = cv2.VideoCapture("exampleVideo.mkv")
cam_matrix = np.array([[940.7360710926395, 0, 615.5884770322365], [0, 939.9932393907364, 328.53938300868], [0, 0, 1]])
dist_coeffs = np.array([[0.054834081023049625, -0.15994111706817074, -0.0017587106009926158, -0.0014671022483263552, 0.049742166267499596]])

options = apriltag.DetectorOptions(
            quad_decimate=1.0
        )
detector = apriltag.Detector(options=options)

tag_size = 0.1651  # 6.5 in to meter
# tag_size = 0.17145 # 6.75 in, this is what polaris uses and I have no idea why 😭😭

# for id 6, the only one shown in the video
tag_pose = Pose3d(Translation3d(1.841500, 8.204200, 1.355852), Rotation3d(0.000000, 0.000000, -1.570796))

while True:
    result, image = cam.read()
    image_grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # -- detect tags --
    detections = detector.detect(image_grayscale)

    # -- run solvepnp for tvec and rvec --
    if not detections:
        continue

    # assuming only 1 apriltag for like everything
    object_points = np.array([[-tag_size / 2.0, tag_size / 2.0, 0.0],
                              [tag_size / 2.0, tag_size / 2.0, 0.0],
                              [tag_size / 2.0, -tag_size / 2.0, 0.0],
                              [-tag_size / 2.0, -tag_size / 2.0, 0.0]])
    image_points = [
        [detections[0].corners[0][0], detections[0].corners[0][1]],
        [detections[0].corners[1][0], detections[0].corners[1][1]],
        [detections[0].corners[2][0], detections[0].corners[2][1]],
        [detections[0].corners[3][0], detections[0].corners[3][1]]
    ]
    _, rvecs, tvecs, errors = cv2.solvePnPGeneric(object_points, np.array(image_points),
                                                  cam_matrix,
                                                  dist_coeffs,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    cv2.drawFrameAxes(image, cam_matrix,
                      dist_coeffs,
                      rvecs[0], tvecs[0], 0.2)

    # -- convert to wpilib field coords --
    camera_to_tag_pose_0 = openCVPoseToWPILib(tvecs[0], rvecs[0])
    camera_to_tag_0 = Transform3d(camera_to_tag_pose_0.translation(), camera_to_tag_pose_0.rotation())
    field_to_camera_0 = tag_pose.transformBy(camera_to_tag_0.inverse())
    field_to_camera_pose_0 = Pose3d(field_to_camera_0.translation(), field_to_camera_0.rotation())
    print(field_to_camera_pose_0)

    cv2.imshow("", image)
    key = cv2.waitKey(100)
    if key == 13:
        break
