import numpy as np
import quaternion
import cv2

def to_homogeneous(points):
    return np.concatenate(
        [points, np.ones((points.shape[0], 1), dtype=points.dtype)], axis=-1)

def from_homogeneous(points):
    return points[:, :-1] / points[:, -1:]

def angle_error(R1, R2):
    cos = (np.trace(np.dot(np.linalg.inv(R1), R2)) - 1) / 2
    return np.rad2deg(np.abs(np.arccos(cos)))

def project_leds(points3d, blobs, pos, orient, camera_matrix, distK):
    matches = list(map(lambda x: x['led'], blobs))

    K = np.array(camera_matrix).reshape((3,3))
    D = np.array(distK)

    # print("camera_matrix {}".format(K))
    # print("distCoeffs {}".format(D))

    points3d = np.array(points3d)

    Rin = quaternion.as_rotation_vector(orient)
    pos = np.array(pos).reshape(3)

    tmp_points3d = np.array(list(map(lambda x: x['pos'], points3d))).reshape((-1, 1, 3)) # all points
    all_outpoints, _ = cv2.fisheye.projectPoints(tmp_points3d, tvec=pos, rvec=Rin , K=K, D=D)

    outpoints = all_outpoints[matches[:]]

    return all_outpoints, outpoints

def undistort_points(imgpoints, camera_matrix, distK, output_camera_matrix=False):
    # Apply fisheye undistortion
    K = np.array(camera_matrix).reshape((3,3))
    D = np.array(distK)
    imgpoints = np.array(imgpoints).astype(np.float32).reshape((-1, 1, 2))

    if output_camera_matrix:
        P = K
    else:
        P = None

    return cv2.fisheye.undistortPoints(imgpoints, K=K, D=D, P=P)

def refine_pose(points3d, blobs, inpos, orient, camera_matrix, distK, reproj_thresh, verbose=False):
    failure = (None, None, None, None)

    # np.random.shuffle(blobs)
    matches = list(map(lambda x: x['led'], blobs))

    # print("blobs {}", blobs)
    # print("matches {}", matches)

    if len(matches) < 4:
        print("Not enough matches. Got {}", matches)
        return failure

    K = np.array(camera_matrix).reshape((3,3))
    D = np.array(distK)

    # print("camera_matrix {}".format(K))
    # print("distCoeffs {}".format(D))

    imgpoints_in = np.array(list(map(lambda x: (x['x'], x['y']), blobs))).astype(np.float32).reshape((-1, 1, 2))

    # Apply fisheye undistortion
    imgpoints = cv2.fisheye.undistortPoints(imgpoints_in, K=K, D=D, P=K)

    # Extract 3D LED points for the matched LEDs
    points3d = np.array(points3d)
    points3d = np.array(list(map(lambda x: x['pos'], points3d[matches[:]]))).reshape((-1, 1, 3))

    if orient is not None:
        Rin = quaternion.as_rotation_vector(orient)
        # perturn the input for testing
        Rin += np.array([0.1, 0.1, 0.1])
    else:
        Rin = None

    if inpos is not None:
        pos = np.array(inpos, copy=True).reshape(3)
        inpos = np.array(inpos).reshape(3)
        # perturb the input for testing
        pos += np.array([0.1, 0.1, 0.1])
    else:
        pos = None

    if True:
        success, R_vec, t, inliers = cv2.solvePnPRansac(
            points3d, imgpoints, cameraMatrix=K, distCoeffs=np.zeros(4),
            flags=cv2.SOLVEPNP_EPNP,
            # flags=cv2.USAC_ACCURATE,
            tvec=pos, rvec=Rin,
            iterationsCount=50, confidence=0.995, reprojectionError=reproj_thresh)

        if success:
            # print("PnP solution R {} T {}".format(R_vec, t))
            if len(inliers) != len(imgpoints):
                imgpoints = imgpoints[inliers[:]].reshape((-1, 1, 2))
                points3d = points3d[inliers[:]].reshape((-1, 1, 3))
            if verbose:
                print("Inliers\n{}\nimgpoints\n{}\nundistorted\n{}\npoints3d\n{}".format(inliers.reshape(1,-1), imgpoints,
                    undistort_points(imgpoints_in, camera_matrix, distK), points3d))

            # If Z is behind the camera, the solution is invalid, otherwise
            # transfer the inputs for the LM refine to run on
            if t[2] > 0:
                Rin, pos = R_vec, t
        else:
            return failure
    else:
        R_vec, t = None, None
    
    if True:
        if verbose:
            print("Running LM refine. Rin {} pos {}".format(Rin, pos))

        # print("points3d {}".format(points3d))
        # print("imgpoints {}".format(imgpoints))
        criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 0.0001)
        R_vec, t = cv2.solvePnPRefineLM(points3d, imgpoints, cameraMatrix=K, distCoeffs=np.zeros(4), rvec=Rin, tvec=pos, criteria=criteria)

        # R_vec, t = cv2.solvePnPRefineVVS(points3d, imgpoints, cameraMatrix=K, distCoeffs=np.zeros(4), rvec=Rin, tvec=pos, criteria=criteria)

    R, _ = cv2.Rodrigues(R_vec)
    t = t.reshape((3))

    # print ("t {} pos {} dt {}".format(t, inpos, t - inpos[:]))
    if inpos is not None:
        error_t = np.linalg.norm(t - inpos[:])
    else:
        error_t = None

    if orient is not None:
        error_R = angle_error(R, quaternion.as_rotation_matrix(orient))
    else:
        error_R = None

    return t, R, error_t, error_R
