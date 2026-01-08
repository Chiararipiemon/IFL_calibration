#!/usr/bin/env python3
import os
import numpy as np
import rospy
import tf2_ros
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf.transformations import quaternion_matrix


def tfmsg_to_T(transform_stamped):
    q = transform_stamped.transform.rotation
    t = transform_stamped.transform.translation
    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z
    return T


def invert_T(T):
    R = T[:3, :3]
    p = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


class HandEyeCapture:
    def __init__(self):
        self.bridge = CvBridge()

        # Frames
        self.base_frame = rospy.get_param("~base_frame", "iiwa_link_0")
        self.ee_frame   = rospy.get_param("~ee_frame",   "iiwa_link_ee")

        # Topics
        self.image_topic = rospy.get_param("~image_topic", "/rgb/image_raw")
        self.info_topic  = rospy.get_param("~info_topic",  "/rgb/camera_info")

        # ArUco params
        self.marker_length = float(rospy.get_param("~marker_length_m", 0.04))  # meters
        self.marker_id     = int(rospy.get_param("~marker_id", 582))
        dict_name          = rospy.get_param("~aruco_dict", "DICT_ARUCO_ORIGINAL")

        # Output
        self.out_file = rospy.get_param("~out_file", os.path.expanduser("~/handeye_samples.npz"))

        # Intrinsics
        self.K = None
        self.D = None

        # TF
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # ArUco init
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco not available. Install opencv-contrib-python.")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if not hasattr(cv2.aruco, "ArucoDetector"):
            raise RuntimeError("OpenCV build missing ArucoDetector (unexpected for 4.12).")

        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.samples = []

        rospy.Subscriber(self.info_topic, CameraInfo, self.cb_info, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1)

        rospy.loginfo("OpenCV version: %s", cv2.__version__)
        rospy.loginfo("HandEyeCapture ready. Press 's' to save, 'q' to quit.")
        rospy.loginfo("Saving to: %s", self.out_file)

    def cb_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.D, dtype=np.float64).reshape(1, -1)  # rational_polynomial -> len 8
            fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
            rospy.loginfo("Got intrinsics: frame_id=%s size=%dx%d", msg.header.frame_id, msg.width, msg.height)
            rospy.loginfo("fx=%.3f fy=%.3f cx=%.3f cy=%.3f | D_len=%d",
                          fx, fy, cx, cy, self.D.shape[1])

    def estimate_marker_pose_pnp(self, img_corners_4x2):
        """
        Compute rvec/tvec for marker->camera using solvePnP.
        Corners ordering from ArUco is: top-left, top-right, bottom-right, bottom-left.
        We define marker frame with origin at center, Z out of the marker plane.
        """
        L = float(self.marker_length)
        half = L / 2.0

        obj_pts = np.array([
            [-half,  half, 0.0],  # top-left
            [ half,  half, 0.0],  # top-right
            [ half, -half, 0.0],  # bottom-right
            [-half, -half, 0.0],  # bottom-left
        ], dtype=np.float32)

        img_pts = np.array(img_corners_4x2, dtype=np.float32).reshape(4, 2)

        # Best flag for squares if available
        flag = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE

        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.D, flags=flag)
        return ok, rvec, tvec

    def cb_image(self, msg: Image):
        if self.K is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _rej = self.aruco_detector.detectMarkers(gray)

        detected = False
        rvec = None
        tvec = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().astype(int)
            rospy.loginfo_throttle(1.0, f"Detected ArUco IDs: {ids_flat.tolist()}")

            if self.marker_id in ids_flat:
                idx = int(np.where(ids_flat == self.marker_id)[0][0])

                # corners[idx] shape is (1, 4, 2) typically
                c = corners[idx].reshape(4, 2)

                ok, rvec, tvec = self.estimate_marker_pose_pnp(c)
                if ok:
                    detected = True
                    cv2.aruco.drawDetectedMarkers(img, corners, ids)
                    cv2.drawFrameAxes(img, self.K, self.D, rvec, tvec, self.marker_length * 0.75, 2)
                else:
                    cv2.aruco.drawDetectedMarkers(img, corners, ids)

            else:
                cv2.aruco.drawDetectedMarkers(img, corners, ids)

        cv2.putText(img, f"samples: {len(self.samples)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(img, f"marker_id: {self.marker_id} detected: {detected}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("handeye_capture", img)
        key = cv2.waitKey(1) & 0xFF

        if key != 255:
            rospy.loginfo_throttle(1.0, f"Key pressed: {key} ({chr(key) if 32 <= key <= 126 else ''})")

        if key == ord('q'):
            self.save_npz()
            rospy.signal_shutdown("Quit")
            return

        if key == ord('s'):
            if not detected:
                rospy.logwarn("Marker not detected / pose not estimated. Not saving.")
                return

            stamp = msg.header.stamp

            # TF at image time: ^B T_E
            try:
                tf_be = self.tf_buf.lookup_transform(
                    self.base_frame, self.ee_frame,
                    stamp, rospy.Duration(0.5)
                )
            except Exception as e:
                rospy.logwarn("TF at image stamp failed (%s). Falling back to latest TF.", str(e))
                try:
                    tf_be = self.tf_buf.lookup_transform(
                        self.base_frame, self.ee_frame,
                        rospy.Time(0), rospy.Duration(0.5)
                    )
                except Exception as e2:
                    rospy.logwarn("TF latest also failed: %s", str(e2))
                    return

            T_BE = tfmsg_to_T(tf_be)
            T_EB = invert_T(T_BE)  # OpenCV wants gripper2base = ^E T_B

            # Marker->camera: ^C T_M from rvec/tvec
            R_CM, _ = cv2.Rodrigues(rvec)
            t_CM = tvec.reshape(3, 1)

            sample = {
                "R_gripper2base": T_EB[:3, :3],
                "t_gripper2base": T_EB[:3, 3].reshape(3, 1),
                "R_target2cam":   R_CM,
                "t_target2cam":   t_CM,
                "stamp":          float(stamp.to_sec())
            }
            self.samples.append(sample)
            rospy.loginfo("Saved sample %d at t=%.6f", len(self.samples), sample["stamp"])
            self.save_npz()

    def save_npz(self):
        if len(self.samples) == 0:
            rospy.logwarn("No samples to save.")
            return

        Rs_g2b = np.stack([s["R_gripper2base"] for s in self.samples], axis=0)
        ts_g2b = np.stack([s["t_gripper2base"] for s in self.samples], axis=0)
        Rs_t2c = np.stack([s["R_target2cam"]   for s in self.samples], axis=0)
        ts_t2c = np.stack([s["t_target2cam"]   for s in self.samples], axis=0)
        stamps = np.array([s["stamp"] for s in self.samples], dtype=np.float64)

        np.savez(self.out_file,
                 R_gripper2base=Rs_g2b, t_gripper2base=ts_g2b,
                 R_target2cam=Rs_t2c,   t_target2cam=ts_t2c,
                 stamp=stamps)
        rospy.loginfo("Wrote %d samples to %s", len(self.samples), self.out_file)


if __name__ == "__main__":
    rospy.init_node("handeye_capture")
    HandEyeCapture()
    rospy.spin()
