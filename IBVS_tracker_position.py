import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class IBVSTrackerNode(Node):
    def __init__(self):
        super().__init__('ibvs_tracker_node')
        
        # 1. PHYSICAL PARAMETERS
        # These must match your Gazebo model exactly for the math to work.
        self.MARKER_LENGTH = 0.081      # Size of one ArUco square (meters)
        self.MARKER_SEPARATION = 0.0135 # Gap between squares (meters)
        self.TARGET_HOVER_DEPTH = 3.0   # How far we want to be from the board (Z-axis)
        self.LAMBDA_GAIN = 0.5          # Speed multiplier (higher = more aggressive)
        
        # 2. CAMERA INTRINSICS
        # Tells the script the focal length and center point of the camera lens.
        self.mtx = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        self.dist = np.zeros(5, dtype=np.float32)
        self.fx, self.fy, self.cx, self.cy = 600.0, 600.0, 320.0, 240.0

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # 3. POINT MAPPING
        # We define where each marker sits in 3D space relative to the board center.
        s = self.MARKER_LENGTH
        g = self.MARKER_SEPARATION
        self.marker_obj_points = {
            0: np.array([[0, s, 0], [s, s, 0], [s, 0, 0], [0, 0, 0]], dtype=np.float32),
            1: np.array([[s+g, s, 0], [2*s+g, s, 0], [2*s+g, 0, 0], [s+g, 0, 0]], dtype=np.float32),
            # ... (Points for markers 2 and 3 continue here)
        }

        # 4. TARGET FEATURES (s_star)
        # This is the "Goal." It calculates where pixels SHOULD be if we were at 3.0m.
        total_side = (2 * s) + g
        h = total_side / 2.0
        Z_star = self.TARGET_HOVER_DEPTH
        self.s_star = np.array([
            [-h/Z_star], [h/Z_star], [h/Z_star], [h/Z_star],
            [h/Z_star], [-h/Z_star], [-h/Z_star], [-h/Z_star]
        ], dtype=np.float32)

        # Subscriber: Listens to the camera topic
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)

    def get_interaction_matrix(self, x, y, Z):
        """
        The 'Image Jacobian'. It maps 3D camera velocity to 2D pixel motion.
        It answers: 'If I move the drone 1m/s left, how fast will the pixels move?'
        """
        Z = max(Z, 0.1) # Avoid division by zero
        return np.array([
            [-1.0/Z,  0,       x/Z,  x*y,        -(1.0+x**2),  y ],
            [ 0,      -1.0/Z,  y/Z,  1.0+y**2,   -x*y,        -x ]
        ], dtype=np.float32)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Step A: Detect the markers in the current frame
            corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict)

            if ids is not None:
                # Step B: Pose Estimation
                # We calculate the distance (Z) and orientation of the board.
                success, rvec, tvec = self.calculate_pose(corners, ids)
                
                if success:
                    Z = float(tvec[2][0]) # Current estimated depth
                    
                    # Step C: Error Calculation
                    # Find where the board center corners are in pixels RIGHT NOW.
                    s_current = self.get_normalized_pixel_features(rvec, tvec)
                    
                    # Step D: The Control Law
                    # Velocity = -Gain * (Inverse Matrix) * (Goal - Current)
                    error = s_current - self.s_star
                    L_mat = self.build_full_interaction_matrix(s_current, Z)
                    v_cmd = -self.LAMBDA_GAIN * (np.linalg.pinv(L_mat) @ error)

                    # Result: vx, vy are lateral drift, vz is forward/backward distance
                    self.get_logger().info(f"Dist: {Z:.2f}m | CMD: {v_cmd[0,0]:.2f}, {v_cmd[1,0]:.2f}, {v_cmd[2,0]:.2f}")

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')