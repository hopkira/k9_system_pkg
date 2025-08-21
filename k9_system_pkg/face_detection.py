import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mediapipe as mp
import threading
import time
from picamera2 import Picamera2, Preview

from k9_interfaces_pkg.msg import FaceWithLandmarks, BoundingBox, FaceLandmark

# Optimized landmark indices
OPTIMIZED_LANDMARKS = [
    33, 133, 159, 145,
    263, 362, 386, 374,
    70, 105, 107,
    336, 334, 300,
    1,
    61, 291,
    152
]

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')

        # Parameters
        self.declare_parameter("frame_rate", 15.0)  # higher frame rate
        self.declare_parameter("camera_width", 640)  # smaller resolution for speed
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("max_num_faces", 1)
        self.declare_parameter("min_detection_confidence", 0.5)
        self.declare_parameter("min_tracking_confidence", 0.5)

        self.frame_rate = self.get_parameter("frame_rate").get_parameter_value().double_value
        self.width = self.get_parameter("camera_width").get_parameter_value().integer_value
        self.height = self.get_parameter("camera_height").get_parameter_value().integer_value
        self.max_num_faces = self.get_parameter("max_num_faces").get_parameter_value().integer_value
        self.min_detection_conf = self.get_parameter("min_detection_confidence").get_parameter_value().double_value
        self.min_tracking_conf = self.get_parameter("min_tracking_confidence").get_parameter_value().double_value

        # Publisher
        self.publisher_ = self.create_publisher(FaceWithLandmarks, 'face_with_landmarks', 10)
        self.bridge = CvBridge()

        # Picamera2 setup in video mode (faster and async-friendly)
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.picam2.configure(config)
        try:
            self.picam2.start()
        except Exception as e:
            self.get_logger().error(f"Failed to start Picamera2: {e}")
            rclpy.shutdown()
            return

        # MediaPipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=self.max_num_faces,
            refine_landmarks=True,
            min_detection_confidence=self.min_detection_conf,
            min_tracking_confidence=self.min_tracking_conf
        )

        # Thread safety
        self._detection_lock = threading.Lock()
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.start_detection_thread)

        self.last_frame_fail_log = 0.0
        self.log_interval = 30.0  # warn less frequently

        self.get_logger().info(f"FaceDetectionNode started at {self.width}x{self.height} @ {self.frame_rate}Hz")

    def start_detection_thread(self):
        if self._detection_lock.locked():
            return
        threading.Thread(target=self.detect_faces, daemon=True).start()

    def detect_faces(self):
        with self._detection_lock:
            try:
                frame = self.picam2.capture_array()
            except Exception:
                now = time.time()
                if now - self.last_frame_fail_log > self.log_interval:
                    self.get_logger().warn("Failed to capture frame from Picamera2")
                    self.last_frame_fail_log = now
                return

            # Direct RGB input to MediaPipe
            results = self.face_mesh.process(frame)

            if not results.multi_face_landmarks:
                return

            for face_landmarks in results.multi_face_landmarks:
                face_msg = FaceWithLandmarks()
                face_msg.header.stamp = self.get_clock().now().to_msg()

                xs = [face_landmarks.landmark[i].x for i in OPTIMIZED_LANDMARKS]
                ys = [face_landmarks.landmark[i].y for i in OPTIMIZED_LANDMARKS]
                x_min = int(max(0, min(xs) * self.width))
                y_min = int(max(0, min(ys) * self.height))
                x_max = int(min(self.width, max(xs) * self.width))
                y_max = int(min(self.height, max(ys) * self.height))

                face_msg.bbox = BoundingBox(x=x_min, y=y_min, width=x_max - x_min, height=y_max - y_min)

                # Crop face image
                face_crop = frame[y_min:y_max, x_min:x_max]
                face_msg.image = self.bridge.cv2_to_imgmsg(face_crop, encoding='rgb8')

                for idx in OPTIMIZED_LANDMARKS:
                    lm = face_landmarks.landmark[idx]
                    landmark_msg = FaceLandmark(x=lm.x, y=lm.y, z=lm.z)
                    face_msg.landmarks.append(landmark_msg)

                self.publisher_.publish(face_msg)

    def destroy_node(self):
        if self.timer:
            self.timer.cancel()
        if self.picam2:
            self.picam2.stop()
        self.face_mesh.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()