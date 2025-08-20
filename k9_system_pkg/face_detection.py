import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mediapipe as mp
import cv2
import threading

from your_package.msg import FaceWithLandmarks, BoundingBox, FaceLandmark  # Replace with actual package

# Optimized landmark indices
OPTIMIZED_LANDMARKS = [
    33, 133, 159, 145,   # Left eye
    263, 362, 386, 374,  # Right eye
    70, 105, 107,         # Left eyebrow
    336, 334, 300,        # Right eyebrow
    1,                     # Nose tip
    61, 291,               # Mouth corners
    152                    # Chin
]

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')

        # Parameters
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_rate", 10.0)
        self.declare_parameter("max_num_faces", 1)
        self.declare_parameter("min_detection_confidence", 0.5)
        self.declare_parameter("min_tracking_confidence", 0.5)

        self.camera_index = self.get_parameter("camera_index").get_parameter_value().integer_value
        self.frame_rate = self.get_parameter("frame_rate").get_parameter_value().double_value
        self.max_num_faces = self.get_parameter("max_num_faces").get_parameter_value().integer_value
        self.min_detection_conf = self.get_parameter("min_detection_confidence").get_parameter_value().double_value
        self.min_tracking_conf = self.get_parameter("min_tracking_confidence").get_parameter_value().double_value

        self.publisher_ = self.create_publisher(FaceWithLandmarks, 'face_with_landmarks', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera at index {self.camera_index}")
            rclpy.shutdown()
            return

        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.start_detection_thread)
        self._detection_lock = threading.Lock()

        # MediaPipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=self.max_num_faces,
            refine_landmarks=True,
            min_detection_confidence=self.min_detection_conf,
            min_tracking_confidence=self.min_tracking_conf
        )

        self.get_logger().info(
            f"FaceDetectionNode started with camera_index={self.camera_index}, "
            f"frame_rate={self.frame_rate} Hz, max_num_faces={self.max_num_faces}, "
            f"min_detection_confidence={self.min_detection_conf}, min_tracking_confidence={self.min_tracking_conf}"
        )

    def start_detection_thread(self):
        if self._detection_lock.locked():
            return
        threading.Thread(target=self.detect_faces, daemon=True).start()

    def detect_faces(self):
        with self._detection_lock:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to read from camera")
                return

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(frame_rgb)

            if not results.multi_face_landmarks:
                return

            height, width, _ = frame.shape

            for face_landmarks in results.multi_face_landmarks:
                face_msg = FaceWithLandmarks()
                face_msg.header.stamp = self.get_clock().now().to_msg()

                # Compute bounding box
                xs = [face_landmarks.landmark[i].x for i in OPTIMIZED_LANDMARKS]
                ys = [face_landmarks.landmark[i].y for i in OPTIMIZED_LANDMARKS]
                x_min = int(max(0, min(xs) * width))
                y_min = int(max(0, min(ys) * height))
                x_max = int(min(width, max(xs) * width))
                y_max = int(min(height, max(ys) * height))

                face_msg.bbox = BoundingBox(x=x_min, y=y_min, width=x_max - x_min, height=y_max - y_min)

                # Crop face image
                face_crop = frame[y_min:y_max, x_min:x_max]
                face_msg.image = self.bridge.cv2_to_imgmsg(face_crop, encoding='bgr8')

                # Add optimized landmarks
                for idx in OPTIMIZED_LANDMARKS:
                    lm = face_landmarks.landmark[idx]
                    landmark_msg = FaceLandmark(x=lm.x, y=lm.y, z=lm.z)
                    face_msg.landmarks.append(landmark_msg)

                self.publisher_.publish(face_msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        if self.timer:
            self.timer.cancel()
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