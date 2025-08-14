import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.publisher_ = self.create_publisher(Image, 'face_images', 10)
        self.bridge = CvBridge()

        # Load Haar Cascade
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        # Initialize camera
        self.cap = cv2.VideoCapture(0)

        # Timer to process frames
        self.timer = self.create_timer(1.0, self.detect_faces)

    def detect_faces(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read from camera')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

        for (x, y, w, h) in faces:
            face_img = frame[y:y+h, x:x+w]
            face_msg = self.bridge.cv2_to_imgmsg(face_img, encoding='bgr8')
            self.publisher_.publish(face_msg)
            self.get_logger().info('Published face image')

    def destroy_node(self):
        self.cap.release()
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
