import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')

        # Create a subscriber to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            'sensor/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a publisher for the compressed image topic
        self.publisher_ = self.create_publisher(
            CompressedImage,
            'sensor/image_compressed',
            10)

        # Create a CvBridge object
        self.br = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Compress the image using JPEG
        success, compressed_image = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

        if success:
            # Create a CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = compressed_image.tobytes()

            # Publish the compressed image
            self.publisher_.publish(compressed_msg)
        else:
            self.get_logger().error("Failed to compress the image")

def main(args=None):
    rclpy.init(args=args)

    image_compressor = ImageCompressor()

    rclpy.spin(image_compressor)

    image_compressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
