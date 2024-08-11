import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

def is_cone_shape(contour):
    epsilon = 0.04 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    if len(approx) == 3:
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        area = cv2.contourArea(approx)
        if 0.5 < aspect_ratio < 1.5 and area > 1000:
            return True
    return False
cap = cv2.VideoCapture(0)

lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])

lower_blue = np.array([110, 100, 100])
upper_blue = np.array([130, 255, 255])
while True:
    ret, frame = cap.read()
    if not ret:
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = mask1 + mask2
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours_red:
        if is_cone_shape(contour):
            cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)  # Red contours
    for contour in contours_blue:
        if is_cone_shape(contour):
            cv2.drawContours(frame, [contour], -1, (255, 0, 0), 2)  # Blue contours

    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


class Object_count(Node):
    def __init__(self):
        super().__init__('object_count')
        self.sub = self.create_subscription(Image,'/camera/image',self.image_callback,10)
    

    def image_callback(self,msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imshow('image',cv_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main(args = None):
    rclpy.init(args=args)
    image_sub = Object_count()
    rclpy.spin(image_sub)
    image_sub.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
