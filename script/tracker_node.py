#!/usr/bin/env python
import cv2
import sys
import os
import rospy

from std_msgs.msg import UInt16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class face_tracker:
    def __init__(self):
        servo_topic = "/face_tracker/servo_controller"
        image_topic = "/face_tracker/image_raw"

        self.servo_pub = rospy.Publisher(servo_topic, UInt16, queue_size=10)
        self.servo_position = 90

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        haar_file = os.path.join(script_dir, '..', 'data', 'face.xml')
        self.face_cascade = cv2.CascadeClassifier(haar_file)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("image convert error: %s ", e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 255, 0), 2)

        # If any face is found, take first and check it position
        if len(faces) > 0:
            rospy.logdebug("faces detected %s", str(len(faces)))
            rospy.logdebug("first face data: %s", faces[0])

            (rows, cols, channels) = cv_image.shape
            center_img_x = cols / 2

            (x, y, w, h) = faces[0]
            center_face_x = x + w / 2

            delta = center_face_x - center_img_x
            rospy.logdebug("image center %s | face center %s | delta %s",
                           center_img_x, center_face_x, delta)

            threshold = cols * 0.1  # 10 percent from image width
            if abs(delta) > threshold:
                direction = 1 if delta < 0 else -1

                position = self.servo_position
                position += 1 * direction

                if position < 0:
                    position = 0
                elif position > 180:
                    position = 180

                if position != self.servo_position:
                    self.servo_write(position)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(10)

    def servo_write(self, angle):
        if angle < 0 or angle > 180:
            return

        try:
            self.servo_pub.publish(angle)
            self.servo_position = angle
        except CvBridgeError as e:
            rospy.logerr("publish error: %s ", e)


def main(args):
    tracker = face_tracker()
    rospy.init_node('face_tracker', anonymous=True, log_level=rospy.DEBUG)

    # Set servo on 90 degress
    tracker.servo_write(90)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
