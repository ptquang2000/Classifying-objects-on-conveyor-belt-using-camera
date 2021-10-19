#!/usr/bin/env python
import rospy
from rospkg import RosPack
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from commander.srv import image_capture_srv, image_capture_srvResponse


class image_capture():
    def __init__(self):
        self._path = RosPack().get_path('commander')+'/images/'
        self._img_x = ''
        self._img_y = ''
        self._img_z = ''
        self._fileext = '.jpg'
        self._bridge = CvBridge()

    def write_img(self, response):
        cv.imwrite(response.filepath+'_x'+self._fileext, self._img_x)
        cv.imwrite(response.filepath+'_y'+self._fileext, self._img_y)
        cv.imwrite(response.filepath+'_z'+self._fileext, self._img_z)

    def export_img(self, req):
        response = image_capture_srvResponse()
        response.filepath = self._path + \
            (req.filename or str(rospy.get_rostime().secs))
        self.write_img(response)
        return response

    def camera_x(self, msg):
        try:
            self._img_x = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logfatal(e)

    def camera_y(self, msg):
        try:
            self._img_y = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logfatal(e)

    def camera_z(self, msg):
        try:
            self._img_z = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logfatal(e)


def main():
    rospy.init_node('image_capture')
    capturer = image_capture()
    rospy.Service('image_capture', image_capture_srv, capturer.export_img)
    rospy.Subscriber(f'camera/camera_x/image_raw', Image, capturer.camera_x)
    rospy.Subscriber(f'camera/camera_y/image_raw', Image, capturer.camera_y)
    rospy.Subscriber(f'camera/camera_z/image_raw', Image, capturer.camera_z)
    rospy.spin()


if __name__ == "__main__":
    main()
