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
        self._fileext = '.jpg'
        self._bridge = CvBridge()

    def write_img(self, response):
        cv.imwrite(response.filepath+'_x'+self._fileext, self._img_x)

    def export_img(self, req):
        response = image_capture_srvResponse()
        response.filepath = self._path + (req.filename or str(rospy.get_rostime().secs))
        self.write_img(response)
        return response

    def camera(self, msg):
        try:
            self._img_x = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logfatal(e)


def main():
    rospy.init_node('image_capture')
    capturer = image_capture()
    rospy.Service('image_capture', image_capture_srv, capturer.export_img)
    rospy.Subscriber(f'sensor/camera/image_raw', Image, capturer.camera)
    rospy.spin()


if __name__ == "__main__":
    main()
