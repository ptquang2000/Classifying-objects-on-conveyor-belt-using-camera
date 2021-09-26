#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

class shape_detector():
  def __init__(self, img):
    self.raw_img = img
    self.img = self.resize_frame

  @property
  def resize_frame(self, scale=0.25):
    width = int(self.raw_img.shape[1] * scale)
    height = int(self.raw_img.shape[0] * scale)
    dimemsion = (width, height)
    resized = cv.resize(self.raw_img, dimemsion, interpolation=cv.INTER_AREA)
    return resized

  @property
  def pre_processing(self):
    # convert to gray scale
    src = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)
    # filter background color
    src[np.where(
      (src == [178])
    )] = [0]
    # blur image
    src = cv.GaussianBlur(src, (3,3), 0, 0)
    cv.imshow('Blur', src)
    src = cv.Canny(src, threshold1=100, threshold2=200)
    cv.imshow('Canny', src)
    return src
  
  def detecting(self):
    contours, _ = cv.findContours(self.pre_processing, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    blank = np.zeros(self.img.shape, dtype='uint8')
    
    for contour in contours:
      area = cv.contourArea(contour)
      if area < 500:
        continue
      approx = cv.approxPolyDP(contour, 0.01 * cv.arcLength(contour, True), True)
      cv.drawContours(blank, contours, -1, (255, 0, 0), 2)
      cv.drawContours(blank, [approx], -1, (0, 255, 0), 2)
      
      if (edge_count:=len(approx)) == 3:
        rospy.loginfo('Object shape: Triangle')
      elif edge_count == 4:
        rospy.loginfo('Object shape: Retangle')
      elif 13 <= edge_count <= 15:
        rospy.loginfo('Object shape: Circle')
      else:
        rospy.loginfo('Unidentify')

      rospy.loginfo(f'Edge count: {edge_count}')

    cv.imshow('Contours', blank)

def callback(msg):
  bridge = CvBridge()
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    sd = shape_detector(cv_image)
    sd.detecting()
    if cv.waitKey(1) == 27:
      cv.destroyAllWindows()

  except CvBridgeError as e:
    rospy.logfatal(e)

def main():
  rospy.init_node('image_processor')
  rospy.Subscriber('camera/camera_x/image_raw', Image, callback)
  rate = rospy.Rate(0.2)
  rate.sleep()
  rospy.spin()

if __name__ == "__main__":
  main()
