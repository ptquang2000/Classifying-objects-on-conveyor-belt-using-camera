#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from commander.msg import object_info_msg

class shape_detector():
  def __init__(self, img, pub):
    self.raw_img = img
    self.img = self.resize_frame
    self.pub = pub

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
    msg = object_info_msg()
    msg.shape = 'Unidentify'
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
        msg.shape = 'Triangle'
      elif edge_count == 4:
        msg.shape = 'Triangle'
      elif 12 <= edge_count <= 16:
        msg.shape = 'Circle'
      rospy.loginfo(f'Edge count: {edge_count}')

    cv.imshow('Contours', blank)
    # publish message
    self.pub.publish(msg)

def callback(msg):
  pub = rospy.Publisher('object_info', object_info_msg, queue_size=10)
  bridge = CvBridge()
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    sd = shape_detector(cv_image, pub)
    sd.detecting()
    if cv.waitKey(1) == 27:
      cv.destroyAllWindows()

  except CvBridgeError as e:
    rospy.logfatal(e)

def main():
  rospy.init_node('image_processor')
  rospy.Subscriber('camera/camera_x/image_raw', Image, callback)
  rospy.spin()

if __name__ == "__main__":
  main()
