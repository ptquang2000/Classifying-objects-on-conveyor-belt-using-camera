#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from commander.msg import object_info_msg

COLOR = {0: 'Red', 1: 'Green', 2: 'Blue', 3: 'Yellow', 4: 'Orange', 5: 'Purple'}

def hsv_finder(img):
  cv.namedWindow('image')
  # Create trackbars for color change
  # Hue is from 0-179 for Opencv
  cv.createTrackbar('HMin', 'image', 0, 255, lambda x: None)
  cv.createTrackbar('SMin', 'image', 0, 255, lambda x: None)
  cv.createTrackbar('VMin', 'image', 0, 255, lambda x: None)
  cv.createTrackbar('HMax', 'image', 0, 255, lambda x: None)
  cv.createTrackbar('SMax', 'image', 0, 255, lambda x: None)
  cv.createTrackbar('VMax', 'image', 0, 255, lambda x: None)

  # Set default value for Max HSV trackbars
  cv.setTrackbarPos('HMax', 'image', 255)
  cv.setTrackbarPos('SMax', 'image', 255)
  cv.setTrackbarPos('VMax', 'image', 255)

  # Initialize HSV min/max values
  hMin = sMin = vMin = hMax = sMax = vMax = 0
  phMin = psMin = pvMin = phMax = psMax = pvMax = 0

  while(1):
    # Get current positions of all trackbars
    hMin = cv.getTrackbarPos('HMin', 'image')
    sMin = cv.getTrackbarPos('SMin', 'image')
    vMin = cv.getTrackbarPos('VMin', 'image')
    hMax = cv.getTrackbarPos('HMax', 'image')
    sMax = cv.getTrackbarPos('SMax', 'image')
    vMax = cv.getTrackbarPos('VMax', 'image')

    # Set minimum and maximum HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Convert to HSV format and color threshold
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    result = cv.bitwise_and(img, img, mask=mask)

    # Print if there is a change in HSV value
    if((phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
      print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
      phMin = hMin
      psMin = sMin
      pvMin = vMin
      phMax = hMax
      psMax = sMax
      pvMax = vMax
    cv.imshow('image', result)
    if cv.waitKey(10) & 0xFF == ord('q'):
      break

class color_detector():
  def __init__(self, img, colors, pub):
    self.raw_img = img
    self.img = self.resize_frame
    self.colors = colors
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
    # convert to hsv
    src = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
    # filter background color
#    src[np.where(
#      (src == [178])
#    )] = [0]
    masks = [cv.inRange(src, color[0], color[1]) for color in self.colors]
#    return [cv.bitwise_and(self.img, self.img, mask=mask) for mask in masks]
    return masks
  
  def detecting(self):
    msg = object_info_msg()
    msg.color = 'Plain'

    masks = self.pre_processing
    contours = [
        cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)[0]
        for mask in masks
        ]
    areas = [
        max([cv.contourArea(cnt) for cnt in cnts] or [0])
        for cnts in contours
        ]
    #rospy.loginfo(areas)
    f_area = [area for area in areas if area != 0]
    if len(f_area) == 1:
      idx = areas.index(f_area[0])
      msg.color = COLOR[idx]
      #cv.imshow('Image', self.img)
      #cv.imshow('Mask', cv.bitwise_and(self.img, self.img, mask=masks[idx]))
    elif len(f_area) > 2:
      msg.color = 'Unidentified'

    # publish color
    self.pub.publish(msg)

def callback(msg):
  red = [np.array([0, 248, 102]), np.array([0, 255, 255])]
  green = [np.array([60, 247, 102]), np.array([60, 255, 255])]
  blue = [np.array([120, 248, 102]), np.array([120, 255, 255])]
  yellow = [np.array([30, 0, 102]), np.array([30, 255, 255])]
  orange = [np.array([14, 218, 102]), np.array([17, 255, 255])]
  purple = [np.array([150, 248, 102]), np.array([150, 255, 255])]
  colors = list()
  colors.append(red)
  colors.append(green)
  colors.append(blue)
  colors.append(yellow)
  colors.append(orange)
  colors.append(purple)

  pub = rospy.Publisher('object_info', object_info_msg, queue_size=10)
  bridge = CvBridge()
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    sd = color_detector(cv_image, colors, pub)
    sd.detecting()
    #hsv_finder(cv_image)
    #if cv.waitKey(1) == 27:
    #  cv.destroyAllWindows()

  except CvBridgeError as e:
    rospy.logfatal(e)

def main():
  rospy.init_node('image_processor')
  rospy.Subscriber('camera/camera_x/image_raw', Image, callback)
  rospy.spin()

if __name__ == "__main__":
  main()