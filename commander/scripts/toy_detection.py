#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from commander.msg import object_info_msg
from tflite_runtime.interpreter import Interpreter
import re
from rospkg import RosPack

# CAMERA_WIDTH = 640
# CAMERA_HEIGHT = 480
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080


class toy_detector():
    def __init__(self,label='labels.txt', interpreter='detect.tflite'):
        self._bridge = CvBridge()
        self._image = None
        path = f"{RosPack().get_path('commander')}/tensorflow/"
        self._path = path+label
        self._interpreter = Interpreter(path+interpreter)
        self._interpreter.allocate_tensors()
        _, self._input_height, self._input_width, _ = self._interpreter.get_input_details()[
            0]['shape']
        self._publisher = rospy.Publisher(f'toy_detection/{topic}', Image, queue_size=10)

    @property
    def get_labels(self):
        with open(self._path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            labels = {}
            for row_number, content in enumerate(lines):
                pair = re.split(r'[:\s]+', content.strip(), maxsplit=1)
                if len(pair) == 2 and pair[0].strip().isdigit():
                    labels[int(pair[0])] = pair[1].strip()
                else:
                    labels[row_number] = pair[0].strip()
        return labels

    def set_input_tensor(self):
        tensor_index = self._interpreter.get_input_details()[0]['index']
        input_tensor = self._interpreter.tensor(tensor_index)()[0]
        input_tensor[:, :] = np.expand_dims((self._image-255)/255, axis=0)

    def get_output_tensor(self, index):
        output_details = self._interpreter.get_output_details()[index]
        tensor = np.squeeze(
            self._interpreter.get_tensor(output_details['index']))
        return tensor

    def detect_toys(self, threshold=0.8):
        self.set_input_tensor()
        self._interpreter.invoke()
        # Get all output details
        scores = self.get_output_tensor(0)
        boxes = self.get_output_tensor(1)
        count = int(self.get_output_tensor(2))
        classes = self.get_output_tensor(3)

        results = []
        for i in range(count):
            if scores[i] >= threshold:
                result = {
                    'bounding_box': boxes[i],
                    'class_id': classes[i],
                    'score': scores[i]
                }
                results.append(result)
        return results


    def callback(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._image = cv.resize(cv.cvtColor(
                cv_image, cv.COLOR_BGR2RGB), (320, 320))
                #cv_image, cv.COLOR_BGR2RGB), (640, 640))
            results = self.detect_toys()

            rospy.loginfo(results)
            for result in results:
                ymin, xmin, ymax, xmax = result['bounding_box']
                xmin = int(max(1, xmin * CAMERA_WIDTH))
                xmax = int(min(CAMERA_WIDTH, xmax * CAMERA_WIDTH))
                ymin = int(max(1, ymin * CAMERA_HEIGHT))
                ymax = int(min(CAMERA_HEIGHT, ymax * CAMERA_HEIGHT))

                cv.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 3)
                text = f"{self.get_labels[int(result['class_id'])]} {int(result['score']*100)}%"
                cv.putText(cv_image, text, (xmin, min(
                    ymax, CAMERA_HEIGHT-20)), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 4, cv.LINE_AA)

            self._publisher.publish(self._bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
            '''
            cv.imshow(topic, cv.resize(cv_image, (320, 240)))
            if cv.waitKey(1) == 27:
                cv.destroyAllWindows()
            '''

        except CvBridgeError as e:
            rospy.logfatal(e)


def main():
    rospy.init_node('image_processor')
    detector = toy_detector()
    rospy.Subscriber(f'camera/{topic}/image_raw', Image, detector.callback)
    rospy.spin()


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 2:
        rospy.logerr('Not meet required arguments')
        sys.exit(1)
    topic = args[1]
    main()
