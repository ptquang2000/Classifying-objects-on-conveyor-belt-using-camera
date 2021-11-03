#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Image
from commander.msg import image_details_msg, toy_score_msg
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import re
from rospkg import RosPack
from keras.models import load_model
from PIL import ImageOps, Image as myPIL

class toy_detector():
    def __init__(self, keras, labels='labels.txt'):
        self._bridge = CvBridge()
        path = f"{RosPack().get_path('commander')}/"
        self._path = path+labels

        # Teachable machine
        self._pil_image = None
        self._model = load_model(path+keras)

        # Ros puslisher
        self._details_publisher = rospy.Publisher(f'toy_detection/{topic}/pre/', image_details_msg, queue_size=10)
        self._score_publisher = rospy.Publisher(f'toy_detection/{topic}/score/', toy_score_msg, queue_size=10)


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

    
    def classify_image(self):
        data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
        size = (224, 224)
        image = ImageOps.fit(self._pil_image, size, myPIL.ANTIALIAS)
        image_array = np.asarray(image)
        normalized_image_array = (image_array.astype(np.float32) / 127.0) - 1
        data[0] = normalized_image_array
        prediction = [int(i*100) for i in self._model.predict(data)[0]]

        # Pubslish score message
        toy_score = toy_score_msg(*prediction)
        self._score_publisher.publish(toy_score)

        idx = prediction.index(max(prediction))
        return prediction[idx], idx


    def callback(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Teachable machine
            self._pil_image = myPIL.fromarray(cv.cvtColor(cv_image, cv.COLOR_BGR2RGB))
            score, idx = self.classify_image()

            img_details_msg = image_details_msg()
            img_details_msg.score = score
            img_details_msg.label = str(idx)
            img_details_msg.camera = topic
            img_details_msg.image = msg
            self._details_publisher.publish(img_details_msg)

        except CvBridgeError as e:
            rospy.logfatal(e)


def main():
    rospy.init_node('image_processor')
    detector = toy_detector(keras= keras_file, labels=labels_file)
    rospy.Subscriber(f'camera/{topic}/image_raw', Image, detector.callback)
    rospy.spin()


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 4:
        rospy.logerr('Not meet required arguments')
        sys.exit(1)
    topic = args[1]
    keras_file = args[2]
    labels_file = args[3]
    main()
