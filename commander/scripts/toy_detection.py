#!/usr/bin/env python
import rospy
import sys
import cv2 as cv
import numpy as np
import re
import base64
from sensor_msgs.msg import Image
from commander.msg import toy_msg
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from rospkg import RosPack
from keras.models import load_model
from tflite_runtime.interpreter import Interpreter
from PIL import ImageOps, Image as myPIL

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
THRESH_HOLD = 0.5
radius = 5

class toy_detector():
    def __init__(self, keras, interpreter, labels):
        self._bridge = CvBridge()
        path = f"{RosPack().get_path('commander')}/"
        self._path = path+labels

        # Teachable machine
        self._pil_image = None
        self._model = load_model(path+keras)
        # Tensorflow
        self._image = None
        self._interpreter = Interpreter(path+interpreter)
        self._interpreter.allocate_tensors()
        _, self._input_height, self._input_width, _ = self._interpreter.get_input_details()[0]['shape']

        # Ros puslisher
        self._score_publisher = rospy.Publisher(f'toy_detection/{topic}/score', toy_msg, queue_size=10)
        self._img_publisher = rospy.Publisher(f'toy_detection/{topic}/img', String, queue_size=10)
        self._counter_publisher = rospy.Publisher(f'toy_detection/{topic}/counter', toy_msg, queue_size=10)

        # Counter
        self._counter = {label: 0 for label in self.get_labels.values()}
        self._right = {'coord': tuple(), 'name': ''}

    def set_input_tensor(self):
        tensor_index = self._interpreter.get_input_details()[0]['index']
        input_tensor = self._interpreter.tensor(tensor_index)()[0]
        input_tensor[:, :] = np.expand_dims((self._image-255)/255, axis=0)


    def get_output_tensor(self, index):
        output_details = self._interpreter.get_output_details()[index]
        tensor = np.squeeze(
            self._interpreter.get_tensor(output_details['index']))
        return tensor

    def draw_contours(self, idx, threshold=THRESH_HOLD):
        self.set_input_tensor()
        self._interpreter.invoke()
        # Get all output details
        scores = self.get_output_tensor(0)
        boxes = self.get_output_tensor(1)
        count = int(self.get_output_tensor(2))
        classes = self.get_output_tensor(3)

        results = []
        for i in range(count):
            if scores[i] >= threshold and classes[i] in idx:
                for result in results:
                    if result['label'] == classes[i]:
                        if result['score'] < scores[i]:
                            result['score'] = scores[i]
                        break
                else:
                    results.append( { 'box': boxes[i], 'label': classes[i], 'score': scores[i] } )

        return results

    
    def count_object(self, midpoint, label):
        if midpoint[0] - (int(CAMERA_WIDTH/2)+radius) > 0:
            self._right['coord'] = midpoint
            self._right['name'] = label
        elif label == self._right['name']:
            self._counter[label] += 1
            self._right['name'] = ''

        counter_msg = toy_msg()
        counter_msg.bear = self._counter['Bear']
        counter_msg.car = self._counter['Car']
        counter_msg.giraffe = self._counter['Giraffe']
        counter_msg.helicopter = self._counter['Helicopter']
        counter_msg.train = self._counter['Train']
        self._counter_publisher.publish(counter_msg)


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
        toy_score = toy_msg(*prediction)
        self._score_publisher.publish(toy_score)

        return [i for i, val in enumerate(prediction) if val != 0]


    def callback(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Teachable machine
            self._pil_image = myPIL.fromarray(cv.cvtColor(cv_image, cv.COLOR_BGR2RGB))
            idxes = self.classify_image()

            # Tensorflow
            self._image = cv.resize(cv.cvtColor(
                cv_image, cv.COLOR_BGR2RGB), (320, 320))
            results = self.draw_contours(idx=idxes)

            for result in results:
                label = self.get_labels[result['label']]
                ymin, xmin, ymax, xmax = result['box']
                xmin = int(max(1, xmin * CAMERA_WIDTH))
                xmax = int(min(CAMERA_WIDTH, xmax * CAMERA_WIDTH))
                ymin = int(max(1, ymin * CAMERA_HEIGHT))
                ymax = int(min(CAMERA_HEIGHT, ymax * CAMERA_HEIGHT))
                
                midpoint = ( xmin, int((ymin+ymax)/2) )
                self.count_object(midpoint, label)
                cv.circle(cv_image, midpoint, radius, (255, 0, 0), 2)

                cv.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 3)
                cv.putText(cv_image, label, (xmin, min(
                    ymax, CAMERA_HEIGHT-10)), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 4, cv.LINE_AA)

            cv.line(cv_image, (int(CAMERA_WIDTH/2)+radius, 0), (int(CAMERA_WIDTH/2)+radius, CAMERA_HEIGHT), (0, 0, 255), 4)
            # cv.imshow(topic, cv.resize(cv_image, (640, 480)))
            # if cv.waitKey(1) == 27:
            #     cv.destroyAllWindows()

            img = base64.b64encode( cv.imencode('.jpg', cv_image)[1] ).decode('utf-8')
            self._img_publisher.publish(img)


        except CvBridgeError as e:
            rospy.logfatal(e)


def main():
    detector = toy_detector(keras= keras_file, interpreter=model_file, labels=labels_file)
    rospy.Subscriber(f'camera/{topic}/image_raw', Image, detector.callback)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node('image_processor')
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 5:
        rospy.logerr('Not meet required arguments')
        sys.exit(1)
    topic = args[1]
    keras_file = args[2]
    model_file = args[3]
    labels_file = args[4]
    main()
