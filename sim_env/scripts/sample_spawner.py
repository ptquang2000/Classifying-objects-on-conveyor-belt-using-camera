#!/usr/bin/env python
from time import time
import xml.dom.minidom
import sys
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
from rospkg import RosPack
from random import randint, uniform
from tf.transformations import quaternion_from_euler
from commander.srv import image_capture_srv
import os

shapes = [
    'sphere', 'pyramid', 'box'
]
colors = ['Red', 'Green', 'Blue', 'Yellow', 'Purple', 'Orange', 'White']

toys = [
    'bear',
    'car',
    'train',
    'giraffe',
    'helicopter'
]


def model_spawner(model, index, name):

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    try:

        # spawn object
        spawn_sdf_model = rospy.ServiceProxy(
            'gazebo/spawn_sdf_model', SpawnModel)

        if model == 'shape':
            model_name = shapes[randint(0, len(shapes) - 1)]

            # get sdf
            model_path = RosPack().get_path('sim_env')+'/models/'+model_name+'/model.sdf'
            dom = xml.dom.minidom.parse(model_path)
            uri = dom.createElement('uri')
            text = dom.createTextNode(
                'file://media/materials/scripts/gazebo.material')
            uri.appendChild(text)
            name = dom.createElement('name')
            text = dom.createTextNode(
                f'Gazebo/{colors[randint(0, len(colors) - 1)]}')
            name.appendChild(text)
            script = dom.createElement('script')
            script.appendChild(uri)
            script.appendChild(name)
            material = dom.createElement('material')
            material.appendChild(script)
            visual = dom.getElementsByTagName('visual')[0]
            visual.appendChild(material)
            model_sdf = dom.toxml('utf-8').decode('utf-8')
            model_pose = Pose(Point(0, 3, 0.725), Quaternion(0, 0, 0, 0))

        if model == 'toy':
            model_name = name
            model_path = RosPack().get_path('sim_env')+'/models/'+model_name+'/model.sdf'
            with open(model_path, 'r') as f:
                model_sdf = f.read()
            model_pose = Pose(
                Point(0, 0, 0.525),
                Quaternion(*quaternion_from_euler(0, 0, uniform(-1, 0))
                           )) if index % 2 == 0 else Pose(
                Point(0, 0, 0.525),
                Quaternion(*quaternion_from_euler(0, 0, uniform(0, 1))
                           ))

        model_name += str(rospy.get_rostime().secs)

        status_message = spawn_sdf_model(
            model_name, model_sdf, "", model_pose, "world")
        rospy.loginfo("Status message: %s" % status_message)
        rate.sleep()

        # capture model
        capture = rospy.ServiceProxy('image_capture', image_capture_srv)
        rospy.wait_for_service('image_capture', timeout=5)
        resp = capture(f'{name}/{name}_{index}')
        rospy.loginfo(resp)
        path = RosPack().get_path('commander') + \
            f'/images/{name}/{name}_{index}_z.jpg'
        while not os.path.exists(path):
            pass

        # delete model
        delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        rospy.wait_for_service('gazebo/delete_model', timeout=5)
        status_msg = delete_model(model_name)
        rospy.loginfo(f'Delete model: {status_msg}')

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 3:
        rospy.logerr('Need model paramaters')
        sys.exit(1)
    model = args[1]
    max = int(args[2]) + 10
    rospy.init_node('object_spawner', anonymous=True)
    rate = rospy.Rate(0.5)
    try:
        if model == 'toy':
            for toy in toys:
                index = max - 10
                while not rospy.is_shutdown() and index < max:
                    rate.sleep()
                    model_spawner(model, index, toy)
                    index += 1

    except rospy.ROSInterruptException:
        pass
