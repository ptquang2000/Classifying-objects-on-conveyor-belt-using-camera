#!/usr/bin/env python
from time import time
import xml.dom.minidom
import sys
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
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
    # 'bear',
    # 'car',
    # 'giraffe',
    'helicopter',
    # 'train',
]


def model_spawner(model, index, name):

    try:

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
            model_name = f"{name}"
            model_path = RosPack().get_path('sim_env')+'/models/'+name+'/model.sdf'
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
        rospy.loginfo(f"Spawn message:{model_name}\n{status_message}")
        rate.sleep()

        # capture model
        resp = capture(f'{name}/{model_name}{index}')
        rospy.loginfo(f"Capture message:\n{resp}")
        rate.sleep()

        # delete model
        status_msg = delete_model(model_name)
        rospy.loginfo(f'Delete model:{model_name}\n{status_msg}')
        rate.sleep()

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)

def disable_physics():
    rospy.wait_for_service('gazebo/set_physics_properties')
    try:
        time_step = 0.0015
        max_update_rate = 1000.0
        gravity = Vector3(0.0, 0.0, 0.0)
        ode_config = ODEPhysics(False, 0, 0, 1.3, 0.0, 0.001, 100.0, 0.0, 0.2, 20)
        disable_physics_srv = rospy.ServiceProxy(
            'gazebo/set_physics_properties', SetPhysicsProperties)
        resp = disable_physics_srv(time_step, max_update_rate, gravity, ode_config)
        rospy.loginfo(resp.status_message)
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 4:
        rospy.logerr('Need model paramaters')
        sys.exit(1)
    model = args[1].strip()
    min = int(args[2].strip())
    max = int(args[3].strip())
    rospy.init_node('object_spawner', anonymous=True)
    disable_physics()
    rate = rospy.Rate(0.25)
    # spawn object
    spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    rospy.wait_for_service('gazebo/spawn_sdf_model', timeout=10)
    delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    rospy.wait_for_service('gazebo/delete_model', timeout=10)
    capture = rospy.ServiceProxy('image_capture', image_capture_srv)
    rospy.wait_for_service('image_capture', timeout=10)
    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    rate.sleep()
    try:
        if model == 'toy':
            for toy in toys:
                index = min
                while not rospy.is_shutdown() and index <= max:
                    rate.sleep()
                    model_spawner(model, index, toy)
                    index += 1

    except rospy.ROSInterruptException:
        pass
