#!/usr/bin/env python
import xml.dom.minidom
import sys
import rospy
from gazebo_msgs.srv import SpawnModel, SetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from rospkg import RosPack
from random import randint, uniform
from tf.transformations import quaternion_from_euler

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


def model_spawner(model):

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
            model_name = toys[randint(0, len(toys) - 1)]
            model_path = RosPack().get_path('sim_env')+'/models/'+model_name+'/model.sdf'
            dom = xml.dom.minidom.parse(model_path)
            plugin = dom.createElement('plugin')
            plugin.setAttribute('name', 'model_push')
            plugin.setAttribute('filename', 'libmodel_push.so')
            dom.getElementsByTagName('model')[0].appendChild(plugin)
            model_sdf = dom.toxml('utf-8').decode('utf-8')
            model_pose = Pose(
                Point(0, 2.5, 0.525),
                Quaternion(*quaternion_from_euler(0, 0, uniform(0, 6.28))
                           ))

        model_name += str(rospy.get_rostime().secs)

        status_message = spawn_sdf_model(
            model_name, model_sdf, "", model_pose, "world")
        #rospy.loginfo("Status message: %s"%status_message)

    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s" % e)

def disable_physics():
    rospy.wait_for_service('gazebo/set_physics_properties')
    try:
        time_step = 0.001
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
    if len(args) != 2:
        rospy.logerr('Need model paramaters')
        sys.exit(1)
    model = args[1]
    rospy.init_node('object_spawner', anonymous=True)
    rate = rospy.Rate(0.2)
    try:
        disable_physics()
        while not rospy.is_shutdown():
            model_spawner(model)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
