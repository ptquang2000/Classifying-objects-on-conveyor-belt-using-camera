#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from rospkg import RosPack
from random import randint
import xml.dom.minidom

models = [
'sphere', 'pyramid',
]
colors = ['Red', 'Green', 'Blue', 'Yellow', 'Purple', 'Orange', 'White']

def object_spawner():

  rospy.wait_for_service('gazebo/spawn_sdf_model')
  try:
    
    # spawn object
    spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    model_name = models[randint(0, len(models) - 1)]
    
    # get sdf
    model_path = RosPack().get_path('sim_env')+'/models/'+model_name+'/model.sdf'
    dom = xml.dom.minidom.parse(model_path)
    uri = dom.createElement('uri')
    text = dom.createTextNode('file://media/materials/scripts/gazebo.material')
    uri.appendChild(text)
    name = dom.createElement('name')
    text = dom.createTextNode(f'Gazebo/{colors[randint(0, len(colors) - 1)]}')
    name.appendChild(text)
    script = dom.createElement('script')
    script.appendChild(uri)
    script.appendChild(name)
    material = dom.createElement('material')
    material.appendChild(script)
    visual = dom.getElementsByTagName('visual')[0]
    visual.appendChild(material)
    model_sdf = dom.toxml('utf-8').decode('utf-8')
            
    model_pose =  Pose(Point(0, 3, 0.525), Quaternion(0, 0, 0, 0))
    model_name += str(rospy.get_rostime().secs)
    
    status_message = spawn_sdf_model(model_name, model_sdf, "", model_pose, "world")
    #rospy.loginfo("Status message: %s"%status_message)
    
  except rospy.ServiceException as e:
    rospy.loginfo("Service call failed: %s"%e)

if __name__ == '__main__':
  rospy.init_node('object_spawner', anonymous=True)
  rate = rospy.Rate(0.25)
  try:
    while not rospy.is_shutdown():
      object_spawner()
      rate.sleep()
  except rospy.ROSInterruptException:
    pass
