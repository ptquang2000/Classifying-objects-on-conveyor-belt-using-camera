#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from rospkg import RosPack
from random import randint

models = [
'sphere', 'pyramid',
'red_sphere', 'blue_sphere', 'green_sphere',
'yellow_sphere', 'orange_sphere', 'purple_sphere',
'red_pyramid', 'blue_pyramid', 'green_pyramid',
'yellow_pyramid', 'orange_pyramid', 'purple_pyramid'
]

def object_spawner():

  rospy.wait_for_service('gazebo/spawn_sdf_model')
  try:
    
    # spawn object
    spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    model_name = models[randint(0, len(models) - 1)]
    model_path = RosPack().get_path('sim_env')+'/models/'+model_name+'/model.sdf'
    with open(model_path, 'r') as f:
      model_sdf = f.read()
    # conveyor y = 7, z = 0.05, pose z = 0.5
    model_pose =  Pose(Point(0, 3, 0.525), Quaternion(0, 0, 0, 0))
    model_name += str(rospy.get_rostime().secs)
    
    status_message = spawn_sdf_model(model_name, model_sdf, "", model_pose, "world")
    #rospy.loginfo("Status message: %s"%status_message)
    
  except rospy.ServiceException as e:
    rospy.loginfo("Service call failed: %s"%e)

if __name__ == '__main__':
  rospy.init_node('object_spawner', anonymous=True)
  rate = rospy.Rate(0.4)
  try:
    while not rospy.is_shutdown():
      object_spawner()
      rate.sleep()
  except rospy.ROSInterruptException:
    pass
