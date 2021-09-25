#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel, ApplyBodyWrench
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Wrench
from rospkg import RosPack
from std_msgs.msg import Time, Duration

models = [
  'a_sphere'
]

def object_spawner():

  rospy.wait_for_service('gazebo/spawn_sdf_model')
  try:
    
    # spawn object
    spawn_sdf_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    model_name = models[0]
    model_path = RosPack().get_path('sim_env')+'/models/'+model_name+'/model.sdf'
    with open(model_path, 'r') as f:
      model_sdf = f.read()
    model_pose =  Pose(Point(0, 2.3, 0.525), Quaternion(0, 0, 0, 0))
    model_name += str(rospy.get_rostime().secs)
    
    status_message = spawn_sdf_model(model_name, model_sdf, "", model_pose, "world")
    print("Status message: %s"%status_message)

    # # push object
    # apply_body_wrench = rospy.ServiceProxy('gazebo/apply_body_wrench', ApplyBodyWrench)

    # model_point =  Point(0, 0, 0)
    # model_wrench = Wrench(Vector3(0, -1, 0), Vector3(0, 0, 0))
    # start_time = rospy.Time(0, 0)
    # duration = rospy.Duration(0, -1)
    
    # status_message = apply_body_wrench(model_name+'::link', 'world', model_point, model_wrench, start_time, duration)
    # print("Status message: %s"%status_message)
    
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

if __name__ == '__main__':
  rospy.init_node('object_spawner', anonymous=True)
  rate = rospy.Rate(0.25)
  try:
    while not rospy.is_shutdown():
      object_spawner()
      rate.sleep()
  except rospy.ROSInterruptException:
    pass
