#!/usr/bin/env python
"""@package docstring
This module is responsible for recieving data from other ROS nodes
as well as placing data from other modules onto publishers and services
"""
import rospy
import tf
from std_msgs.msg import Int8
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from auv_msgs.msg import SetVelocity
from auv_msgs.msg import SetPosition
from auv_msgs.msg import CVTarget
from auv_msgs.msg import SonarTarget

import task_controller

class Autonomy():
  velocity_publisher = None
  position_publisher = None
  CV_target_publisher = None
  sonar_target_publisher = None
  fixed_frame = "/robot/initial_horizon"
  horizon_frame = "/robot/horizon"
  filtered_depth = -1
  target_info = ""

  """callback for subsriber on /state_estimation/filtered_depth
  stores filtered_depth for access in tasks
  """
  def filtered_depth_callback(self, msg):
    self.filtered_depth = msg.data

  def target_info_callback(self, msg):
    self.target_info = msg.data  

  def get_transform(self, target_frame):
    listener = tf.TransformListener()
    t = rospy.Time(0)
    if listener.frameExists(target_frame) and listener.frameExists(self.horizon_frame):
      t = listener.getLatestCommonTime(target_frame, self.horizon_frame)
      print t
    position, quaternion = listener.lookupTransform(target_frame, self.horizon_frame, t)
    print position, quaternion
    return (position, quaternion)

  def set_velocity(self, desired):
    velocitymsg = SetVelocity()
    velocitymsg.surgeSpeed = desired[0]
    velocitymsg.swaySpeed = desired[1]
    velocitymsg.depth = desired[2]
    velocitymsg.roll = desired[3]
    velocitymsg.pitch = desired[4]
    velocitymsg.yaw = desired[5]
    self.velocity_publisher.publish(velocitymsg)

  def set_position(self, desired):
    positionmsg = SetPosition()
    positionmsg.xPos = desired[0]
    positionmsg.yPos = desired[1]
    positionmsg.depth = desired[2]
    positionmsg.roll = desired[3]
    positionmsg.pitch = desired[4]
    positionmsg.yaw = desired[5]
  #  positionmsg.frame = desired[6]
    self.position_publisher.publish(positionmsg)

  def set_cv_target(self, new_target):
    CVmsg = CVTarget()
    CVmsg.CVTarget = new_target
    self.CV_target_publissonar_target_publisher

  def set_sonar_target(self, new_target):
    Sonarmsg = SonarTarget()
    Sonarmsg.SonarTarget = new_target
    self.sonar_target_publisher.publish(Sonarmsg)

  def rosInit(self):
    rospy.init_node('autonomy', anonymous=False)
    rospy.Subscriber("state_estimation/filtered_depth", Int8, self.filtered_depth_callback)
    rospy.Subscriber("front_cv/target_info", String, self.target_info_callback)

    self.velocity_publisher = rospy.Publisher("autonomy/set_velocity", SetVelocity, queue_size = 1000)
    self.position_publisher = rospy.Publisher("autonomy/set_position", SetPosition, queue_size = 1000)
    self.CV_target_publisher = rospy.Publisher("autonomy/cv_target", CVTarget, queue_size = 1000)  
    self.sonar_target_publisher = rospy.Publisher("autonomy/sonar_target", SonarTarget, queue_size = 1000)
    
if __name__ == '__main__':
  my_autonomy = Autonomy()
  my_autonomy.rosInit()

  task_controller.populate_tasks()
  task_controller.populate_routine()
#  my_autonomy.get_transform("/horizon")
  task_controller.temp(my_autonomy)
