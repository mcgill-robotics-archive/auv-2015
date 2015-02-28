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
  FIXED_FRAME = "/robot/initial_horizon"
  HORIZON_FRAME = "/robot/horizon"

  velocity_publisher = None
  position_publisher = None
  cv_target_publisher = None
  sonar_target_publisher = None
  filtered_depth = -1
  target_info = ""

  """callback for subscriber on /state_estimation/filtered_depth
  stores filtered_depth for access in tasks
  """
  def filtered_depth_callback(self, msg):
    self.filtered_depth = msg.data

  """callback for subscriber on /front_cv/target_info
  stores information regarding current CV object beside position
  """
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
    velocity_msg = SetVelocity()
    velocity_msg.surgeSpeed = desired[0]
    velocity_msg.swaySpeed = desired[1]
    velocity_msg.depth = desired[2]
    velocity_msg.roll = desired[3]
    velocity_msg.pitch = desired[4]
    velocity_msg.yaw = desired[5]
    self.velocity_publisher.publish(velocity_msg)

  def set_position(self, desired):
    position_msg = SetPosition()
    position_msg.xPos = desired[0]
    position_msg.yPos = desired[1]
    position_msg.depth = desired[2]
    position_msg.roll = desired[3]
    position_msg.pitch = desired[4]
    position_msg.yaw = desired[5]
#    positionmsg.frame = desired[6]

    self.position_publisher.publish(position_msg)

  def set_cv_target(self, new_target):
    cv_msg = CVTarget()
    cv_msg.CVTarget = new_target
    self.CV_target_publisher.publish(cv_msg)

  def set_sonar_target(self, new_target):
    sonar_msg = SonarTarget()
    sonar_msg.SonarTarget = new_target
    self.sonar_target_publisher.publish(sonar_msg)

  def print_info(self, msg):
    rospy.loginfo(msg)  

  def ros_init(self):
    rospy.init_node('autonomy', anonymous=False)

    rospy.Subscriber("state_estimation/filtered_depth", Int8, self.filtered_depth_callback)
    rospy.Subscriber("front_cv/target_info", String, self.target_info_callback)

    # TODO: make these all into services
    self.velocity_publisher = rospy.Publisher("autonomy/set_velocity", SetVelocity, queue_size = 1000)
    self.position_publisher = rospy.Publisher("autonomy/set_position", SetPosition, queue_size = 1000)
    self.cv_target_publisher = rospy.Publisher("autonomy/cv_target", CVTarget, queue_size = 1000)
    self.sonar_target_publisher = rospy.Publisher("autonomy/sonar_target", SonarTarget, queue_size = 1000)
    # Need to allow some time for topics to be created before publishing on them
    rospy.sleep(1)
    
if __name__ == '__main__':
  my_autonomy = Autonomy()
  my_autonomy.ros_init()
  my_task_controller = task_controller.TaskController(my_autonomy)
  my_task_controller.run_routine()
  rospy.spin()
