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

  depth_subscriber = None
  cv_target_subscriber = None

  velocity_publisher = None
  position_publisher = None
  cv_target_publisher = None
  sonar_seek_target_publisher = None
  sonar_track_target_publisher = None

  filtered_depth = -1
  target_info = ""

  """return a list of all topics the autonomy node subscribes to
  """
  def get_subscribers(self):
    return [self.depth_subscriber, self.cv_target_subscriber]

  """callback for subscriber on /state_estimation/filtered_depth
  stores filtered_depth for access in tasks
  """
  def filtered_depth_callback(self, msg):
    self.filtered_depth = msg.data

  """returns the filtered depth value from state estimation for
  use in tasks 
  (should be a float)
  """
  def get_filtered_depth(self):
    return self.filtered_depth

  """callback for subscriber on /front_cv/target_info
  stores information regarding current CV object beside position
  """
  def target_info_callback(self, msg):
    self.target_info = msg.data

  """returns the target info value from computer vision for
  use in tasks, this will be non-position information such as color
  (should be a string)
  """
  def get_target_info(self):
    return self.target_info  

  """given a target frame, returns the relative position and angle
  between the robot and the given frame
  (should be a tuple of a position msg and a quaternion msg)
  """
  def get_transform(self, target_frame):
    listener = tf.TransformListener()
    t = rospy.Time(0)
    if (listener.frameExists(target_frame) and 
        listener.frameExists(self.horizon_frame)):
      t = listener.getLatestCommonTime(target_frame, self.horizon_frame)
#      print t
    position, quaternion = listener.lookupTransform(target_frame, self.horizon_frame, t)
#    print position, quaternion
    return (position, quaternion)


  """switches controls to open loop control and gives them velocities
  in all directions
  [all 0s should stop motors, not float them]
  """
  def set_velocity(self, desired):
    velocity_msg = SetVelocity()
    velocity_msg.surgeSpeed = desired[0]
    velocity_msg.swaySpeed = desired[1]
    velocity_msg.depth = desired[2]
    velocity_msg.roll = desired[3]
    velocity_msg.pitch = desired[4]
    velocity_msg.yaw = desired[5]

    self.velocity_publisher.publish(velocity_msg)

  """switches controls to closed loop control and gives them a
  relative position to a given frame
  """
  def set_position(self, desired):
    position_msg = SetPosition()
    position_msg.xPos = desired[0]
    position_msg.yPos = desired[1]
    position_msg.depth = desired[2]
    position_msg.roll = desired[3]
    position_msg.pitch = desired[4]
    position_msg.yaw = desired[5]
#    position_msg.frame = desired[6]

    self.position_publisher.publish(position_msg)

  """send CV a target object to look for
  """
  def set_cv_target(self, new_target):
    cv_msg = CVTarget()
    cv_msg.CVTarget = new_target
    self.cv_target_publisher.publish(cv_msg)

  """send Sonar a target object to seek for
  (sonar does not need to have seen this object before)
  """
  def set_sonar_seek_target(self, new_target):
    sonar_msg = SonarTarget()
    sonar_msg.SonarTarget = new_target
    self.sonar_seek_target_publisher.publish(sonar_msg)

  """send Sonar a target object to track
  (sonar must have seen this object before)
  """
  def set_sonar_track_target(self, new_target):
    sonar_msg = SonarTarget()
    sonar_msg.SonarTarget = new_target
    self.sonar_track_target_publisher.publish(sonar_msg)

  """print information to the ROS terminal
  """
  def print_info(self, msg):
    rospy.loginfo(msg)  

  def ros_init(self):
    rospy.init_node('autonomy', anonymous=False)

    self.depth_subscriber = rospy.Subscriber("state_estimation/filtered_depth", Int8, self.filtered_depth_callback)
    self.cv_target_subscriber = rospy.Subscriber("front_cv/target_info", String, self.target_info_callback)

    self.velocity_publisher = rospy.Publisher("autonomy/set_velocity", SetVelocity, queue_size = 1000)
    self.position_publisher = rospy.Publisher("autonomy/set_position", SetPosition, queue_size = 1000)
    self.cv_target_publisher = rospy.Publisher("autonomy/cv_target", CVTarget, queue_size = 1000)  
    self.sonar_seek_target_publisher = rospy.Publisher("autonomy/sonar_seek_target", SonarTarget, queue_size = 1000)
    self.sonar_track_target_publisher = rospy.Publisher("autonomy/sonar_track_target", SonarTarget, queue_size = 1000)

if __name__ == '__main__':
  my_autonomy = Autonomy()
  my_autonomy.ros_init()
  my_task_controller = task_controller.TaskController(my_autonomy)
  my_task_controller.run_routine()