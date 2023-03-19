#!/usr/bin/env python3

#Adding libraries
from geometry_msgs.msg import Twist
import rospy
from sensor_msgs.msg import Range

front_distance = left_distance = right_distance = 0

pub = None

def front_callback(msg):
    global front_distance
    front_distance = msg.range
    take_action()

def left_callback(msg):
    global left_distance
    left_distance = msg.range

def right_callback(msg):
    global right_distance
    right_distance = msg.range

def take_action():
  threshold_dist = 0.6
  linear_speed = 0.6
  angular_speed = 0.2

  msg = Twist()
  linear_x = 0
  angular_z = 0
  
  state_description = ''
  
  if front_distance > threshold_dist and left_distance > threshold_dist and right_distance > threshold_dist:
    state_description = 'case 1 - no obstacle'
    linear_x = linear_speed
    angular_z = 0
  elif front_distance < threshold_dist and left_distance < threshold_dist and right_distance < threshold_dist:
    state_description = 'case 7 - front and left and right'
    linear_x = -linear_speed
    angular_z = 0 
  elif front_distance < threshold_dist and left_distance > threshold_dist and right_distance > threshold_dist:
    state_description = 'case 2 - front'
    linear_x = 0
    angular_z = angular_speed
  elif front_distance > threshold_dist and left_distance > threshold_dist and right_distance < threshold_dist:
    state_description = 'case 3 - right'
    linear_x = 0
    angular_z = angular_speed
  elif front_distance > threshold_dist and left_distance < threshold_dist and right_distance > threshold_dist:
    state_description = 'case 4 - left'
    linear_x = 0
    angular_z = -angular_speed
  elif front_distance < threshold_dist and left_distance > threshold_dist and right_distance < threshold_dist:
    state_description = 'case 5 - front and right'
    linear_x = 0
    angular_z = angular_speed
  elif front_distance < threshold_dist and left_distance < threshold_dist and right_distance > threshold_dist:
    state_description = 'case 6 - front and left'
    linear_x = 0
    angular_z = -angular_speed
  elif front_distance > threshold_dist and left_distance < threshold_dist and right_distance < threshold_dist:
    state_description = 'case 8 - left and right'
    linear_x = linear_speed
    angular_z = 0
  else:
    state_description = 'unknown case'

  rospy.loginfo(state_description)
  msg.linear.x = linear_x
  msg.angular.z = angular_z
  pub.publish(msg)

def main():
    global pub
    rospy.init_node('obstacle_avoidance')
    rospy.Subscriber('/front_sonar', Range, front_callback)
    rospy.Subscriber('/left_sonar', Range, left_callback)     
    rospy.Subscriber('/right_sonar', Range, right_callback)     
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
  main()