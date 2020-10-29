#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import rospy
import sys

markerArray = MarkerArray()
publisher = rospy.Publisher('markers', MarkerArray, queue_size=100)

def Plotter(center):

  marker = Marker()
  marker.header.frame_id = "world"
  marker.type = marker.SPHERE
  marker.action = marker.ADD
  marker.scale.x = 0.05
  marker.scale.y = 0.05
  marker.scale.z = 0.05
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 0.0
  marker.color.b = 0.0
  marker.pose.orientation.w = 1.0
  
  #if center.pose.position.x > 0.1:
  
  marker.pose.position.x = center.pose.position.x + 0.21 + 0.335
  marker.pose.position.y = center.pose.position.y - 0.495
  marker.pose.position.z = center.pose.position.z + 0.39

  markerArray.markers.append(marker)

   # Renumber the marker IDs
  id = 0
  for m in markerArray.markers:
    m.id = id
    id += 1
  print(center.pose.position.x)
  publisher.publish(markerArray)


def main(args):
  rospy.init_node('strawberry_plt')
  rospy.Subscriber("/sb_coordinates", PoseStamped, Plotter)
  print("Registering callback")
  rospy.spin()

if __name__=='__main__':
    main(sys.argv)



