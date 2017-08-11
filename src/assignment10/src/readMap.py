import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

#rospy.init_node("trajectory_node")
#trajectory_pub = rospy.Publisher("fub_trajectory_msgs", Point)
with open('sample_map_origin_map.txt') as f:
 content = f.readlines()
content = [x.strip() for x in content] 
for i in range(0,18):
 del content[0]
#coordinates hat die Koordinaten der Punkte
coordinates = []
point = Point()
for i in content:
 a = tuple(filter(None, i.split('	')))
 coordinates.append((a[1],a[2]))
 point.x = a[1]
 point.y = a[2]
 #trajectory_pub.publish(point)
 print(point)

 
