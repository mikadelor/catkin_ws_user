import rospy
import math

from std_msgs.msg import String, Float32, Int16
from geometry_msgs.msg import Point, PointStamped

#TODO problem seems to be the gps having some issues, maybe try to filter those

#assignment 10 1
def readMap():
 with open('sample_map_origin_map.txt') as f:
  content = f.readlines()
 content = [x.strip() for x in content] 
 for i in range(0,18):
  del content[0]
 #coordinates has the coordinates of the txt file
 coordinates = []
 for i in content:
  a = tuple(filter(None, i.split('	')))
  coordinates.append((float(a[1]),float(a[2])))
 return coordinates

#das skalarprodukt ausrechnen
def skalarprodukt(vektor1, vektor2):
  return (vektor1[0] * vektor2[0] + vektor1[1] * vektor2[1])

#die norm ausrechnen
def norm(vektor):
  return math.sqrt(vektor[0]**2 + vektor[1]**2)

#angle between the direction of the car, when it starts and the point it is headed to next
def winkel(current_pos, dest_pos):
 wanted_dir = (dest_pos[0] - current_pos[0], dest_pos[1] - current_pos[1])
 vektor1 = wanted_dir
 vektor2 = start_dir
 #if-else for determining, if the car should head left or right
 if vektor1[1] > 0:
  return -math.degrees(math.acos(skalarprodukt(vektor1,vektor2) / (norm(vektor1) * norm(vektor2))))
 else:
  return math.degrees(math.acos(skalarprodukt(vektor1,vektor2) / (norm(vektor1) * norm(vektor2))))


def gps_callback(data):
 global coordinates, car_pos
 print("gps")
 car_pos = (data.point.x,data.point.y)
 current_pos_x = data.point.x
 current_pos_y = data.point.y
 dest_pos_x = coordinates[0][0] 
 dest_pos_y = coordinates[0][1]
 print("Test")
 #if the distance of the car to the next point is too low, the next point gets changed
 if(norm((dest_pos_x - current_pos_x, dest_pos_y - current_pos_y)) < minDist):
  coordinates.append(coordinates[0])
  del coordinates[0]
  dest_pos = coordinates[0]

 

def yaw_callback(data):
 global steering_pub,yaw_set,car_pos,init_yaw
 if yaw_set:
  init_yaw = data.data
  yaw_set = False
 #the direction the car is heading 
 currentHeading = data.data - init_yaw
 #the direction the car should be heading
 desiredHeading = (winkel(car_pos, coordinates[0]))
 print(desiredHeading)
 print(currentHeading)
 #bangbang controller
 steering = Kp * (-desiredHeading*2 + currentHeading) + calibratedZeroSteeringAngle
 if steering < 0:
  steering = 0
 elif steering > 180:
  steering = 180
 print(car_pos)
 print(coordinates[0])
 print(steering)

 if math.abs(lastSteering - currentSteering) < 40:
  steering_pub.publish(steering)
  lastSteering = currentSteering
 



rospy.init_node("drive_node")
Kp = 2
init_yaw = 0
yaw_set = True
calibratedZeroSteeringAngle = 70
lastSteering = calibratedZeroSteeringAngle
start_dir = (1,0)
car_pos = None
minDist = 0.50
start_pub = rospy.Publisher("/manual_control/stop_start",Int16,queue_size=1,latch=True)
steering_pub = rospy.Publisher("/manual_control/steering",Int16,queue_size=1,latch=True)
speed_pub = rospy.Publisher("/manual_control/speed",Int16,queue_size=1,latch=True)
start_pub.publish(0)
speed_pub.publish(-200)

steering_pub.publish(calibratedZeroSteeringAngle)
coordinates = readMap()
gps_sub = rospy.Subscriber("/visual_gps/", PointStamped, gps_callback)
yaw_sub = rospy.Subscriber("/model_car/yaw", Float32, yaw_callback)
rospy.spin()










