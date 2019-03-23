import gym
import numpy as np
import time
import pygame
import math
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from robomaster import *
from utils import *
from pid_controller.msg import DynObstacleArray, DynObstacle 

# env.init_from_state([1058, 1, 408.6590398623192, 415.1700916785658, 68.14982363927881, 90, 73, 0, 0, 0, 1800, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 238.0520211100148, 79.06723937912398, 112.19999999999996, -50.713777541139656, 24, 0, 0, 0, 1650, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 26.799999999999628, 73.19999999999996, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

env = gym.make('Robomaster-v0').unwrapped
total_rounds = int(env.full_time / env.tau)
joystick_control = True
# clock = pygame.time.Clock()
# pygame.joystick.init()

#transforms x,y ROS -> Sim // given that ROS gives origin top right, Sim gives origin bottom left
def translate_x(x):
	return 800 - x
def translate_y(y):
	return 500 - y


def callback(dynObstacles):
	env.add_temp_obstacles([(translate_x(obj.x), translate_y(obj.y), obj.width, obj.height) for obj in dynObstacles.lists])

rospy.init_node('sim_sync', anonymous=True)
pub = rospy.Publisher('short_term_position', Path, queue_size=10)
rospy.Subscriber("/controller/dynamic_obstalces", DynObstacleArray, callback) #TODO set topic listener
rate = rospy.Rate(10)

controlled_robot = "BLUE"
my_robot = env.characters['robots'][1] if controlled_robot == "RED" else env.characters['robots'][0]
initial_x, initial_y, initial_ang = my_robot.center.x, my_robot.center.y, my_robot.angle_radian

print(total_rounds)
t1 = float(time.clock())
print(t1)
for i in range(total_rounds):

	# for e in pygame.event.get():
	# 	if e.type == pygame.QUIT:
	# 		# pygame.quit()
	# 		env.close()

	# j = pygame.joystick.Joystick(0)
	# j.init()

	# coords = [j.get_axis(0), j.get_axis(1), j.get_axis(3)]

	# for i in range(len(coords)):
	# 	if float_equals(coords[i], 0, 0.05):
	# 		coords[i] = 0

	# env.coords = coords
	# print(coords)
	# clock.tick(1)
	# t2 = float(time.clock())
	# env.step(executor)
	env.step()
	new_x, new_y, new_yaw = my_robot.center.x - initial_x, my_robot.center.y - initial_y, my_robot.angle_radian - initial_ang
	quat = quaternion_from_euler(0, 0, new_yaw)
	pt = PoseStamped()
	pt.pose.orientation.x, pt.pose.orientation.y, pt.pose.orientation.z, pt.pose.orientation.w = quat[0], quat[1], quat[2], quat[3]
	pt.pose.position.x = new_x
	pt.pose.position.y = new_y

	path = Path(None, [pt])
	pub.publish(path)
	rospy.loginfo(path)
	rate.sleep()

	# time.sleep(0.01)
	if env.finished:
		break
