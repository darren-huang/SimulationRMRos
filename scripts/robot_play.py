import gym
import numpy as np
import time
import pygame
import math
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from robomaster import *
from utils import *
from pid_controller.msg import DynObstacleArray, DynObstacle

env = gym.make('Robomaster-v0').unwrapped  # GLOBAL


# transforms x,y ROS -> Sim // given that ROS gives origin top right, Sim gives origin bottom left
def ros_to_sim_x(x):
    return 800. - (x * 100.)


def ros_to_sim_y(y):
    return 500. - (y * 100.)


def ros_to_sim_yaw(yaw):
    return (to_degree(yaw) + 180.) % 360  # given as radian, convert to degree, flip 180


def sim_to_ros_x(x):
    return (800. - x) / 100.


def sim_to_ros_y(y):
    return (500. - y) / 100.


def sim_to_ros_yaw(yaw):
    return to_radian((yaw + 180) % 360)  # given degree, flip 180, convert to degree


def make_posed_stamped(sim_x, sim_y, sim_angle):
    # TODO CALL TRANSLATE SIMX TO ROS X
    new_x, new_y, new_yaw = sim_to_ros_x(sim_x), sim_to_ros_y(sim_y), sim_to_ros_yaw(sim_angle)
    quat = quaternion_from_euler(0, 0, new_yaw)
    pt = PoseStamped()
    pt.pose.orientation.x, pt.pose.orientation.y, pt.pose.orientation.z, pt.pose.orientation.w = quat[0], quat[1], quat[
        2], quat[3]
    pt.pose.position.x = new_x
    pt.pose.position.y = new_y
    return pt


def dynamic_obstacles_CB(dynObstacles):
    rospy.loginfo("msgs:{}".format(dynObstacles))
    env.add_temp_obstacles(
        [(ros_to_sim_x(obj.x), ros_to_sim_y(obj.y), obj.width, obj.height) for obj in dynObstacles.lists])


def pub_robo_odom_CB(msg):
    pos = msg.pose.pose.position
    x, y = pos.x, pos.y
    yaw = euler_from_quaternion(msg.pose.pose.orientation)[2]
    sim_x, sim_y, sim_yaw = ros_to_sim_x(x), ros_to_sim_y(y), ros_to_sim_yaw(yaw)
    env.set_pub_robot_pose(sim_x, sim_y, sim_yaw)

def pub_robo_enemy_odom_CB(msg):
    pos = msg.pose.pose.position
    x, y = pos.x, pos.y
    yaw = euler_from_quaternion(msg.pose.pose.orientation)[2]
    sim_x, sim_y, sim_yaw = ros_to_sim_x(x), ros_to_sim_y(y), ros_to_sim_yaw(yaw)
    env.set_enemy_sub_pose(sim_x, sim_y, sim_yaw)

def main():
    total_rounds = int(env.full_time / env.tau)
    joystick_control = True
    rospy.init_node('sim_sync', anonymous=True)
    pub = rospy.Publisher('short_term_position', Path, queue_size=10)
    rospy.Subscriber("/controller/dynamic_obstalces", DynObstacleArray, dynamic_obstacles_CB)
    rospy.Subscriber('/odom', Odometry, pub_robo_odom_CB)  # TODO check topic name - our robot
    rospy.Subscriber('/enemy_odom', Odometry, pub_robo_enemy_odom_CB)  # TODO check topic name - enemy robot
    rate = rospy.Rate(10)

    controlled_robot = "BLUE"
    my_robot = env.characters['robots'][1] if controlled_robot == "RED" else env.characters['robots'][0]
    initial_x, initial_y, initial_ang = my_robot.center.x, my_robot.center.y, my_robot.angle_radian

    print(total_rounds)
    t1 = float(time.clock())
    print(t1)
    for i in range(total_rounds):
        env.step()
        if env.ready_to_publish():
            path = env.compute_path_to_publish()
            posed_stamped_path = Path(None, [make_posed_stamped(x, y, angle) for x, y, angle in path])
            pub.publish(posed_stamped_path)

        rate.sleep()
        if env.finished:
            break


if __name__ == '__main__':
    main()
