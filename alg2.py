#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan
import math
from operator import itemgetter


epsilon = 0.02



# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
def scan_callback(scan):
	global laser
	laser = scan



# funkcja obracająca robota w kierunku celu
def rotate_to_target(x, y, theta):
	global main_state
	new_angle = math.atan2(des_y - y, des_x - x)
	new_vel.angular.z = math.fabs((new_angle - theta)) * 0.2
	if( math.fabs(new_vel.angular.z) <= epsilon ):
		main_state = "forward"



# funkcja obracająca robota, jeżeli ten dojedzie do punktu w którym ma zawrócić
def turn():
	global laser, main_state
	global x, y, theta, turn_angle, is_first_hit
	global new_hit_x, new_hit_y, new_last_dist
	global hit_x, hit_y, last_dist
	new_vel.angular.z = math.fabs((turn_angle - theta)) * 0.2
	if( math.fabs(new_vel.angular.z) <= 0.3 ):
		new_vel.angular.z = 0
		hit_x = new_hit_x
		hit_y = new_hit_y
		last_dist = new_last_dist
		is_first_hit = True
		main_state = "wall_following"



# funkcja sprawdzająca czy robot uderzył w przeszkodę
def obstacle_is_hit():
	global laser
	if laser is not None:
		if( laser.ranges[0] <= 0.8 or laser.ranges[30] <= 0.8 or laser.ranges[330] <= 0.8):
			return 1
		return 0
	return 0



# funkcja poruszania się robota po linii do punktu docelowego i zapisująca punkt trafienia w przeszkodę - funkcja ta jest wywoływana tylko za pierwszym razem!
def forward():
	global main_state
	global hit_x, hit_y, x, y, des_x, des_y, last_dist, sWF
	new_vel.linear.x = 0.25
	new_vel.angular.z = 0
	if( obstacle_is_hit() ):
		new_vel.linear.x = 0
		main_state = "count_distance"
		hit_x = x
		hit_y = y
		sWF = -1
		last_dist = math.sqrt( math.pow(des_x - x, 2) + math.pow(des_y - y, 2) )



# funkcja poruszania się robota po linii do punktu docelowego i zapisująca punkt trafienia w przeszkodę - funkcja ta jest wywoływana zawsze, oprócz pierwszego razu!
def new_forward():
	global new_hit_x, new_hit_y, new_last_dist, sWF
	global main_state
	global x, y, des_x, des_y
	new_vel.linear.x = 0.25
	new_vel.angular.z = 0
	if( obstacle_is_hit() ):
		main_state = "wall_following"
		sWF = -1
		new_hit_x = x
		new_hit_y = y
		new_last_dist = math.sqrt( math.pow(des_x - x, 2) + math.pow(des_y - y, 2) )



# funkcja licząca odległość robota do punktu docelowego
def count_distance():
	global main_state
	global des_x, des_y, hit_x, hit_y, last_dist
	main_state = "wall_following"
	last_dist = math.sqrt( math.pow( (des_x - hit_x), 2) + math.pow( (des_y - hit_y), 2) )



# funkcja sprawdzająca czy robot może się poruszać po linii prostej w kierunku celu
def able_to_move_to_dest():
	global laser, des_x, des_y, last_dist
	global x, y, theta
	des_angle = math.atan2(des_y - y, des_x - x)
	dist_from_des = math.sqrt( math.pow(des_x - x, 2) + math.pow(des_y - y, 2) )
	if( laser.ranges[0] >= 2.5 and math.fabs(des_angle - theta) < 0.2 and dist_from_des < last_dist):
		return True
	else:
		return False



# funkcja sprawdzająca czy robot pierwszy raz uderzył w dany punkt
def check_if_out():
	global x, y, hit_x, hit_y, is_first_hit
	dist_from_hit = math.sqrt( math.pow(hit_x - x, 2) + math.pow(hit_y - y, 2) )
	if( dist_from_hit > 0.6 ):
		is_first_hit = False



# funkcja sprawdzająca czy robot uderzył ponownie w ten sam punkt
def reach_same_hit():
	global des_x, des_y, hit_x, hit_y, last_dist
	global x, y
	dist_from_hit = math.sqrt( math.pow(hit_x - x, 2) + math.pow(hit_y - y, 2) )
	dist_from_hit_to_des = math.sqrt( math.pow(des_x - hit_x, 2) + math.pow(des_y - hit_y, 2) )
	dist_from_des = math.sqrt( math.pow(des_x - x, 2) + math.pow(des_y - y, 2) )
	if( dist_from_hit < 0.5 and math.fabs( dist_from_hit_to_des - dist_from_des) < 0.5 ):
		return True
	else:
		return False



# funkcja wyznaczająca odległość od ściany we wzoru wysokości trójkąta
def distance_from_wall(sWF):
	global laser
	beta = 42
	if( sWF == -1 ):
		return laser.ranges[270] * laser.ranges[270+beta] * math.sin(beta) / math.sqrt(math.pow(laser.ranges[270], 2)
		+ math.pow(laser.ranges[270+beta], 2) - 2*laser.ranges[270]*laser.ranges[270+beta]*math.cos(beta))
	else:
		return laser.ranges[90] * laser.ranges[90-beta] * math.sin(beta) / math.sqrt(math.pow(laser.ranges[90], 2)
		+ math.pow(laser.ranges[90-beta], 2) - 2*laser.ranges[90]*laser.ranges[90-beta]*math.cos(beta))



# funkcja korygująca ruch robota wzdłuż ściany
def wall_following_and_aligning(sWF):
	global laser
	beta = 42
	limit = 0.5 - distance_from_wall(sWF)
	if( math.fabs(limit) > 0.1 ):
		if( limit < 0.1 ):
			return (-1) * sWF * 0.25
		else:
			return sWF * 0.25
	elif( math.fabs(limit) < 0.1 ):
		if( laser.ranges[270] > laser.ranges[270+beta] * math.cos(beta) and sWF == -1 ):
			return (-1) * sWF * 0.25
		elif( laser.ranges[90] > laser.ranges[90-beta] * math.cos(beta) and sWF == 1 ):
			return sWF * 0.25
		else:
			return (-1) * sWF * 0.25
	else:
		return 0



# funkcja realizująca algorytm śledzenia ściany
def wall_following():
	global main_state, wall_state
	global sWF, laser
	global is_first_hit, turn_angle
	beta = 42

	check_if_out()
	if( able_to_move_to_dest() ):
		main_state = "new_forward"
		return
	elif( not is_first_hit ):
		if( reach_same_hit() ):
			sWF = 1
			new_vel.linear.x = 0
			main_state = "turn"
			if( theta >= 0 ):
				turn_angle = theta - 3.14
			else:
				turn_angle = theta + 3.14
			return

	if( wall_state == "rotate_to_align_wall" ):
		new_vel.linear.x = 0 
		new_vel.angular.z = sWF * 0.25

		if( sWF == 1 ):
			if( laser.ranges[86] > laser.ranges[92] ):
				wall_state = "wall_following_and_aligning"
				return
			elif( laser.ranges[90-beta] == float("inf") ):
				new_vel.angular.z = 0
				wall_state = "rotate_around_corner"
				return
		elif( sWF == -1 ):
			if( laser.ranges[274] > laser.ranges[268] ):
				wall_state = "wall_following_and_aligning"
				return
			elif( laser.ranges[270+beta] == float("inf") ):
				wall_state = "rotate_around_corner"
				return

	elif( wall_state == "wall_following_and_aligning" ):
		new_vel.linear.x = 0.25
		new_vel.angular.z = wall_following_and_aligning(sWF)

		if( laser.ranges[0] <= 0.7 ):
			wall_state = "rotate_to_align_wall"
			return
		elif( laser.ranges[270+beta] == float("inf") and sWF == -1 ):
			wall_state = "rotate_around_corner"
			return
		elif( laser.ranges[90-beta] == float("inf") and sWF == 1 ):
			wall_state = "rotate_around_corner"
			return

	elif( wall_state == "rotate_around_corner" ):
		new_vel.linear.x = 0.25
		new_vel.angular.z = (-1) * sWF * 0.25 / 0.7

		if( sWF == 1 ):
			if( laser.ranges[90] < laser.ranges[92] ):
				wall_state = "wall_following_and_aligning"
				return
			elif( laser.ranges[90] <= 0.5 ):
				wall_state = "rotate_to_align_wall"
				return
		elif( sWF == -1 ):
			if( laser.ranges[270] < laser.ranges[268] ):
				wall_state = "wall_following_and_aligning"
				return
			elif( laser.ranges[0] <= 0.7 ):
				wall_state = "rotate_to_align_wall"
				return



# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
	global laser
	global start_x, start_y, get_first_point
	global des_x, des_y
	global hit_x, hit_y, last_dist
	global new_hit_x, new_hit_y, new_last_dist
	global main_state, wall_state, sWF
	global new_vel
	global x, y, theta

	pose = Pose()
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y
	pose.theta = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
	print "Pozycja x: ",odom.pose.pose.position.x
	print "Pozycja y: ",odom.pose.pose.position.y
	print "Pozycja theta: ",pose.theta

	# warunek uzyskujący punkt startowy i wyliczający kąt docelowy
	if( get_first_point ):
		start_x = odom.pose.pose.position.x
		start_y = odom.pose.pose.position.y
		get_first_point = False

	x = pose.x
	y = pose.y
	theta = pose.theta

	# warunek kończący program po osiągnięciu punktu docelowego
	if( math.sqrt( math.pow(des_x - x, 2) + math.pow(des_y - y, 2) ) <= 0.8 ):
		new_vel.linear.x = 0 
		new_vel.angular.z = 0
		return
	
	
	# część odpowiedzialna za jazdę robota do przodu w kierunku T
	if( main_state == "rotate_to_target" ):
		rotate_to_target(pose.x, pose.y, pose.theta)
	if( main_state == "forward" ):
		forward()
	if( main_state == "new_forward" ):
		new_forward()
	if( main_state == "count_distance" ):
		count_distance()
	if( main_state == "wall_following" ):
		wall_following()
	if( main_state == "turn" ):
		turn()



if __name__== "__main__":
	# deklaracja globalnej zmiennej lasera
	global laser
	laser = None
	global x, y, theta
	x = 0
	y = 0
	theta = 0

	# zmienne odpowiedzialne za uzyskanie startowej pozycji robota
	global start_x, start_y, get_first_point
	start_x = 0
	start_y = 0
	get_first_point = True

	# zmienne odpowiedzialne za punkt destynacji
	global des_x, des_y, turn_angle
	des_x = -2.5
	des_y = 13
	turn_angle = 0

	# zmienne odpowiedzialne za przechowanie punktu, w którym nastąpiło uderzenie w ścianę i odległości do celu
	global hit_x, hit_y, last_dist, is_first_hit
	global new_hit_x, new_hit_y, new_last_dist
	hit_x = 0
	hit_y = 0
	last_dist = float("inf")
	is_first_hit = True
	new_hit_x = 0
	new_hit_y = 0
	new_last_dist = float("inf")

	# zmienne do obsługi bloków algorytmu
	global main_state, wall_state, sWF
	main_state = "rotate_to_target"
	wall_state = "rotate_to_align_wall"
	sWF = -1

	# inne globalne zmienne pomocnicze
	global min_from_right
	min_from_right = float("inf")

	global new_vel
	new_vel = Twist()
	rospy.init_node('wr_zad', anonymous=True)
	print("ready")
	rospy.Subscriber( '/odom' , Odometry, odom_callback)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber( '/scan' , LaserScan, scan_callback)
	
	rate=rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
		pub.publish(new_vel)#wyslanie predkosci zadanej
		rate.sleep()

	print("END")
