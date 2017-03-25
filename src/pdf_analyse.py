#!/usr/bin/env python
from matplotlib.backends.backend_pdf import PdfPages, Stream
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rospy
import sys

from ackermann_msgs.msg import AckermannDrive
from arc_msgs.msg import State
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float32MultiArray

#Name of pdf file.
file_name = sys.argv[0] + "_analyse.pdf"
#Topic names.
navigation_info_topic = 'navigation_info' #rospy.get_param("/topic/NAVIGATION_INFO")
repeat_path_topic = 'path' #rospy.get_param("/topic/PATH")
state_topic = 'state' #rospy.get_param("/topic/STATE")
steering_angle_topic = 'state_steering_angle' #rospy.get_param("/topic/STATE_STEERING_ANGLE")
stellgroessen_topic = 'stellgroessen_safe' #rospy.get_param("/topic/STELLGROESSEN_SAFE")
teach_path_topic = 'teach_path' #rospy.get_param("/topic/TEACH_PATH")
tracking_error_topic = 'tracking_error' #rospy.get_param("/topic/TRACKING_ERROR")
wheel_left_topic = 'wheel_rear_left' #rospy.get_param("/topic/WHEEL_REAR_LEFT")
wheel_right_topic = 'wheel_rear_right' #rospy.get_param("/topic/WHEEL_REAR_RIGHT")
#Init time.
init_time = 0
#Information vectors.
braking_distance = []
distance_end = []
distance_start = []
index_path = []
index_steering = []
index_velocity = []
radius = []
repeat_path = []
should_steering_angle = []
should_velocity = []
should_safe_steering_angle = []
should_safe_velocity = []
teach_path = []
time = []
tracking_error = []
velocity = []
velocity_bound_physical = []
velocity_bound_teach = []
wheel_left = []
wheel_right = []
#Current information.
current_breaking_distance = 0
current_distance_end = 0
current_distance_start = 0
current_index_path = 0
current_index_steering = 0
current_index_velocity = 0
current_radius = 0
current_should_steering_angle = 0
current_should_velocity = 0
current_should_safe_steering_angle = 0
current_should_safe_velocity = 0
current_steering_angle = 0
current_tracking_error = 0
current_velocity = 0
current_velocity_bound_physical = 0
current_velocity_bound_teach = 0
current_wheel_left = 0
current_wheel_right = 0

def createPDF():
	#Create PDF.
	pdf = PdfPages('Test_Analysis.pdf')
	#Plotting paths.
	np_teach_path = np.array(teach_path)
	np_repeat_path = np.array(repeat_path)
	plt.plot(np_teach_path[0,],np_teach_path[1,],'ro', 
			 np_repeat_path[0,],np_repeat_path[1,],'bo')
	plt.title('Teach and Repeat Path')
	pdf.savefig()
	plt.close()
	#Create Dataframe.
	df = pd.DataFrame({'time':time,'ss': should_steering_angle})
	table = df.plot()
	fig = table.get_figure()
	pdf.savefig()
	#Close PDF.
	pp.close()

def navigationInfoCallback(msg):
	current_distance_start = msg.data[0]
	current_distance_end = msg.data[1]
	current_index_steering = msg.data[2]
	current_should_steering_angle = msg.data[3]
	current_index_velocity = msg.data[4]
	current_radius = msg.data[5]
	current_velocity_bound_physical = msg.data[6]
	current_breaking_distance = msg.data[7]
	current_velocity_bound_teach = msg.data[8]
	current_should_velocity = msg.data[9]
	update()
    
def repeatPathCallback(msg):
	repeat_path = []
	for element in msg.poses:
		path_element = [element.position.x, element.position.y]
		repeat_path.append(path_element)
	update()

def stateCallback(msg):
	current_index_path = msg.current_arrayposition
	current_velocity = msg.pose_diff
	update()

def steeringAngleCallback(msg):
	current_steering_angle = msg.data
	update()

def stellgroessenCallback(msg):
	current_should_safe_steering_angle = msg.steering_angle
	current_should_safe_velocity = msg.speed
	update()

def teachPathCallback(msg):
	teach_path = []
	for element in msg.poses:
		path_element = [element.position.x, element.position.y]
		teach_path.append(path_element)
	update()

def trackingErrorCallback(msg):
	current_tracking_error = msg.data

def update():
	braking_distance.append(current_breaking_distance)
	distance_end.append(current_distance_end)
	distance_start.append(current_distance_start)
	index_path.append(current_index_path)
	index_steering.append(current_index_steering)
	index_velocity.append(current_index_velocity)
	radius.append(current_radius)
	should_steering_angle.append(current_should_steering_angle)
	should_velocity.append(current_should_velocity)
	should_safe_steering_angle.append(current_should_safe_steering_angle)
	should_safe_velocity.append(current_should_safe_velocity)
	time.append(rospy.get_rostime()-init_time)
	tracking_error.append(current_tracking_error)
	velocity.append(current_velocity)
	velocity_bound_physical.append(current_velocity_bound_physical)
	velocity_bound_teach.append(current_velocity_bound_teach)
	wheel_left.append(current_wheel_left)
	wheel_right.append(current_wheel_right)

def wheelLeftCallback(msg):
	current_wheel_left = msg.data
	update()

def wheelRightCallback(msg):
	current_wheel_right = msg.data
	update()

def main():
	#Init ros.
	rospy.init_node('analyse_pdf')
	init_time = rospy.get_rostime()
	#Init Subscriber.
	rospy.Subscriber(navigation_info_topic, Float32MultiArray, navigationInfoCallback)
	rospy.Subscriber(repeat_path_topic, Path, repeatPathCallback)
	rospy.Subscriber(state_topic, State, stateCallback)
	rospy.Subscriber(steering_angle_topic, Float64, steeringAngleCallback)
	rospy.Subscriber(stellgroessen_topic, AckermannDrive, stellgroessenCallback)
	rospy.Subscriber(teach_path_topic, Path, teachPathCallback)
	rospy.Subscriber(tracking_error_topic, Float64, trackingErrorCallback)
	rospy.Subscriber(wheel_left_topic, Float64, wheelLeftCallback)
	rospy.Subscriber(wheel_right_topic, Float64, wheelRightCallback)
	#Init subscribing loop.
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()
	#Writing in pdf.
	createPDF()



if __name__ == '__main__':
	main()
