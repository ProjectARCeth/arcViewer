#!/usr/bin/env python
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
import os
import rospy
import sys

from ackermann_msgs.msg import AckermannDrive
from arc_msgs.msg import State
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float32MultiArray

#Path to latex.
file_path = str(sys.argv[1])
#General constants.
v_freedom = 0.0;
#Init time and array index.
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
steering_angle = []
teach_path = []
time = []
tracking_error = []
velocity = []
velocity_bound_physical = []
velocity_bound_teach = []
velocity_teach = []
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

def createTxtEntry(file, array, index, name):
	if(index == -1):
		string = name
	else:
		try:
			string = str(round(array[index], 4))
		except Exception, e:
			pass
	file.write("\t"+string)

def getIndexArray(array):
	index_array = []
	index = 0
	for element in array:
		index_array.append(index)
		index += 1
	return index_array

def getTwoArrays(array):
	x = []
	y = []
	for i in range(0, len(array)):
		x.append(array[i][0])
		y.append(array[i][1])
	return x,y

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
		path_element = [element.pose.position.x, element.pose.position.y]
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
		path_element = [element.pose.position.x, element.pose.position.y]
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
	steering_angle.append(current_steering_angle)
	time.append(rospy.get_time()-init_time)
	tracking_error.append(current_tracking_error)
	velocity.append(current_velocity)
	velocity_bound_physical.append(current_velocity_bound_physical)
	velocity_bound_teach.append(current_velocity_bound_teach)
	velocity_teach.append(current_velocity_bound_teach-v_freedom)
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
	init_time = rospy.get_time()
	#General constants.
	v_freedom = rospy.get_param("/control/V_FREEDOM")
	#Topic names.
	navigation_info_topic = rospy.get_param("/topic/NAVIGATION_INFO")
	repeat_path_topic = rospy.get_param("/topic/PATH")
	state_topic = rospy.get_param("/topic/STATE")
	steering_angle_topic = rospy.get_param("/topic/STATE_STEERING_ANGLE")
	stellgroessen_topic = rospy.get_param("/topic/STELLGROESSEN_SAFE")
	teach_path_topic = rospy.get_param("/topic/TEACH_PATH")
	tracking_error_topic = rospy.get_param("/topic/TRACKING_ERROR")
	wheel_left_topic = rospy.get_param("/topic/WHEEL_REAR_LEFT")
	wheel_right_topic = rospy.get_param("/topic/WHEEL_REAR_RIGHT")
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
		rospy.spinOnce();
		rate.sleep()
	#Create plots.
	fig = plt.figure(figsize=(10, 7))
	gs = gridspec.GridSpec(4, 4)
	gs.update(hspace=0.4)

	ax0 = plt.subplot(gs[0, :4])
	plt.title("Path Analysis")
	index_array = getIndexArray(tracking_error)
	plt.plot(index_array, tracking_error)
	plt.ylabel('tracking_error[m]')

	ax1 = plt.subplot(gs[1, :4])
	index_base = getIndexArray(velocity)
	plt.plot(index_base, velocity)
	index_target = getIndexArray(velocity_teach)
	plt.plot(index_target, velocity_teach)
	plt.ylabel('velocity[m/s]')

	ax2 = plt.subplot(gs[2:4,:2])
	teach_x, teach_y = getTwoArrays(teach_path)
	repeat_x, repeat_y = getTwoArrays(repeat_path)
	plt.plot(teach_x, teach_y, 'ro', label="teach")
	plt.plot(repeat_x, repeat_y, 'bo', label="repeat")
	plt.ylabel('Teach and Repeat path')

	ax3= plt.subplot(gs[2:3,2:4])
	plt.axis('off')
	frame = plt.gca()
	frame.axes.get_xaxis().set_ticks([])
	frame.axes.get_yaxis().set_ticks([])
	path_vals =[['Time[s]',round(time[len(time)-2],3)],
				['Distance[m]', round(distance_start[len(distance_start)-2],3)]]
	path_table = plt.table(cellText=path_vals,
	                  	   colWidths = [0.1]*2,
	                       loc='center left')
	path_table.set_fontsize(14)
	path_table.scale(2.2,3)

	ax4= plt.subplot(gs[3:4,2:4])
	plt.axis('off')
	frame = plt.gca()
	frame.axes.get_xaxis().set_ticks([])
	frame.axes.get_yaxis().set_ticks([])
	mean_labels=['','Mean','Variance','Median']
	mean_vals=[['Track Error',round(np.mean(tracking_error),3),round(np.var(tracking_error),3),round(np.median(tracking_error),3)],
				['Velocity',round(np.mean(velocity),3),round(np.var(velocity),3),round(np.median(velocity),3)]]
	mean_table = plt.table(cellText=mean_vals,
	                  	   colWidths = [0.1]*4,
	                  	   colLabels=mean_labels,
	                  	   loc='center left')
	mean_table.set_fontsize(14)
	mean_table.scale(2.2,3)

	plt.savefig(file_path+"_infos.png")
	plt.close()
	#Create txt file table.
	file = open(file_path+"_infos.txt", "w")
	for index in range(-1, len(tracking_error)):
		createTxtEntry(file, getIndexArray(tracking_error), index, "Index")
		createTxtEntry(file, time, index, "Time")
		createTxtEntry(file, distance_start, index, "Start")
		createTxtEntry(file, distance_end, index, "End")
		createTxtEntry(file, index_path, index, "InPa")
		createTxtEntry(file, tracking_error, index, "TraEr")
		createTxtEntry(file, index_steering, index, "InSte")
		createTxtEntry(file, steering_angle, index, "Ste")
		createTxtEntry(file, should_steering_angle, index, "Stesh")
		createTxtEntry(file, should_safe_steering_angle, index, "Stesaf")
		createTxtEntry(file, index_velocity, index, "InVel")
		createTxtEntry(file, radius, index, "Rad")
		createTxtEntry(file, velocity, index, "Vel")
		createTxtEntry(file, should_velocity, index, "Velsh")
		createTxtEntry(file, should_safe_velocity, index, "Velsaf")
		createTxtEntry(file, velocity_bound_physical, index, "Velphy")
		createTxtEntry(file, velocity_bound_teach, index, "Veltea")
		createTxtEntry(file, braking_distance, index, "Brake")
		createTxtEntry(file, wheel_left, index, "Whle")
		createTxtEntry(file, wheel_right, index, "Wheri")
		file.write("\n")
	file.close()

if __name__ == '__main__':
	main()
