import numpy as np
from math import *

def Dynamics(wheelbase, search_length, speed, dt):
    Deg2Rad = pi/180    #Factor for converting degee to radian
    max_steering = 30 * Deg2Rad   #Maximum steering angle in radians (both directions)
    steering_resolution = 5 * Deg2Rad #Steering angle resolution in degree
    steering_step_num = int(1 + (2 * max_steering)/steering_resolution)   #Number possible steering angles
    steering_step = np.linspace(-max_steering, max_steering, steering_step_num, endpoint = True)    #Array of steering angle
    steering_step = steering_step[steering_step != 0] #Remove 0 steering to prevent 0 as denominator
    R_rear = wheelbase/np.tan(steering_step)    #Calculate the turning radius of each steering of rear wheels in meter
    theta = search_length/R_rear    #Heading change of each steering in radian
    theta_future = (speed * dt)/R_rear   #Heading change of each steering taking speed into account in radian
    dy_rear = (-(-R_rear * (np.cos(theta) - 1))).tolist() #Displacement in x direction (left & right) in meter
    dy_rear.append(0)
    dx_rear = (R_rear * np.sin(theta)).tolist()    #Displacement in y direction (back & forth) in meter
    dx_rear.append(search_length)
    displacement_rear = np.transpose([dx_rear, dy_rear])
    dy_rear_future = (-(-R_rear * (np.cos(theta_future) - 1))).tolist() #Displacement in x direction (left & right) in meter taking speed into account
    dy_rear_future.append(0)
    dx_rear_future = (R_rear * np.sin(theta_future)).tolist()    #Displacement in y direction (back & forth) in meter taking speed into account
    dx_rear_future.append(speed*dt)
    displacement_rear_future = np.transpose([dx_rear_future, dy_rear_future])
    theta = theta.tolist()
    theta.append(0)
    theta_future = theta_future.tolist()
    theta_future.append(0)
    steering_step = steering_step.tolist()
    steering_step.append(0)

    return theta, theta_future, displacement_rear, displacement_rear_future, steering_step