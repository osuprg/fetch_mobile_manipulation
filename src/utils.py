#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  6 12:12:18 2020

@author: sritee
"""


import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from tf.transformations import euler_from_quaternion as quat_to_euler



def pose_to_array(pose, orientation = False):
    """converts pose message to array of x y z, orientation includes yaw only"""
    
    
    if not orientation:
        return np.array([pose.position.x, pose.position.y, pose.position.z])
    else:
        yaw = quat_to_euler([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[-1]
        return np.array([pose.position.x, pose.position.y, pose.position.z, yaw])
    
    

def extract_poses_with_times(dataframe, include_navigation_time = False):
    
    assert(type(include_navigation_time) == bool)
    
    result = np.zeros([dataframe.shape[0], 3]) #posex, posey, time
    
    for idx in range(dataframe.shape[0]):
        
        result[idx][0:2] = pose_to_array(dataframe.iloc[idx]['Arm Execution Start Pose'])[:2] #ignoring yaw
        result[idx][2] = dataframe.iloc[idx]['Arm Execution Duration'] + include_navigation_time * dataframe.iloc[idx]['Base Navigation Duration']
        
    return result

def get_vis_times(pose_with_times, x_low = -0.5, x_high = 0, y_low = 1.55, y_high = 1.7, num_bins = 10):
    #returns an digitized array of average times, along with number of samples aslong that bin
    
    x = np.linspace(x_low, x_high, num_bins)
    y = np.linspace(y_low, y_high, num_bins)
    epsilon = 1e-6 #for clipping
    np.clip(pose_with_times[:, 0:2], np.array([x_low + epsilon , y_low + epsilon]), 
            np.array([x_high - epsilon, y_high - epsilon]), out = pose_with_times[:, 0:2]) #clip within range
    
    time_val = np.zeros([np.prod(x.shape), np.prod(y.shape), 2]) #last dimension holds the counts of values in this
    
    #probably there is a faster, vectorized way to do this
    for datapoint in pose_with_times:
            
        x_bin = np.digitize(datapoint[0], x)
        y_bin = np.digitize(datapoint[1], y)
        time_val[y_bin][x_bin][0] += datapoint[2] #note x goes rightways to match gazebo
        time_val[y_bin][x_bin][1] += 1
        
    time_val[:, :, 0]/= time_val[:, :, 1]
    
    return time_val[:, :, 0].copy(), time_val[:, :, 1].copy()
    

        
    