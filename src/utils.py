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
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1 import make_axes_locatable
from sklearn.linear_model import RANSACRegressor
from mpl_toolkits.mplot3d import Axes3D
from sklearn.linear_model import Ridge
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline



def pose_to_array(pose, orientation = False):
    """converts pose message to array of x y z, orientation includes yaw only"""
    
    
    if not orientation:
        return np.array([pose.position.x, pose.position.y, pose.position.z])
    else:
        yaw = quat_to_euler([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[-1]
        return np.array([pose.position.x, pose.position.y, pose.position.z, yaw])
    
    

def extract_poses_with_times(dataframe, include_navigation_time = False, include_arm_execution_time = True, 
                             include_arm_planning_time = True):
    
    assert(type(include_navigation_time) == type(include_arm_planning_time) == type(include_arm_execution_time) == bool)
    
    result = np.zeros([dataframe.shape[0], 3]) #posex, posey, time
    
    for idx in range(dataframe.shape[0]):
        
        result[idx][0:2] = pose_to_array(dataframe.iloc[idx]['Arm Execution Start Pose'])[:2] #ignoring yaw
        result[idx][2] = include_arm_execution_time * dataframe.iloc[idx]['Arm Execution Duration'] + \
            include_navigation_time * dataframe.iloc[idx]['Base Navigation Duration'] + \
            include_arm_planning_time * dataframe.iloc[idx]['Arm Planning Duration']
        
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


def add_can_and_table_to_axes(ax, can_pose, table_pose, table_size):
    
    ax.scatter(can_pose[0], can_pose[1], c = 'r', marker = 'x', s = 1000)
    rect = patches.Rectangle((table_pose[0],table_pose[1]),table_size[0], table_size[1],linewidth=1,edgecolor='r',facecolor='none')
    ax.add_patch(rect)
    
def add_limits_and_labels_to_axes(ax, x_range = None, y_range = None, x_title = 'X values (metres)',  y_title = 'Y values (metres)', fontsize = 40, title = 'Pose vs Times'):
    
    ax.set_title(title, fontsize = fontsize)
    
    if x_range is not None:
        ax.set_xlim(x_range[0] - 0.3, x_range[1])
    if y_range is not None:
        ax.set_ylim(y_range[0] - 0.1, y_range[1] + 0.4)
        
    ax.set_xlabel(x_title, fontsize = fontsize)
    ax.set_ylabel(y_title, fontsize = fontsize)
    
def add_colorbar(fig, ax, scatter, label = 'Times', fontsize = 40):
    
    divider = make_axes_locatable(ax)
    cax = divider.append_axes('right', size='5%', pad=0.05)
    
    cbar = fig.colorbar(scatter, cax=cax, orientation='vertical')
    cbar.set_label(label, fontsize = fontsize)
    
def get_surface_plot(fig, pose_with_times, x_range, y_range, title = 'Pose vs Times', model = 'linear', fontsize = 30):

  
    x = pose_with_times[:, 0:2]
    y = pose_with_times[:, -1]
    
    if model == 'linear':
        reg = RANSACRegressor().fit(x, y)
    elif model == 'quadratic':
        reg = make_pipeline(PolynomialFeatures(2), Ridge())
        reg.fit(x, y)
    elif model == 'cubic':
         reg = make_pipeline(PolynomialFeatures(3), Ridge())
         reg.fit(x, y)
    else:
        assert(False)
    
    xx_grid, yy_grid = np.meshgrid(np.linspace(x_range[0]  - 0.3, x_range[1], 50), np.linspace(y_range[0] - 0.1, y_range[1] + 0.1, 50))
    X_grid = np.c_[xx_grid.ravel(), yy_grid.ravel()]
    surface = reg.predict(X_grid)
  
    pred_val = reg.predict(pose_with_times[:, :2])
    color = (pose_with_times[:, -1] > pred_val)
    
    ax  = fig.add_subplot(111, projection='3d')
    ax.plot_surface(xx_grid, yy_grid, surface.reshape(-1, xx_grid.shape[0]))
 
    ax.scatter3D(pose_with_times[:, 0], pose_with_times[:, 1], pose_with_times[:, -1], c = color)

    ax.set_xlim(x_range[0] - 0.3, x_range[1])
    ax.set_ylim(y_range[0] - 0.1, y_range[1] + 0.1)
    ax.set_xlabel('X values (metres)', fontsize = 10)
    ax.set_ylabel('Y values (metres)', fontsize = 10)
    ax.set_title(title)

    return ax

def replace_text_in_file(file_name, old_text = 'worlds/empty.world', new_text = 'worlds/empty.world'):
    
    assert(type(old_text) == type(new_text) == str)
    
    with open(file_name) as f:
        newText=f.read().replace(old_text, new_text)

    with open(file_name, "w") as f:
        f.write(newText)
    
    

#def make_hist_plot(data, column = 'Arm Times', title = 'Arm Execution Histogram',
#                   xlabel = 'Time in Seconds for Planning and Execution', 
#                   ylabel = 'Frequency'):
    
        
    