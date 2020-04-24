#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 22 18:58:58 2020

@author: sritee
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import datetime
import utils
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1 import make_axes_locatable
import os
import matplotlib
import glob
import time

filter_arm_failure = True #filter planning failures
filter_only_success = True
y_min_filter = 1.57

NORMALIZE_POSE_ACCORDING_TO_CAN = False #shift the exec pose linearly according to a default can pose

choices = ['logs_26_constant_singlecanpose', 'logs_33_constant_singlecanpose', 'logs_50_constant_singlecanpose',
             'hard_world_2', 'hard_world_1', 'hard_world_2_and_1']

choices_wanted = [-1]
dir_names = []

for choices_idx in choices_wanted:
    dir_names.append(choices[choices_idx])


all_plots = ['success_plot', 'navigation_and_grasping_plot', 
                 'only_navigation_plot', 'grasping_only_plot']

surface_plots = []
#surface_plots = ['linear', 'quadratic', 'cubic']

save_dir = '../results/' + dir_names[-1] + '/'

for plotted in all_plots:
    

    x_range = [-0.2, 0.2] #OVERRIDEN LATER according to can pose
    y_range = [1.55, 1.7]
    
    if plotted == 'grasping_only_plot':
        if not NORMALIZE_POSE_ACCORDING_TO_CAN:
            title_str = 'Pose vs Arm Execution Time'
        else:
            title_str = 'Pose vs Arm Time Normalized by can offset'
        time_range = [12, 19] #this is the range at which we will normalize  and clip the times to
    
    elif plotted == 'navigation_and_grasping_plot':
        title_str = 'Pose vs (Navigation + Arm Execution) Time'
        time_range = [18, 37] #this is the range at which we will normalize  and clip the times to
    
    elif plotted == 'only_navigation_plot':
        title_str = 'Pose vs Navigation Time '
        time_range = [10, 24] #this is the range at which we will normalize  and clip the times to
    
    elif plotted == 'success_plot':
        title_str = 'Pose vs Success'
    else:
        assert(False)
    
    can_pose = [-0.27, 2.25] #OVERRIDEN LATER
    
    table_size = [0.85, 0.25]
    table_pose = [-0.49 - table_size[0]/2, 2.1]
    
    fontsize = 40
    
    dataframe_lists = []
    run_per_log = []
    
    for dir_name in dir_names:
        data = []
        num_runs = len(glob.glob('../' + dir_name + '/*.pkl'))
        run_per_log.append(num_runs)
        assert(num_runs > 0)
        
        for file_idx in range(1, num_runs + 1):
            
            data_part = pd.read_pickle('../{}/run_{}.pkl'.format(dir_name, file_idx))
            data.append(data_part[(data_part.iloc[:, 0] != 0)]) #remove all zero rows -- no data stored, todo - fix hack
        
        data = pd.concat(data, ignore_index = True)
        
        if filter_arm_failure:
            data = data.loc[data['Arm Execution End'] != 0]
        if filter_only_success:
            data = data.loc[data['Success'] == 1]
        
        run_per_log[-1] = data.shape[0]
        
        dataframe_lists.append(data)
    
    
    data = pd.concat(dataframe_lists, ignore_index = True)
    
    
    log_durations = ['Base Planning', 'Base Navigation', 'Arm Planning', 'Arm Execution']
    

#    if filter_arm_failure:
#       data = data.loc[data['Arm Execution End'] != 0]
#    if filter_only_success:
#       data = data.loc[data['Success'] == 1]
       
    num_runs = data.shape[0] #cumulative length
       
    can_pose = utils.pose_to_array(data.iloc[0]['Can Pose'].pose)
    default_can_pose = utils.pose_to_array(data.iloc[0]['Can Pose'].pose)
    x_range = [can_pose[0] - 0.4, can_pose[0] + 0.4]
    
    data.loc[data['Arm Execution End'] == 0, 'Arm Execution End'] = datetime.datetime.now() #just set it to current time, means fail
    
    for task in log_durations:
        
        data[task + ' Duration'] = (data[task + ' End' ] - data[task + ' Start'])
        #print(task)
        data[task + ' Duration'] = data[task + ' Duration'].apply(lambda k : k.total_seconds())
    
    #data_ = data[data['Arm Execution Duration'] < 20]
    data = data[data.apply(lambda r : r['Base Planning Goal'][1] > y_min_filter, axis = 1)]
    data.reset_index(inplace = True, drop = True)
        
    
    pose_with_times =  utils.extract_poses_with_times(data, include_navigation_time = ((plotted == 'navigation_and_grasping_plot')
               or (plotted =='only_navigation_plot')) , include_arm_execution_time = not(plotted == 'only_navigation_plot'),
    include_arm_planning_time = True)
    
    if NORMALIZE_POSE_ACCORDING_TO_CAN and plotted == 'grasping_only_plot':
        
        for i in range(len(pose_with_times)):
            can_offset = default_can_pose[:2] - utils.pose_to_array(data.iloc[i]['Can Pose'].pose)[:2]  
            pose_with_times[i, 0:2] += can_offset
     
    
    if plotted == 'success_plot':
        val_plotted = data['Success']
    else:
        val_plotted = np.clip(pose_with_times[:, -1], time_range[0], time_range[1])
    
    fig = plt.figure(figsize = (20, 20))
    
    ax = fig.add_subplot(1, 1, 1)
    
    scatter = ax.scatter(pose_with_times[:, 0], pose_with_times[:, 1], c = val_plotted, cmap = matplotlib.cm.coolwarm_r,  s = 500)
    
    
    
    utils.add_limits_and_labels_to_axes(ax, x_range, y_range, x_title
                                        = 'X val', y_title = 'Y val', fontsize = 40, 
                                        title = title_str  + ' for {} runs'.format(num_runs))
    
    utils.add_can_and_table_to_axes(ax, can_pose, table_pose, table_size)
    
    if not plotted == 'success_plot':
        utils.add_colorbar(fig, ax, scatter, label = 'Time (s)', fontsize = 40)
    else:
        utils.add_colorbar(fig, ax, scatter, label = 'Success bool', fontsize = 40)
        
    
    
    #fig.show()
    if not os.path.exists(save_dir):
        os.mkdir(save_dir)
        
    fig.savefig(save_dir + plotted + '_' + str(num_runs) + filter_only_success * '_success_filtered' + '.png')
    plt.close()
      
    #histogram plotting
    fig = plt.figure(figsize = (10, 10))
    
    ax = fig.add_subplot(1, 1, 1)
    
    if plotted == 'success_plot':
        ax.hist((val_plotted.values) * 1.0) #success
        utils.add_limits_and_labels_to_axes(ax, x_title = 'Success Bool', y_title = 'Number of Occurances',
                                        title = plotted + filter_only_success * '_success_filtered_ ' + 'histogram',
                                        fontsize = 20)
    else:
        ax.hist(pose_with_times[:, -1]) #Unclipped times histogram
        utils.add_limits_and_labels_to_axes(ax, x_title = 'Time (s)', y_title = 'Number of Occurances',
                                        title = plotted + filter_only_success * '_success_filtered_ ' + 'histogram',
                                        fontsize = 20)
    
    time.sleep(0.25)
    fig.savefig(save_dir + 'histogram_' +  plotted + '_' + str(num_runs) + filter_only_success * '_success_filtered' +  '.png')
  
    plt.close()
   
    for model in surface_plots:
        fig = plt.figure(figsize = (20, 20)) #new 3d figure
        
        utils.get_surface_plot(fig, pose_with_times, x_range, y_range, title = title_str, model = model)
        fig.savefig(save_dir + plotted + '_' + model + '_surface_' + str(num_runs) + '.png')
    #plt.show()

means = []
means_plan = []
medians = []
var = []
start_idx = 0
chunk = []
suc = []

for i in range(len(run_per_log)):
    
    print(start_idx, start_idx + run_per_log[i])
    data_chunk = data.iloc[start_idx : start_idx + run_per_log[i]]
    data_chunk_rel = pose_with_times[start_idx : start_idx + run_per_log[i]][:, -1] #This holds arm grasping time
    means_plan.append(data_chunk['Arm Planning Duration'].mean())
    means.append(np.mean(data_chunk_rel))
    var.append(np.std(data_chunk_rel))
    medians.append(np.median(data_chunk_rel))
    suc.append(data_chunk['Success'].mean())
    start_idx = np.sum(run_per_log[0:i + 1]).astype('int32')


print('done')

'''
Available columns for reference
Index([u'Base Planning Start', u'Base Planning End', u'Base Planning Goal',
       u'Base Navigation Start', u'Base Navigation End',
       u'Base Navigation Start Pose', u'Base Navigation End Pose',
       u'Arm Planning Start', u'Arm Planning End', u'Arm Planning Success',
       u'Arm Execution Start Pose', u'Arm Execution End Pose',
       u'Arm Execution Start', u'Arm Execution End', u'Success',
       u'Base Planning Duration', u'Base Navigation Duration',
       u'Arm Planning Duration', u'Arm Execution Duration'],
      dtype='object')

'''