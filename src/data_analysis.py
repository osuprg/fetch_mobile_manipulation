#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 22 18:58:58 2020

@author: sritee
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import datetime
import utils

file_range = [1, 100]
filter_arm_failure = True
x_range = [-0.5, 0]
y_range = [1.55, 1.7]


data = []

for file_idx in range(file_range[0], file_range[1] + 1):
    
    data_part = pd.read_pickle('../logs_random/run_{}.pkl'.format(file_idx))
    data.append(data_part[(data_part.iloc[:, 0] != 0)]) #remove all zero rows -- no data stored, todo - fix hack
   
data = pd.concat(data, ignore_index = True)


log_durations = ['Base Planning', 'Base Navigation', 'Arm Planning', 'Arm Execution']


if filter_arm_failure:
   data = data.loc[data['Arm Execution End'] != 0]

data.loc[data['Arm Execution End'] == 0, 'Arm Execution End'] = datetime.datetime.now() #just set it to current time, means fail

for task in log_durations:
    
    data[task + ' Duration'] = (data[task + ' End' ] - data[task + ' Start'])
    #print(task)
    data[task + ' Duration'] = data[task + ' Duration'].apply(lambda k : k.total_seconds())


pose_with_times =  utils.extract_poses_with_times(data, include_navigation_time = False)
visualized_data, occurances = utils.get_vis_times(pose_with_times, x_low = x_range[0], x_high = x_range[1], 
                                                  y_low = y_range[0], y_high = y_range[1], num_bins = 7)

visualized_data[np.isnan(visualized_data)] = np.nanmax(visualized_data) #make nan as the max time color

visualized_data = visualized_data[1:, 1:] #drop boundaries
visualized_data_norm = visualized_data/np.max(visualized_data) #lighter means lesser time



plt.imshow(1 - visualized_data_norm)

#data.to_csv('../logs_random/results.csv')

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