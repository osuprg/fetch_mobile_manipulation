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

filter_arm_failure = True
success_plot = False
grasping_only_plot = True
navigation_and_grasping_plot = False
clip_times = True #clip within predefined range

dir_name = 'logs_33_constant_singlecanpose'
num_runs = len(os.listdir('../' + dir_name))


assert sum([success_plot, grasping_only_plot, navigation_and_grasping_plot]) == 1, 'only one value should be true'
assert(num_runs > 0)

file_range = [1, num_runs]

x_range = [-0.4, 0.1]
y_range = [1.55, 1.7]

time_range = [5, 12] #this is the range at which we will normalize  and clip the times to

can_pose = [-0.27, 2.25]

table_size = [0.85, 0.25]
table_pose = [-0.49 - table_size[0]/2, 2.1]

fontsize = 40

data = []
for file_idx in range(file_range[0], file_range[1] + 1):
    
    data_part = pd.read_pickle('../{}/run_{}.pkl'.format(dir_name, file_idx))
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

#data_ = data[data['Arm Execution Duration'] < 20]
    

pose_with_times =  utils.extract_poses_with_times(data, include_navigation_time = False)

times_clipped = np.clip(pose_with_times[:, -1], time_range[0], time_range[1])

fig = plt.figure(figsize = (20, 20))

ax = fig.add_subplot(1, 1, 1)

scatter = ax.scatter(pose_with_times[:, 0], pose_with_times[:, 1], c = times_clipped, cmap = matplotlib.cm.coolwarm_r,  s = 500)

utils.add_limits_and_labels_to_axes(ax, x_range, y_range, x_title
                                    = 'X val', y_title = 'Y val', fontsize = 40, 
                                    title = 'Pose vs Time for {} runs'.format(num_runs))

utils.add_can_and_table_to_axes(ax, can_pose, table_pose, table_size)

utils.add_colorbar(fig, ax, scatter, label = 'Times', fontsize = 40)


fig.show()

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