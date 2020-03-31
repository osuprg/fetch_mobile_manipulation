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

file_range = [1, 200]
filter_arm_failure = True
x_range = [-0.5, 0]
y_range = [1.55, 1.7]


data = []

can_pose = [-0.27, 2.25]

table_size = [0.85, 0.25]
table_pose = [-0.49 - table_size[0]/2, 2.1]

for file_idx in range(file_range[0], file_range[1] + 1):
    
    data_part = pd.read_pickle('../logs_200_random/run_{}.pkl'.format(file_idx))
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

data_ = data[data['Arm Execution Duration'] < 20]
pose_with_times =  utils.extract_poses_with_times(data, include_navigation_time = False)
#visualized_data, occurances = utils.get_vis_times(pose_with_times, x_low = x_range[0], x_high = x_range[1], 
#                                                  y_low = y_range[0], y_high = y_range[1], num_bins = 7)
#
#visualized_data[np.isnan(visualized_data)] = np.nanmax(visualized_data) #make nan as the max time color
#
#visualized_data = visualized_data[1:, 1:] #drop boundaries
#visualized_data_norm = visualized_data/np.max(visualized_data) #lighter means lesser time
#
#
#
#plt.imshow(1 - visualized_data_norm)

#data.to_csv('../logs_random/results.csv')



#plt.scatter(pose_with_times[:, 0], pose_with_times[:, 1], c = pose_with_times[:, -1])

norm_times = pose_with_times[:, -1]/np.max(pose_with_times[:, -1])
fig = plt.figure(figsize = (20, 20))

fontsize = 40
ax = fig.add_subplot(1, 1, 1)
ax.set_xlim(x_range[0] - 0.3, x_range[1])
ax.set_ylim(y_range[0] - 0.1, y_range[1] + 0.7)
ax.set_xlabel('X values (metres)', fontsize = fontsize)
ax.set_ylabel('Y values (metres)', fontsize = fontsize)

scatter = ax.scatter(pose_with_times[:, 0], pose_with_times[:, 1], c = pose_with_times[:, -1], s = 500)

ax.scatter(can_pose[0], can_pose[1], c = 'r', marker = 'x', s = 1000)
rect = patches.Rectangle((table_pose[0],table_pose[1]),table_size[0], table_size[1],linewidth=1,edgecolor='r',facecolor='none')
ax.add_patch(rect)
ax.set_title('Pose vs (Arm Execution + Planning Times) for 200 runs', fontsize = fontsize)

divider = make_axes_locatable(ax)
cax = divider.append_axes('right', size='5%', pad=0.05)
#cax.set_ylabel('Time in Seconds for Execution and Planning')


cbar = fig.colorbar(scatter, cax=cax, orientation='vertical')
cbar.set_label('Time in Seconds', fontsize = fontsize)

#mpl.colorbar.ColorbarBase(ax2, norm = mpl.colors.Normalize(np.min(pose_with_times[:, -1]), np.max(pose_with_times[:, -1])))
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