#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 22 18:58:58 2020

@author: sritee
"""

import pandas as pd
import numpy as np
import datetime

file_range = [1, 20]
filter_arm_failure = True


data = []

for file_idx in range(file_range[0], file_range[1] + 1):
    
    data_part = pd.read_pickle('../logs/run_{}.pkl'.format(file_idx))
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