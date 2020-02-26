#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 22 18:58:58 2020

@author: sritee
"""

import pandas as pd
import numpy as np


file_range = [1, 20]


data = []

for file_idx in range(file_range[0], file_range[1] + 1):
    
    data_part = pd.read_pickle('../logs/run_{}.pkl'.format(file_idx))
    data.append(data_part[(data_part.iloc[:, 0] != 0)]) #remove all zero rows -- no data stored, todo - fix hack
   
data = pd.concat(data, ignore_index = True)


log_durations = ['Base Planning', 'Base Navigation', 'Arm Planning', 'Arm Execution']

data['Arm Execution Failure'] = data['Arm Execution End'] == 0
data.loc[data['Arm Execution End'] == 0, 'Arm Execution End'] = datetime.datetime.now() #just set it to current time, means fail

for task in log_durations:
    
    data[task + ' Duration'] = (data[task + ' End' ] - data[task + ' Start'])
    #print(task)
    data[task + ' Duration'] = data[task + ' Duration'].apply(lambda k : k.total_seconds())

