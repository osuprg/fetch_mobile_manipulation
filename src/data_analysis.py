#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 22 18:58:58 2020

@author: sritee
"""

import pandas as pd
import numpy as np


file_range = [1, 10]

#for file_idx in range(file_range[0], file_range[1]):
    
    
    


data = pd.read_pickle('run_1.pkl')
data = data[(data != 0).any(axis = 1)] #remove all zero rows -- no data stored
log_durations = ['Base Planning', 'Base Navigation', 'Arm Planning', 'Arm Execution']
#

for task in log_durations:
    
    data[task + ' Duration'] = (data[task + ' End' ] - data[task + ' Start'])
    #print(task)
    data[task + ' Duration'] = data[task + ' Duration'].apply(lambda k : k.total_seconds())

