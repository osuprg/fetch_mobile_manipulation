#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  4 17:19:51 2020

@author: sritee


The goal of this is to log event and their times
"""


import pandas as pd
import numpy as np
import datetime
import os



class CustomLogger:
    
    
    def __init__(self, column_list = None, preallocated_size = 10):
        
        self._preallocated_size = preallocated_size
        self._trial_number = 0
        
        if column_list is None: #just ending with start/end means timestamp, otherwise position
            columns = ['Base Planning Start', 'Base Planning End', 'Base Planning Goal', 
                       'Base Navigation Start', 'Base Navigation End', 'Base Navigation Start Pose',
                       'Base Navigation End Pose', 'Arm Planning Start', 'Arm Planning End', 
                        'Arm Planning Success',
                       'Arm Execution Start Pose', 'Arm Execution End Pose'
                   , 'Arm Execution Start', 'Arm Execution End', 'Arm Execution Success', 'Cartesian Servoing Success',\
                   'Cartesian Linear Success', 'Success', 'Can Pose']
        else:
            columns = column_list
        
        self._possible_columns = set(columns)
        self._data = pd.DataFrame(np.zeros([self._preallocated_size, len(columns)]), columns = columns, dtype='object') #zero initialized
        

    def update_log(self, column_name, value = None): #updates column value, by default the current time
        
        
        assert column_name in self._possible_columns, 'invalid column name {}! possible ones are {}'.format(column_name, self._possible_columns)
        
        if value is None:
            value = datetime.datetime.now()
            
        self._data.iloc[self._trial_number][column_name] = value
        
        
    def increment_trial_number(self):
        
        self._trial_number = self._trial_number + 1
        
        '''TODO -- Resizing when we go out of size
        if self._trial_number > self._preallocated_size:
            self._preallocated_size*=2
            self.data = self.data.append(np.zeros([self._preallocated_size]))
        
        '''
        
        return self._trial_number #returning new number if the caller wants it
    
    def save(self, logdir, name, config_files = None):
        
        self._data.to_pickle(logdir + name)
        for file_name in config_files:
            os.popen('cp {} {}'.format(file_name, logdir)) #Save the config files seperately
            
    
    def __str__(self):
        
        return str(self._data)
        
        
        
                