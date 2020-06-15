#!/bin/bash

num_trials=$1
for ((i=1; i<=num_trials; i++)); do

   echo "========================================================\n"
   echo "This is the $i th run\n"
   echo "========================================================\n"
   python single_trial.py
done
