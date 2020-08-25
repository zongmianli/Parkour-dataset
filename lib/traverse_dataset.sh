#!/bin/bash

# ------------------------------------------------------------
# Configuration

option=${1}
parkour_laas="/path/to/Parkour-LAAS/data" #TODO
parkour_dataset="/path/to/Parkour_dataset/repo" #TODO

# ------------------------------------------------------------
# Execution

exec_path="${parkour_dataset}/lib/traverse_action.sh"

actions=('kv' 'mu' 'pu' 'sv')
num_actions=${#actions[@]}

for i in $( seq 0 $(( ${num_actions} - 1 )) )
do
    action=${actions[i]}
    echo source ${exec_path} ${option} ${parkour_laas} ${parkour_dataset} ${action}
done
