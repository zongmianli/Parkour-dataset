#!/bin/bash

# ------------------------------------------------------------
# Configuration

option=${1}
parkour_mocap=${2}
parkour_dataset=${3}
action=${4}

# ------------------------------------------------------------
# Execution

action_known=1
if [ ${action} = 'kv' ]
then
    # 7 "two-hand jump" sequences
    video_names=(
        'kv01_PKFC' 'kv01_PKLP' 'kv02_PKLP' 'kv03_PKLP'  'kv04_PKLP' \
        'kv05_PKLP' 'kv06_PKLP')
    plate_contact_force_map=(
        '1234' '1234' '2143' '1234' '2143' \
        '1234' '2143')
    missing_forces_l_ankle=(
        'None' 'None' '2368,2462' 'None' '1806,1981' \
        'None' '2272,2420')
    missing_forces_r_ankle=(
        'None' 'None' '2368,2462' '2048,2312' '1806,1981' \
        '2445,2172' '2272,2420')
elif [ ${action} = 'mu' ]
then
    # 6 "move-up" sequences
    video_names=(
        'mu02_PKLP' 'mu03_PKLP' 'mu04_PKLP' 'mu05_PKFC' 'mu09_PKLP' \
        'mu11_PKFC')
    plate_contact_force_map=(
        '1234' '1234' '1234' '1234' '1234' \
        '1234')
    missing_forces_l_ankle=(
        'None' 'None' 'None' 'None' 'None' \
        'None')
    missing_forces_r_ankle=(
        '1740,1890' '1711,2081' '1821,1985' 'None' '403,517'   \
        'None')
elif [ ${action} = 'pu' ]
then
    # 6 "pull-up" sequences
    video_names=(
        'pu02_PKLP' 'pu02_PKLT' 'pu03_PKLP' 'pu03_PKLT' 'pu04_PKLP' \
        'pu04_PKLT')
    plate_contact_force_map=(
        '1234' '1234' '1234' '1234' '1234' \
        '1234')
    missing_forces_l_ankle=(
        'None' 'None' 'None' 'None' 'None' \
        'None')
    missing_forces_r_ankle=(
        '1581,1826,1885,1958' '1303,1840' '1464,1689,1792,1892,2671,-1' '1860,2185' '1577,1730,1866,1926,2664,-1' \
        '1300,1738' )
elif [ ${action} = 'sv' ]
then
    # 9 "one-hand hop" sequences
    video_names=(
        'sv01_PKFC' 'sv02_PKFC' 'sv03_PKFC' 'sv03_PKLP' 'sv03_PKMT' \
        'sv04_PKFC' 'sv04_PKMT' 'sv05_PKLP' 'sv07_PKMT')
    plate_contact_force_map=(
        '1032' '2123' '1032' '2123' '2123' \
        '2123' '1032' '2123' '2123')
    missing_forces_l_ankle=(
        '1093,-1' '734,754' '1146,-1'   '1817,1866,2363,-1' '962,1039' \
        '490,525' 'None'    '1543,1612' '1272,1347')
    missing_forces_r_ankle=(
        '399,560,1093,-1' 'None'    '398,672,1153,-1' '2363,-1' 'None' \
        'None'            '183,353' 'None'            'None')
else
    echo "traverse_action.sh: Unknown action (${action})"
    action_known=0
fi


if [ ${action_known} -eq 1 ]
then
    num_videos=${#video_names[@]}
    for i in $( seq 0 $(( ${num_videos} - 1 )) )
    do
        video_name=${video_names[$i]}
        echo "(traverse_action.sh) Processing ${video_name} ..."

        if [ ${option} = 'extract_motion_forces' ]
        then
            python lib/data_extractor.py ${parkour_mocap} ${parkour_dataset} ${video_name} ${plate_contact_force_map[$i]} --missing-forces-l-ankle=${missing_forces_l_ankle[$i]} --missing-forces-r-ankle=${missing_forces_r_ankle[$i]} --save-results
        fi
    done
fi


