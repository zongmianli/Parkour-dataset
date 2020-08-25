#!/bin/bash

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

dataset_dir=${1}
save_format="png"

# ------------------------------------------------------------
# Extract frame images from the original video using ffmpeg
# ------------------------------------------------------------

source lib/helpers/split_string.sh

video_names=($(ls "${dataset_dir}/videos"))
num_videos=${#video_names[@]}

echo "(video_to_frames.sh) Processing ${num_videos} videos from ${dataset_dir} ..."

for i in $(seq 0 $(( ${num_videos} - 1 )))
do
    # ------------------------------------------------------------
    # 1. Convert original videos to image sequences
    # ------------------------------------------------------------
    echo "  Processing ${video_name} ..."
    video_name=${video_names[$i]}
    input_file="${dataset_dir}/videos/${video_name}"
    # Split video_name from extension
    output_name=($(split_string ${video_name} .))
    output_path="${dataset_dir}/frames/${output_name[0]}"
    mkdir -p ${output_path}
    ffmpeg -i ${input_file} -vf yadif ${output_path}/%04d.${save_format}
done
