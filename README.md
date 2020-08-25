# The Parkour dataset

## Introduction

The *Parkour dataset* contains 28 RGB videos capturing human subjects performing four typical parkour actions: *two-hand jump*, *move-up*, *pull-up* and *single-hand hop*.
These are highly dynamic motions with rich contact interactions with the environment.
The dataset is provided with *ground truth 3D motion* and *contact forces*.

## Quick setup

### Dependencies

* Python 2.7
* Pinocchio
* Gepetto-viewer Corba
* FFmpeg (optional, for converting video to images)
* BTK Python (optional, for reproducing the dataset from C3D)

### Installation

```terminal
git clone https://github.com/zongmianli/Parkour-dataset ${parkour_dataset}
cd ${parkour_dataset}
```
where `${parkour_dataset}` is the local path to a desired location for saving this dataset.

### (Optional) Converting videos to image sequences
```terminal
source lib/video_to_frames.sh ${parkour_dataset}
```
The output images are saved under `${parkour_dataset}/frames/`.

## Computing motion and force estimation errors

This dataset is provided with a Python library for evaluating the performance of 3D motion and/or force estimators.

To use the library, append `${parkour_dataset}/lib/` to `sys.path` and create a new instance of the class `ParkourEvaluator`.
For example:
```python
from parkour_evaluator import ParkourEvaluator

gt_dir = "/path/to/${parkour_dataset}/gt_motion_forces"
sequence_name = "kv01_PKFC" # for example
evaluator = ParkourEvaluator(gt_dir, sequence_name)
```

The following code computes the mean per joint position error (MPJPE) of the estimated 3D pose (`joint_3d_positions_pred`) with respect to the ground truth after rigid alignment (by solving an orthogonal Procrustes problem):
```python
evaluator.Evaluate3DPoses(joint_3d_positions_pred)
print("mpjpe: {0:.2f} mm".format(evaluator.mpjpe['procrustes']))
```

Similarly, force errors are computed like this:
```python
evaluator.EvaluateContactForces(contact_forces_pred)
print("mean linear force errors: {0} N".format(
    evaluator.mean_linear_force_errors))
print("mean torque errors: {0} N.m".format(
    evaluator.mean_torque_errors))
```

## (Optional) Reproducing Parkour dataset from C3D MoCap data

The Parkour dataset is created on top of a set of original motion capture (MoCap) data captured with a Vicon motion capture system and force sensors.
For clarity, we refer to the original MoCap data as *Parkour-LAAS* and make the distinction between Parkour-LAAS data and the Parkour dataset (this repo).

This section is aimed at reproducing the Parkour dataset from the original Parkour-LAAS data.

### Extracting frame images from video

Download the Parkour-LAAS data ([link](tbd)) and decompress it to an arbitrary location denoted by `${parkour_laas}`.
Then run the following script:
```terminal
cd ${parkour_dataset}
source lib/video_to_frames.sh ${parkour_laas}
```
The frame images are saved under `${parkour_laas}/frames/`.

### Computing ground truth 3D motion and contact forces

To do the job, run
```terminal
source lib/traverse_dataset.sh extract_motion_forces
```
This command saves the frame images for Parkour dataset, computes 3D person joint positions from raw marker positions and local contact forces from the original analog channels read from C3D files.
The output frame images are saved under `${parkour_dataset}/frames/`.
The ground-truth 3D motion and contact forces are saved under `${parkour_dataset}/gt_motion_forces/`.

Note that the length of image sequences in `${parkour_dataset}/frames` are shorter than the original ones saved in `${parkour_laas}/frames`.
For example, the sequence *kv01_PKFC* which originally has 127 frames is shortened to 34 frames.
This is becuase for each Parkour-LAAS video, only a short period of time (34 frames in the case of *kv01_PKFC*) is recorded with MoCap (e.g. marker positions, raw forces).
For this reason, we remove the frames where Mocap data is missing (127-34=93 frames in the case of *kv01_PKFC*) and rename the remaining frames with new indices.
