# The LAAS Parkour dataset

The *LAAS Parkour dataset* contains 28 RGB videos capturing human subjects performing four typical parkour techniques: *safety-vault*, *kong vault*, *pull-up* and *muscle-up*.
These are highly dynamic motions with rich contact interactions with the environment.
The dataset is provided with the *ground truth 3D positions* of 16 pre-defined human joints, together with the *contact forces* at the human subjects' hand and foot joints exerted by the environment.

This dataset is based on the [LAAS Parkour MoCap database](https://gepettoweb.laas.fr/parkour/) which is originally created for biomechanics research. 
The major differences between the two datasets are summarized in the following table:

| Difference        | The LAAS Parkour MoCap database                                                                                            | The LAAS Parkour dataset (this repo)                                  |
| :---------------: | :-------------------------------                                                                                           | :------------------------                                             |
| Short name        | Parkour-MoCap                                                                                                              | Parkour-dataset                                                       |
| Size              | 65 videos                                                                                                                  | 28 shortened videos                                                   |
| Ground truth data | Raw sequence data captured with a Vicon MoCap system and force sensors (3D marker positions, force platform records, etc.) | 3D human joint positions & 6D contact forces computed from MoCap data |


## Quick setup

### Dependencies

* Python 2.7
* Pinocchio (for computing rigid-body dynamics)
* Gepetto-viewer Corba (for visualization)
* FFmpeg (optional, for converting video to images)
* BTK Python (optional, for reproducing this dataset)

### Installation

```terminal
git clone https://github.com/zongmianli/Parkour-dataset
```
In the following parts, we will use the symbol `${parkour_dataset}` to denote the local path to the dataset repository `Parkour-dataset/`.

### (Optional) Extracting frames from video
In terminal, run the following script to convert all videos in the dataset to frames. 
The output images are saved in the folder `${parkour_dataset}/frames/`.
```terminal
cd ${parkour_dataset}
source lib/video_to_frames.sh ${parkour_dataset}
```
As an alternative, we provide a set of pre-extracted frames which can be downloaded from [here](https://www.di.ens.fr/willow/research/motionforcesfromvideo/data/Parkour-dataset-frames.zip)

## Computing motion and force estimation errors

This dataset is provided with a Python library for evaluating the performance of 3D motion and/or force estimators.

To use the library, append `${parkour_dataset}/lib/` to `sys.path` and create a new instance of the class `ParkourEvaluator`.
For example:
```python
from parkour_evaluator import ParkourEvaluator

gt_dir = "${parkour_dataset}/gt_motion_forces"
sequence_name = "kv01_PKFC" # for example
evaluator = ParkourEvaluator(gt_dir, sequence_name)
```

The following code computes the mean per joint position error (MPJPE) of a set of input joint 3D positions (`joint_3d_positions_pred`) with respect to the ground truth after rigid alignment (by solving an orthogonal Procrustes problem):
```python
evaluator.Evaluate3DPoses(joint_3d_positions_pred)
print("MPJPE: {0:.2f} mm".format(evaluator.mpjpe['procrustes']))
```

Similarly, force errors are computed like this:
```python
evaluator.EvaluateContactForces(contact_forces_pred)
print("mean linear force errors: {0} N".format(
    evaluator.mean_linear_force_errors))
print("mean torque errors: {0} N.m".format(
    evaluator.mean_torque_errors))
```

## (Optional) Reproducing the dataset from MoCap data

As mentioned above, we make the distinction between the LAAS Parkour dataset (aka *Parkour-dataset*) and the original LAAS Parkour MoCap database (aka *Parkour-Mocap*).
This section is aimed at reproducing the *Parkour dataset* from the original *Parkour-MoCap* data.

### Download Parkour-MoCap data

First of all, download the original motion data and RGB videos ([download link](https://gepettoweb.laas.fr/parkour/)).
Decompress the downloaded packages into a new local repository named `Parkour-MoCap/`.
This will create two subfolders `c3d/` and `videos/` with motion files and RGB videos, respectively.
Similar to `${parkour_dataset}`, we use the symbol `${parkour_mocap}` to denote the path to the `Parkour-MoCap/` folder.

### Extracting frames from Parkour-MoCap video

Run the following script in terminal:
```terminal
cd ${parkour_dataset}
source lib/video_to_frames.sh ${parkour_mocap}
```
The output images are saved under `${parkour_mocap}/frames/`.

### Computing ground truth 3D motion and contact forces
To do the job, update line 7 and line 8 in `lib/traverse_dataset.sh` with correct paths and then run
```terminal
source lib/traverse_dataset.sh extract_motion_forces
```
This command saves the frames for the *Parkour dataset*, computes 3D person joint positions from raw marker positions and local contact forces from the original analog channels in C3D files.
The output images are saved to `${parkour_dataset}/frames/`.
The ground-truth 3D motion and contact forces are saved to `${parkour_dataset}/gt_motion_forces/`.

Note that the length of image sequences in `${parkour_dataset}/frames` are shorter than the original ones saved in `${parkour_mocap}/frames`.
For example, the sequence *kv01_PKFC* which originally has 127 frames is shortened to 34 frames.
This is because for each Parkour-MoCap video, only a short period of time (34 frames in the case of *kv01_PKFC*) was recorded with MoCap and force sensors.
In other words, the lengths of the original RGB videos are longers than the MoCap/force sequences saved in `*.c3d`.
For this reason, we chose to aligne each MoCap/force sequence with the corresponding video in time, remove the video frames with missing Mocap data (127-34=93 frames in the case of *kv01_PKFC*) and rename the remaining video frames with new indices.
