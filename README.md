# The LAAS Parkour dataset

The *LAAS Parkour dataset* contains 28 RGB videos capturing human subjects performing four typical parkour techniques: *safety-vault*, *kong vault*, *pull-up* and *muscle-up*.
These are highly dynamic motions with rich contact interactions with the environment.

The dataset is provided with the *ground truth 3D positions* of 16 pre-defined human joints, the *contact states* of the human subjects' hand and foot joints together with the *contact forces* exerted by the environment.

This dataset is based on the [LAAS Parkour MoCap database](https://gepettoweb.laas.fr/parkour/) which is originally created for biomechanics research. 
We have extracted the most relevant videos from LAAS Parkour MoCap database and provided additional script to process it. 
The database is composed of a set of raw sequence data captured with a Vicon MoCap system and force sensors.

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
Unless otherwise specified in this page, the local path to the cloned repo is denoted by the symbol `${parkour_dataset}`.

### (Optional) Extracting frame images from video
In terminal, run the following script to convert all videos in the dataset to image sequences. 
The resulting images are saved in the folder `${parkour_dataset}/frames/`.
```terminal
cd ${parkour_dataset}
source lib/video_to_frames.sh ${parkour_dataset}
```

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

For clarity, we refer to the original LAAS Parkour MoCap database as *Parkour-MoCap* and make the distinction between the *Parkour-MoCap* database and the present *Parkour dataset* used for evaluating 3D motion and force estimations.

This section is aimed at reproducing the *Parkour dataset* from the original *Parkour-MoCap* data.

### Download Parkour-MoCap data

First of all, [download](https://gepettoweb.laas.fr/parkour/) and decompress the motion files and the original videos in a local machine.
Create a local folder named *Parkour-MoCap* with two subfolders `c3d` and `videos` to host the motion files and the videos, respectively.
Similarly to `${parkour_dataset}`, we use the symbol `${parkour_mocap}` to denote the path to the *Parkour-MoCap* folder.

### Extracting frame images from Parkour-MoCap video
Run the following script in terminal:
```terminal
cd ${parkour_dataset}
source lib/video_to_frames.sh ${parkour_mocap}
```
The output images are saved under `${parkour_mocap}/frames/`.

### Computing ground truth 3D motion and contact forces

To do the job, run
```terminal
source lib/traverse_dataset.sh extract_motion_forces
```
This command saves the frame images for Parkour dataset, computes 3D person joint positions from raw marker positions and local contact forces from the original analog channels read from C3D files.
The output frame images are saved under `${parkour_dataset}/frames/`.
The ground-truth 3D motion and contact forces are saved under `${parkour_dataset}/gt_motion_forces/`.

Note that the length of image sequences in `${parkour_dataset}/frames` are shorter than the original ones saved in `${parkour_mocap}/frames`.
For example, the sequence *kv01_PKFC* which originally has 127 frames is shortened to 34 frames.
This is becuase for each Parkour-MoCap video, only a short period of time (34 frames in the case of *kv01_PKFC*) is recorded with MoCap (e.g. marker positions, raw forces).
For this reason, we remove the frames where Mocap data is missing (127-34=93 frames in the case of *kv01_PKFC*) and rename the remaining frames with new indices.
