import btk
import logging
import argparse
import shutil
import time
import cv2 as cv
import numpy as np
import numpy.linalg as LA
import cPickle as pk
import pinocchio as se3
from pinocchio.utils import eye,zero,se3ToXYZQUAT
from os import makedirs, remove
from os.path import join, exists, abspath, dirname, basename, isfile

from helpers.display import PointCloud, Viewer
from helpers.io_utils import check_and_save_data
from helpers.maths import rotation_matrix

class ParkourDataExtractor(object):

    def __init__(self, video_name, parkour_laas, parkour_dataset,
                 visualize=False):

        self.video_name = video_name
        self.parkour_laas = parkour_laas
        self.parkour_dataset = parkour_dataset

        # Load c3d file with BTK
        self.c3d_path = join(parkour_laas, "c3d", video_name+".c3d")
        print("  Loading c3d data from {}".format(self.c3d_path))
        self.reader = btk.btkAcquisitionFileReader()
        self.reader.SetFilename(self.c3d_path)
        self.reader.Update()
        self.acq = self.reader.GetOutput()
        self.fps = float(self.acq.GetPointFrequency())
        self.fps_force = float(self.acq.GetAnalogFrequency())
        self.first_frame = self.acq.GetFirstFrame()-1 # count from 0
        self.last_frame = self.acq.GetLastFrame()-1 # count from 0
        self.num_frames = self.acq.GetPointFrameNumber()
        self.num_frames_forces = self.acq.GetAnalogFrameNumber()

        # Obtain video information
        video_path = join(parkour_laas, "videos", video_name+".mp4")
        self.video = cv.VideoCapture(video_path)
        self.fps_video = float(self.video.get(cv.CAP_PROP_FPS))
        self.first_frame_in_video = \
            self.MotionFrameIdToVideoFrameId(self.first_frame)
        self.last_frame_in_video = \
            self.MotionFrameIdToVideoFrameId(self.last_frame)
        self.num_frames_in_video = \
            self.last_frame_in_video - self.first_frame_in_video + 1

        # Stardard marker set (with marker IDs)
        self.marker_names = (
            "SEL",   # 0
            "OCC",   # 1
            "LTEMP", # 2
            "RTEMP", # 3
            "STR",   # 4
            "LA",    # 5
            "RA",    # 6
            "C7",    # 7
            "T10",   # 8
            "SUP",   # 9
            "LLHE",  # 10
            "RLHE",  # 11
            "LMHE",  # 12
            "RMHE",  # 13
            "LUS",   # 14
            "RUS",   # 15
            "LRS",   # 16
            "RRS",   # 17
            "LHMH5", # 18
            "RHMH5", # 19
            "LHMH2", # 20
            "RHMH2", # 21
            "LFT3",  # 22
            "RFT3",  # 23
            "LASIS", # 24
            "RASIS", # 25
            "LPSIS", # 26
            "RPSIS", # 27
            "LGT",   # 28
            "RGT",   # 29
            "LLFE",  # 30
            "RLFE",  # 31
            "LMFE",  # 32
            "RMFE",  # 33
            "LATT",  # 34
            "RATT",  # 35
            "LSPH",  # 36
            "RSPH",  # 37
            "LLM",   # 38
            "RLM",   # 39
            "LCAL",  # 40
            "RCAL",  # 41
            "LMFH1", # 42
            "RMFH1", # 43
            "LTT2",  # 44
            "RTT2",  # 45
            "LMFH5", # 46
            "RMFH5")  # 47

        # Names of the joints considered in the Parkour dataset
        self.joint_names = (
            "l_hip",      # 0
            "l_knee",     # 1
            "l_ankle",    # 2
            "l_toes",     # 3
            "r_hip",      # 4
            "r_knee",     # 5
            "r_ankle",    # 6
            "r_toes",     # 7
            "l_shoulder", # 8
            "l_elbow",    # 9
            "l_wrist",    # 10
            "l_fingers",  # 11
            "r_shoulder", # 12
            "r_elbow",    # 13
            "r_wrist",    # 14
            "r_fingers")  # 15

        # Mapping from person joints to markers
        self.joint_marker_mapping = {
            "l_hip": ("LGT", "LASIS", "LPSIS"),
            "l_knee": ("LLFE", "LMFE"),
            "l_ankle": ("LLM", "LSPH"),
            "l_toes": ("LMFH1", "LMFH5"),
            "r_hip": ("RGT", "RASIS", "RPSIS"),
            "r_knee": ("RLFE", "RMFE"),
            "r_ankle": ("RLM", "RSPH"),
            "r_toes": ("RMFH1", "RMFH5"),
            "l_shoulder": ("LA",),
            "l_elbow": ("LLHE", "LMHE"),
            "l_wrist": ("LRS", "LUS"),
            "l_fingers": ("LHMH2", "LHMH5"),
            "r_shoulder": ("RA",),
            "r_elbow": ("RLHE", "RMHE"),
            "r_wrist": ("RRS", "RUS"),
            "r_fingers": ("RHMH2", "RHMH5")}

        # Names of the contact forces considered in the Parkour dataset
        self.contact_force_names = (
            "l_ankle", "r_ankle", "l_fingers", "r_fingers")

        # Names of the analog channels
        self.analog_names = (
            ("Fx1", "Force.Fx1"),
            ("Fy1", "Force.Fy1"),
            ("Fz1", "Force.Fz1"),
            ("Mx1", "Moment.Mx1"),
            ("My1", "Moment.My1"),
            ("Mz1", "Moment.Mz1"),
            ("Fx2", "Force.Fx2"),
            ("Fy2", "Force.Fy2"),
            ("Fz2", "Force.Fz2"),
            ("Mx2", "Moment.Mx2"),
            ("My2", "Moment.My2"),
            ("Mz2", "Moment.Mz2"),
            ("Fx3", "Force.Fx3"),
            ("Fy3", "Force.Fy3"),
            ("Fz3", "Force.Fz3"),
            ("Mx3", "Moment.Mx3"),
            ("My3", "Moment.My3"),
            ("Mz3", "Moment.Mz3"),
            ("Fx4", "Force.Fx4"),
            ("Fy4", "Force.Fy4"),
            ("Fz4", "Force.Fz4"),
            ("Mx4", "Moment.Mx4"),
            ("My4", "Moment.My4"),
            ("Mz4", "Moment.Mz4"))

        self.num_markers = len(self.marker_names)
        self.num_joints = len(self.joint_names)
        self.num_contact_forces = len(self.contact_force_names)

        # ------------------------------------------------------------------
        # Create visuals with Gepetto-viewer

        if visualize:
            self.viewer = Viewer()

            # Human joint visuals
            self.joint_colors = [
                [205,0,255], # l_leg pink
                [205,0,255],
                [205,0,255],
                [205,0,255],
                [0,255,255], # r_leg cyan
                [0,255,255],
                [0,255,255],
                [0,255,255],
                [255,215,0], # l_arm yellow
                [255,215,0],
                [255,215,0],
                [255,215,0],
                [127,255,0], # r_arm green
                [127,255,0],
                [127,255,0],
                [127,255,0]]
            self.joint_visuals = PointCloud(
                'joint_visuals',
                self.viewer.gui,
                names=self.joint_names,
                colors=self.joint_colors,
                opacity=1.,
                size=0.02)

            # Force visuals
            self.force_colors = \
                [[1.,0.,0.,.9], # linear force: red
                 [0.,0.,1.,.9], # moment: blue
                 [0.,0.,0.,1.]] # no aquisition: black
            self.force_arrow_radius = .01
            # Initialize arrow length with an arbitary small value
            force_arrow_len = .01
            for i in range(self.num_contact_forces):
                linear_force_name = "force_"+self.contact_force_names[i]
                torque_name = "torque_"+self.contact_force_names[i]
                self.viewer.gui.addArrow(
					"world/"+linear_force_name,
                    self.force_arrow_radius,
                    force_arrow_len,
                    self.force_colors[0])
                self.viewer.gui.addArrow(
					"world/"+torque_name,
                    self.force_arrow_radius,
                    force_arrow_len,
                    self.force_colors[1])


    def MotionFrameIdToVideoFrameId(self, frame_id):
        '''
        Convert the index of a frame in motion channel (aka point channel)
        to the index of the video frame corresponding to the same time step
        '''
        return int(self.fps_video/self.fps*frame_id)


    def ForceFrameIdToVideoFrameId(self, frame_id_force):
        '''
        Convert the index of a frame in force channel (aka analog channel)
        to the index of the video frame corresponding to the same time step
        '''
        return int(self.fps_video/self.fps_force*frame_id_force)


    def CopyImagesFromParkourLaas(self, source_folder=None,
                                  target_folder=None, offset=3):
        '''
        Extract subset of frame images from the original sequence of video
        frames. The image sequence is aligned with Mocap sequence with a
        temporal translation (offset) of three frames.
        '''

        if source_folder is None:
            source_folder = join(self.parkour_laas, "frames",
                                 self.video_name)

        if target_folder is None:
            target_folder = join(self.parkour_dataset, "frames",
                                 self.video_name)

        for i in range(self.num_frames_in_video-offset):
            fid = self.first_frame_in_video+offset+i
            print("  Saving frame image #{0} ({1}/{2})".format(
                fid+1, i+1, self.num_frames_in_video-offset))
            image_path = join(source_folder,
                              "{0:04d}.png".format(fid+1)) # count from 1
            save_path = join(target_folder,
                             "{0:04d}.png".format(i+1)) # count frame # from 1
            if not exists(target_folder):
                makedirs(target_folder)
            shutil.copy(image_path, save_path)


    def ExtractMarkerPositionsFromC3d(self, save_result=False):
        '''
        Extracts the 3D positions of the 48 standard markers from a c3d file,
        then downsamples the sequence to the same length as the video
        '''

        # Extract raw marker positionss
        raw_marker_positions = np.zeros(
            (self.num_frames, self.num_markers, 3))
        missing_markers = []
        for i in range(self.num_markers):
            marker_name = self.marker_names[i]
            try:
                marker = self.acq.GetPoint(marker_name)
                raw_marker_positions[:, i, :] = \
                    marker.GetValues() # nframes x 3 array
            except RuntimeError:
                print("  * Missing marker {}".format(marker_name))
                missing_markers.append(marker_name)
                raw_marker_positions[:, i, :] = float('nan')

        raw_marker_positions /= 1000. # convert mm to meters
        self.missing_markers = missing_markers
        self.raw_marker_positions = raw_marker_positions

        # Downsample the raw marker positions
        marker_positions = np.zeros(
            (self.num_frames_in_video, self.num_markers, 3))
        downsample_rate = round(self.fps/self.fps_video) # 400/25.==16.
        marker_positions_downsampled = \
            self.raw_marker_positions[::int(downsample_rate)]
        num_frames_downsampled = marker_positions_downsampled.shape[0]
        if num_frames_downsampled == self.num_frames_in_video:
            marker_positions = marker_positions_downsampled
        elif num_frames_downsampled == self.num_frames_in_video + 1:
            marker_positions = marker_positions_downsampled[:-1]
        elif num_frames_downsampled == self.num_frames_in_video - 1:
            marker_positions[:-1] = marker_positions_downsampled
            marker_positions[-1] = self.raw_marker_positions[-1]
        else:
            raise ValueError("Should never happen!")
        self.marker_positions = marker_positions


    def InferJointPositionsFromMarkerPositions(self, marker_positions,
                                               save_result=False):
        '''
        Given a marker_positions array of size (num_frames x 48 x 3),
        this function computes the joint positions by averaging
        the positions of the associated markers. The marker association
        is supposed to be provided by self.joint_marker_mapping
        '''

        num_frames = marker_positions.shape[0] # == self.num_frames_in_video
        joint_3d_positions = np.zeros((num_frames, self.num_joints, 3))

        bad_capture = False
        for j in range(self.num_joints):
            joint_name = self.joint_names[j]
            count = 0
            for marker_name in self.joint_marker_mapping[joint_name]:
                if marker_name in self.missing_markers:
                    bad_capture = True
                    print("  * Bad capture with missing marker {0}".format(
                        marker_name))
                    break
                id_marker = self.marker_names.index(marker_name)
                joint_3d_positions[:, j, :] += \
                    marker_positions[:, id_marker, :]
                count += 1
            if bad_capture:
                break
            joint_3d_positions[:, j, :] /= count
        self.joint_3d_positions = joint_3d_positions

        # Save results
        if save_result and not bad_capture:
            results = {"joint_3d_positions": self.joint_3d_positions,
                       "fps": self.fps_video,
                       "joint_names": self.joint_names,
                       "joint_marker_mapping": self.joint_marker_mapping}
            save_path = join(self.parkour_dataset, "gt_motion_forces",
                             "joint_3d_positions.pkl")
            check_and_save_data(results, save_path, self.video_name)
            print('  Joint positions saved to {}'.format(save_path))


    def ExtractForcesFromC3d(self, plate_contact_force_map):
        '''
        Extracts raw 6D contact forces from a c3d file, then downsamples the
        sequence to the same length as the video.
        '''
        num_forceplates = len(plate_contact_force_map)
        if num_forceplates != 4:
            raise ValueError("num_forceplates != 4")
        raw_contact_forces = np.zeros((
            self.num_frames_forces, num_forceplates, 6))
        missing_analog = []
        for i in range(num_forceplates):
            for k in range(6):
                num_possible_names = len(self.analog_names[6*i+k])
                for n in range(num_possible_names):
                    try_name = self.analog_names[6*i+k][n]
                    try:
                        analog = self.acq.GetAnalog(try_name)
                        raw_contact_forces[:,i,k] += \
                            analog.GetValues().reshape(-1) # nframes array
                        break
                    except RuntimeError:
                        if n == num_possible_names - 1:
                            print("  * Missing analog entry: {}".format(
                                self.analog_names[6*i+k][0]))
                            missing_analog.append(
                                self.analog_names[6*i+k][0])
                            raw_contact_forces[:,i,k] = float('nan')

        # Moments: N.mm -> N.m
        raw_contact_forces[:,:,3:6] /= 1000.
        self.missing_analog = missing_analog
        self.raw_contact_forces = raw_contact_forces

        # Downsample the raw contact forces
        contact_forces = np.zeros((
            self.num_frames_in_video, num_forceplates, 6))
        downsample_rate = round(self.fps_force/self.fps_video) # 2000/25.==80.
        contact_forces_downsampled = \
            self.raw_contact_forces[::int(downsample_rate)]
        num_frames_downsampled = contact_forces_downsampled.shape[0]
        if num_frames_downsampled == self.num_frames_in_video:
            contact_forces = contact_forces_downsampled
        elif num_frames_downsampled == self.num_frames_in_video + 1:
            contact_forces = contact_forces_downsampled[:-1]
        elif num_frames_downsampled == self.num_frames_in_video - 1:
            contact_forces[:-1] = contact_forces_downsampled
            contact_forces[-1] = self.raw_contact_forces[-1]
        else:
            raise ValueError("Should never happen!")

        # Reduce noise
        for i in range(num_forceplates):
            f_mag = LA.norm(contact_forces[:,i,:3], axis=1)
            flag_rm = f_mag < 30.
            for n,do_rm in enumerate(flag_rm):
                if do_rm:
                    contact_forces[n,i,:] = 0.

        self.contact_forces = contact_forces


    def TransformForcesToJointLocalFrames(self, contact_forces,
                                          plate_contact_force_map,
                                          contact_force_joint_map,
                                          save_result=False):
        '''
        Express each contact force w.r.t. the "local frame" of the contact
        point, i.e. the coordinate frame whose origin overlaps the point
        of contact and whose axis are in parallel with the world frame.
        '''

        self.contact_force_joint_map = contact_force_joint_map

        oM_forceplates = self.ComputeForceplateDisplacements()

        contact_forces_local = np.zeros((
            self.num_frames_in_video, self.num_contact_forces, 6))

        for k in range(self.num_contact_forces):
            # Get contact joint id and force plate id using contact force id
            joint_id = contact_force_joint_map[k]
            forceplate_ids = []

            for fpid,fid in enumerate(plate_contact_force_map):
                # fpid: force platform id
                # fid: contact force id
                if fid==k+1:
                    forceplate_ids.append(fpid)

            if len(forceplate_ids)==0:
                continue

            for i in range(self.num_frames_in_video):
                p_c = self.joint_3d_positions[i, joint_id, :] # in meters

                force_wrt_c = se3.Force(zero(6))
                for fpid in forceplate_ids:
                    # Compute forceplate displacement wrt person joint
                    cR_plate = oM_forceplates[fpid].rotation.copy()
                    p_plate = oM_forceplates[fpid].translation.copy()

                    if fpid in [0,1]: # force plates on the floor
                        relative_position = p_plate - np.matrix(p_c).T
                        cM_plate = se3.SE3(cR_plate, relative_position)
                    elif fpid in [2,3]: # force sensors on the shelf
                        cM_plate = se3.SE3(cR_plate, zero(3))
                    else:
                        raise ValueError("Should never happen!")

                    # Express the 6D force k in person joint's local frame
                    force_wrt_plate = se3.Force(
                        np.matrix(self.contact_forces[i, fpid, :]).T)
                    force_wrt_c += cM_plate.act(force_wrt_plate)

                contact_forces_local[i, k, :] = \
                    force_wrt_c.vector.getA().reshape(-1)


        self.contact_forces_local = contact_forces_local
        # save the results to file
        if save_result:
            results = {"contact_forces_local": self.contact_forces_local,
                       "contact_forces_names": self.contact_force_names,
                       "fps": self.fps_video}
            if hasattr(self, "mask_uncaptured_forces"):
                results["mask_uncaptured_forces"] = \
                    self.mask_uncaptured_forces
            save_path = join(self.parkour_dataset, "gt_motion_forces",
                             "contact_forces_local.pkl")
            check_and_save_data(results, save_path, self.video_name)
            print('  Contact force saved to {}'.format(save_path))


    def ComputeForceplateDisplacements(self):
        '''
        Computes the transformation from the world frame to the frame of
        the force platforms
        '''
        # Initialize the BTK force platforms extractor
        fpe = btk.btkForcePlatformsExtractor()
        fpe.SetInput(self.reader.GetOutput())
        fpe.Update()

        # Extract from the c3d file the corner positions of force platforms,
        # from which we can obtain the center of the force platforms
        fp_collection = fpe.GetOutput()
        num_fp = fp_collection.GetItemNumber()
        if num_fp != 4:
            raise ValueError("Should never happen!")
        forceplate_centers = []
        for k in range(num_fp):
            fp = fp_collection.GetItem(k)
            corners = fp.GetCorners() # 3 x 4 (xyz, four corners)
            center = np.mean(corners.astype(float), axis=1)
            center /= 1000. # convert mm to meters
            forceplate_centers.append(np.matrix(center).T)

        # Force platforms: orientations
        oR1 = np.matrix([[ 0, -1,  0],
                         [-1,  0,  0],
                         [ 0,  0, -1]]).astype(float)
        oR2 = oR1.copy() # is equal to oR1
        oR3 = np.matrix([[ 0,  0, -1],
                         [ 0,  1,  0],
                         [ 1,  0, 0]]).astype(float)
        oR4 = oR3.copy() # is equal to oR3
        oM_forceplates = [se3.SE3(oR1, forceplate_centers[0]),
                          se3.SE3(oR2, forceplate_centers[1]),
                          se3.SE3(oR3, forceplate_centers[2]),
                          se3.SE3(oR4, forceplate_centers[3])]
        return oM_forceplates


    def PlaySequence(self, vis_forces=False, pause=False):
        '''
        Visualize the human joints extracted from c3d.
        '''
        self.viewer.ShowXYZAxis(False)
        for i in range(self.num_frames_in_video):
            joint_3d_positions_reshaped = np.matrix(
                self.joint_3d_positions[i].reshape(-1)).T
            self.joint_visuals.Display(joint_3d_positions_reshaped)
            if vis_forces:
                for n in range(self.num_contact_forces):
                    joint_id = self.contact_force_joint_map[n]
                    oMc = se3.SE3(eye(3),
                                  np.matrix(self.joint_3d_positions[i, joint_id, :]).T)
                    cPhic = np.matrix(self.contact_forces_local[i, n, :]).T
                    if self.mask_uncaptured_forces[i,n] == 1:
                        color_linear_force = self.force_colors[2]
                    else:
                        color_linear_force = self.force_colors[0]
                    self.PlaceForceArrow(self.contact_force_names[n],
                                         oMc,
                                         cPhic,
                                         color_linear_force=color_linear_force)

            self.viewer.gui.refresh()
            time.sleep(1/self.fps_video)
            if pause:
                raw_input('#{}. Press any key to continue...'.format(i))

        self.viewer.ShowXYZAxis(True)


    def PlaceForceArrow(self, name, oMc, cPhic, color_linear_force=None,
                        refreshGui=False):
        name_linForce = "force_"+name
        name_torque = "torque_"+name
        linForce = cPhic[:3,0]
        torque = cPhic[3:,0]
        val_linForce = max(LA.norm(linForce), 1e-4)
        val_torque = max(LA.norm(torque), 1e-4)
        dir_linForce = linForce/val_linForce
        dir_torque = torque/val_torque
        dir0 = np.matrix([1.,0.,0.]).T
        oMlinForce = oMc * se3.SE3(
            rotation_matrix(dir0, dir_linForce), zero(3))
        oMtorque = oMc * se3.SE3(
            rotation_matrix(dir0, dir_torque), zero(3))
        # divid by gravity to normalize the forces
        self.viewer.gui.resizeArrow("world/"+name_linForce,
                                    self.force_arrow_radius,
                                    val_linForce/728.22)
        self.viewer.gui.resizeArrow("world/"+name_torque,
                                    self.force_arrow_radius,
                                    val_torque/728.22)
        # set color for linear force
        if color_linear_force is not None:
            self.viewer.gui.setColor("world/"+name_linForce,
                                     color_linear_force)
        self.viewer.gui.applyConfiguration("world/"+name_linForce,
                                           se3ToXYZQUAT(oMlinForce))
        self.viewer.gui.applyConfiguration("world/"+name_torque,
                                           se3ToXYZQUAT(oMtorque))
        if refreshGui:
            self.viewer.gui.refresh()


    def GenerateMaskUncapturedForces(self, missing_forces_l_ankle,
                                     missing_forces_r_ankle):
        '''
        The function converts user-annoatated time intervals of missing
        forces into a binary matrix of size (num_frames_in_video,
        num_contact_forces), where the (i, j)-th entry is one if contact
        force #j is missing at video frame #i, and zero otherwise.
        '''

        if missing_forces_l_ankle.shape[1] != 2 \
           or missing_forces_r_ankle.shape[1] != 2:
            raise ValueError("Should never happen!")

        force_ids = [0,1] # l_ankle, r_ankle
        missing_forces = [missing_forces_l_ankle, missing_forces_r_ankle]

        mask_uncaptured_forces_raw = np.zeros((
            self.num_frames, self.num_contact_forces)).astype(int)

        for i in range(len(missing_forces)):
            for n in range(missing_forces[i].shape[0]):
                f_start = missing_forces[i][n,0] - self.first_frame
                if f_start < 0:
                    raise ValueError("Should never happen!")
                if missing_forces[i][n,1] == -2:
                    f_end = self.last_frame
                elif missing_forces[i][n,1] >= 0:
                    f_end = missing_forces[i][n,1] - self.first_frame
                else:
                    raise ValueError("Should never happen!")
                mask_uncaptured_forces_raw[f_start:(f_end+1),force_ids[i]] = 1

        # Downsample mask_uncaptured_forces_raw to num_frames_in_video
        mask_uncaptured_forces = np.zeros((
            self.num_frames_in_video, self.num_contact_forces)).astype(int)
        downsample_rate = round(self.fps/self.fps_video) # 400/25. == 16.
        mask_uncaptured_forces_downsampled = \
            mask_uncaptured_forces_raw[::int(downsample_rate)]
        num_frames_downsampled = mask_uncaptured_forces_downsampled.shape[0]
        if num_frames_downsampled == self.num_frames_in_video:
            mask_uncaptured_forces = mask_uncaptured_forces_downsampled
        elif num_frames_downsampled == self.num_frames_in_video + 1:
            mask_uncaptured_forces = mask_uncaptured_forces_downsampled[:-1]
        elif num_frames_downsampled == self.num_frames_in_video - 1:
            mask_uncaptured_forces[:-1] = mask_uncaptured_forces_downsampled
            mask_uncaptured_forces[-1] = mask_uncaptured_forces_raw[-1]
        else:
            raise ValueError("Should never happen!")
        self.mask_uncaptured_forces = mask_uncaptured_forces


if __name__ == '__main__':

    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(
        description="Extracting ground truth 3D Parkour motion and contact "
        "forces from Mocap data files (*.c3d)")
    parser.add_argument(
        'parkour_laas', nargs='?', help="Path to Parkour-LAAS dataset")
    parser.add_argument(
        'parkour_dataset', nargs='?', help="Path to Parkour dataset")
    parser.add_argument(
        'video_name', nargs='?', help="Video name (without extension)")
    parser.add_argument(
        'plate_contact_force_map', nargs='?',
        help="A mapping from force plate ids (defined in c3d file) to "
        "contact force indices. For example, the string '2143' means that "
        "force plate #1, #2, #3, #4 corresponds to contact force #2, #1, #4,"
        " #3, respectively")
    parser.add_argument(
        "--missing-forces-l-ankle", type=str, default="None",
        help="Time periods in which the contact forces exerted on left foot "
        "is not accurate (due to acquisition noise, etc.). The input format "
        "is a string of frame numbers split by comma, e.g. 399,560,1093,-1, "
        "which indicates [399, 560] and [1093, end of sequnce]")
    parser.add_argument(
        "--missing-forces-r-ankle", type=str, default="None",
        help="The same as --missing-forces-l-ankle but for right foot")
    parser.add_argument(
        "--save-results", default=False, action="store_true",
        help="Save motion and forces data to file")
    parser.add_argument(
        "--visualize", default=False, action="store_true",
        help="Visualize the 3D motion and forces extracted from c3d")

    args = parser.parse_args()
    parkour_laas = args.parkour_laas
    parkour_dataset = args.parkour_dataset
    video_name = args.video_name
    plate_contact_force_map = args.plate_contact_force_map
    visualize = args.visualize
    save_results = args.save_results


    # ------------------------------------------------------------------
    # Reshape missing forces into 2D arrays of two columns

    if args.missing_forces_l_ankle != "None":
        missing_forces_l_ankle = args.missing_forces_l_ankle.split(',')
        missing_forces_l_ankle = [int(missing_forces_l_ankle[i]) \
            for i in range(len(missing_forces_l_ankle))]
        missing_forces_l_ankle = np.array(
            missing_forces_l_ankle).reshape((-1,2)) - 1 # count frame from 0
    else:
        missing_forces_l_ankle = np.zeros((0,2)).astype(int)

    if args.missing_forces_r_ankle != "None":
        missing_forces_r_ankle = args.missing_forces_r_ankle.split(',')
        missing_forces_r_ankle = [int(missing_forces_r_ankle[i]) \
            for i in range(len(missing_forces_r_ankle))]
        missing_forces_r_ankle = np.array(
            missing_forces_r_ankle).reshape((-1,2)) - 1 # count frame from 0
    else:
        missing_forces_r_ankle = np.zeros((0,2)).astype(int)


    # ------------------------------------------------------------------
    # Initialize data extractor

    worker = ParkourDataExtractor(
        video_name, parkour_laas, parkour_dataset, visualize=visualize)


    # ------------------------------------------------------------------
    # Copy a subsequence of frame images with ground truth poses and forces
    # comment this function if frames are already copied

    worker.CopyImagesFromParkourLaas()


    # ------------------------------------------------------------------
    # Extract 3D marker positions and compute joint positions

    worker.ExtractMarkerPositionsFromC3d(save_result=save_results)
    worker.InferJointPositionsFromMarkerPositions(
        worker.marker_positions, save_result=save_results)


    # ------------------------------------------------------------------
    # Extract 3D contact forces

    # Each force plate is mapped to a contact force.
    # There are five possible values in plate_contact_force_map:
    # #fid, description
    # 0, the force plate is unused and is thus mapped to nothing
    # 1, contact force on left foot (represented by l_ankle joint id #2)
    # 2, contact force on right foot (represented by r_ankle joint id #6)
    # 3, contact force on left hand (represented by l_fingers joint id #11)
    # 4, contact force on right hand (represented by r_fingers joint id #15)
    plate_contact_force_map = list(plate_contact_force_map)
    plate_contact_force_map = [int(i) for i in plate_contact_force_map]
    worker.ExtractForcesFromC3d(plate_contact_force_map)


    # ------------------------------------------------------------------
    # Generate a binary matrix indicating at which frames there are
    # uncaptured contact forces

    worker.GenerateMaskUncapturedForces(
        missing_forces_l_ankle, missing_forces_r_ankle)


    # ------------------------------------------------------------------
    # Express each of the contact forces in which human joint frames?

    contact_force_joint_map = [2, 6, 11, 15]
    worker.TransformForcesToJointLocalFrames(
        worker.contact_forces, # nf x 4 x 6
        plate_contact_force_map,
        contact_force_joint_map,
        save_result=save_results)

    if visualize:
        raw_input("  (Visualization) press Enter to play sequence ...")
        worker.PlaySequence(vis_forces=True, pause=False)

